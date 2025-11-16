import cv2
import numpy as np
import open3d as o3d
from copy import deepcopy
from scipy.optimize import least_squares


# Para não imprimir cada iteração:
# Comentar linhas 102 a 125 e 181 a 187
# Descomentar linhas 247 a 257 e 280 a 289

view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 1.622727601414635, 0.92732282434191027, 2.9839999675750732 ],
			"boundingbox_min" : [ -1.7043713967005412, -1.1394533466157459, -0.029999999999999999 ],
			"field_of_view" : 60.0,
			"front" : [ 0.54877655571863571, 0.040714697251322977, -0.83497700885792325 ],
			"lookat" : [ 0.39833293954817622, -0.11328460994204304, 1.9350827314289289 ],
			"up" : [ -0.2163456823136507, -0.95786868355247656, -0.18889714347677763 ],
			"zoom" : 0.69999999999999996
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}


#========================================================
# Funções Auxiliares para o ICP Personalizado
#========================================================
def get_transformation_matrix(params):
    """
    Converte um vetor de 6 parâmetros (rx, ry, rz, tx, ty, tz)
    numa matriz de transformação 4x4.
    Aqui, rx, ry, rz são considerados ângulos de rotação em torno dos eixos X, Y, Z.
    Para simplificar e dada a natureza incremental, podemos usar get_rotation_matrix_from_axis_angle.
    """
    
    # Os primeiros 3 parâmetros são a rotação 
    rotation_vector = params[0:3]
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_vector)
    
    T = np.identity(4)          # Matriz identidade 4x4 onde será colocada a transformação
    T[0:3, 0:3] = R             # Coloca a matriz de rotação
    T[0:3, 3] = params[3:6]     # Coloca a translação
    
    return T

def residuals_func(x, source_points_matched, target_points_matched, target_normals_matched):
    """
    Parâmetros:
    x: vetor de 6 parâmetros de transformação incremental (rx, ry, rz, tx, ty, tz).
    Estes representam a transformação que estamos a tentar otimizar NESTA iteração.
    source_points_matched: N x 3 array de pontos da fonte que têm correspondência.
    target_points_matched: N x 3 array de pontos do alvo que correspondem aos pontos da fonte.
    target_normals_matched: N x 3 array de normais dos pontos do alvo correspondentes.

    Retorna:
    Um array NumPy de resíduos individuais para cada correspondência.
    """
    
    # Converte os parâmetros x (transformação incremental) em uma matriz 4x4
    incremental_transform_matrix = get_transformation_matrix(x)

    # Aplica a transformação incremental aos pontos da fonte 
    # (multiplica os pontos da fonte com correspondência pela rotação e adiciona a translação)
    # source_points_matched é N x 3. A matriz de rotação é 3x3.
    # Para multiplicar: (3x3 @ 3xN).T + 3x1.T => N x 3
    source_points_transformed = (incremental_transform_matrix[:3, :3] @ source_points_matched.T).T + incremental_transform_matrix[:3, 3]

    # Calcula os resíduos (Point-to-Plane)
    # Erro = (s_p_transformed - t_p_matched) . t_n_matched
    # Distância do ponto transformado da fonte ao plano definido pelo ponto alvo e sua normal 
    # (distância entre os pontos multiplicada pela normal do plano - projeção)
    # np.sum(..., axis=1) soma os componentes x,y,z do produto escalar para cada ponto.
    residuals = np.sum((source_points_transformed - target_points_matched) * target_normals_matched, axis=1)
    
    return residuals


#========================================================
# Função do ICP Personalizado
#========================================================
def custom_icp(source_cloud, target_cloud, initial_transform, max_correspondence_distance, max_iterations=100, tolerance=1e-6):
    
    # Faz uma cópia da nuvem fonte para aplicar as transformações
    current_source_cloud = deepcopy(source_cloud)
    current_source_cloud.transform(initial_transform) # Aplica a transformação inicial

    # Armazena a transformação total acumulada
    total_transformation = initial_transform

    # Constrói a KDTree para a nuvem alvo (encontra os vizinhos mais próximos de cada ponto, necessário para correspondências)
    # Para o alvo não é necessário atualizar a KDTree, pois o alvo permanece fixo
    target_kdtree = o3d.geometry.KDTreeFlann(target_cloud)


    # --- Configuração do Visualizador para a Animação ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='ICP Animation', width=800, height=600)
    
    # Adiciona as nuvens de pontos (fonte e alvo) ao visualizador
    # A nuvem alvo (azul) permanece estática
    target_cloud.paint_uniform_color([0, 0, 1]) # alvo: azul
    vis.add_geometry(target_cloud)
    
    # A nuvem fonte (vermelha inicialmente) será atualizada
    current_source_cloud.paint_uniform_color([1, 0, 0]) # fonte: vermelho
    vis.add_geometry(current_source_cloud)

    # Adiciona e configura o frame de coordenadas
    axes = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
    vis.add_geometry(axes)

    # Configura a câmara do visualizador com a vista pré-definida
    ctr = vis.get_view_control()
    ctr.set_front(view['trajectory'][0]['front'])
    ctr.set_lookat(view['trajectory'][0]['lookat'])
    ctr.set_up(view['trajectory'][0]['up'])
    ctr.set_zoom(view['trajectory'][0]['zoom'])
    vis.update_renderer()



    for i in range(max_iterations):
        print(f"Iteração {i+1}/{max_iterations}")

        # --- Associação de Correspondências ---
        source_points = np.asarray(current_source_cloud.points)
        target_points_matched = []
        source_points_matched = []
        target_normals_matched = []

        for j in range(len(source_points)):
            # Para cada ponto da fonte, encontra o ponto mais próximo no alvo e armazena o seu índice e distância euclidiana ao quadrado
            [k, idx, dist_sq] = target_kdtree.search_knn_vector_3d(source_points[j], 1)
            
            if dist_sq[0] < max_correspondence_distance**2:                                     # Verifica a distância
                source_points_matched.append(source_points[j])                                  # Adiciona o ponto da fonte
                target_points_matched.append(np.asarray(target_cloud.points)[idx[0]])           # Adiciona o ponto do alvo correspondente
                target_normals_matched.append(np.asarray(target_cloud.normals)[idx[0]])         # Adiciona a normal do ponto do alvo correspondente

        if not source_points_matched:
            print("Nenhuma correspondência encontrada. ICP interrompido.")
            break

        source_points_matched = np.array(source_points_matched)
        target_points_matched = np.array(target_points_matched)
        target_normals_matched = np.array(target_normals_matched)

        # --- Estimativa da Transformação (com scipy.optimize.least_squares) ---
        # A estimativa inicial para a transformação INCREMENTAL é zero (identidade)
        initial_guess_params = np.zeros(6) 

        # Otimização
        res = least_squares(
            residuals_func,
            initial_guess_params,
            args=(source_points_matched, target_points_matched, target_normals_matched),
            method='lm',    # Algoritmo Levenberg-Marquardt
            verbose=0       # 0 para não imprimir durante a otimização
        )
        
        incremental_params = res.x          # res.x é o vetor de parâmetros [rx, ry, rz, tx, ty, tz] que least_squares encontrou
        incremental_transform_matrix = get_transformation_matrix(incremental_params)

        # Verifica a convergência
        # Calcula a norma do vetor de parâmetros incrementais, se for menor que a tolerância, considera que convergiu
        if np.linalg.norm(incremental_params) < tolerance:
            print(f"Convergiu após {i+1} iterações.")
            break

        # --- Aplicação da Transformação ---
        current_source_cloud.transform(incremental_transform_matrix)                # Aplica a transformação incremental à fonte atual para a próxima iteração
        total_transformation = incremental_transform_matrix @ total_transformation  # Acumula a transformação

        # --- Atualização do Visualizador para Animação ---
        # A nuvem de pontos 'current_source_cloud' já foi transformada.
        vis.update_geometry(current_source_cloud)
        vis.poll_events()
        vis.update_renderer()
        
    vis.destroy_window()        # Fecha o visualizador no final

    return current_source_cloud, total_transformation


def main():

    # =========================================================
    # 1) CARREGAMENTO (OpenCV) + FILTRAGEM DE PROFUNDIDADE
    # =========================================================
    # Imagem 1 (RGB, Depth)
    rgb1_cv  = cv2.cvtColor(cv2.imread("tum_dataset/rgb/1.png", cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth1   = cv2.imread("tum_dataset/depth/1.png", cv2.IMREAD_UNCHANGED)  # Não alterar o número de bits (PNG de 16 bits)
    depth1   = cv2.medianBlur(depth1, 5)                                    # Filtro de mediana para reduzir ruído, compara cada píxel com a vizinhança numa matriz 5x5
    depth1_m = depth1.astype(np.float32) / 5000.0                           # uint16 para float32 e de milímetros para metros (Fator de escala assumido:5000)
    depth1_m[(depth1_m < 0.1) | (depth1_m > 3.0)] = 0.0                     # Remove valores de profundidade inválidos (< 0.1m ou > 3.0m), '|' é o operador 'ou' em numpy

    # Imagem 2 (RGB, Depth)
    rgb2_cv  = cv2.cvtColor(cv2.imread("tum_dataset/rgb/2.png", cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth2   = cv2.imread("tum_dataset/depth/2.png", cv2.IMREAD_UNCHANGED)
    depth2   = cv2.medianBlur(depth2, 5)
    depth2_m = depth2.astype(np.float32) / 5000.0
    depth2_m[(depth2_m < 0.1) | (depth2_m > 3.0)] = 0.0


    # =========================================================
    # 2) CRIAÇÃO DE NUVENS (Open3D a partir de dados OpenCV)
    # =========================================================
    color1_o3d = o3d.geometry.Image(rgb1_cv)
    depth1_o3d = o3d.geometry.Image(depth1_m)
    rgbd1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color1_o3d, depth1_o3d, depth_scale=1.0, depth_trunc=5.0, convert_rgb_to_intensity=False
    )

    color2_o3d = o3d.geometry.Image(rgb2_cv)
    depth2_o3d = o3d.geometry.Image(depth2_m)
    rgbd2 = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color2_o3d, depth2_o3d, depth_scale=1.0, depth_trunc=5.0, convert_rgb_to_intensity=False
    )

    intr = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
    )
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd1, intr)
    pcd2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd2, intr)


    # =========================================================
    # 3) PRÉ-PROCESSAMENTO
    # =========================================================
    pcd1_ds = pcd1.voxel_down_sample(0.01)  # 1 cm
    pcd1_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))  # Superfície local de 30 pontos máx num raio de 5 cm

    pcd2_ds = pcd2.voxel_down_sample(0.01)  # 1 cm
    pcd2_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

    pcd1_ds, _ = pcd1_ds.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    pcd2_ds, _ = pcd2_ds.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)


    # # Visualização ANTES (fonte vermelha, alvo azul)
    # axes = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
    # pcd1_ds.paint_uniform_color([1, 0, 0])  # fonte: vermelho
    # pcd2_ds.paint_uniform_color([0, 0, 1])  # alvo: azul
    # o3d.visualization.draw_geometries(
    #     [pcd1_ds, pcd2_ds, axes],
    #     front=view['trajectory'][0]['front'],
    #     lookat=view['trajectory'][0]['lookat'],
    #     up=view['trajectory'][0]['up'],
    #     zoom=view['trajectory'][0]['zoom'],
    # )


    # =========================================================
    # 4) ICP Personalizado
    # =========================================================
    # Transformação rígida com rotação de -10 graus em y e -5 graus em z, com ligeira translação (x=0.80m, y=-0.05m, z=-0.05m)
    T_init = np.asarray([
        [ 0.9848,  0.0872,  -0.1736,   0.80],
        [ 0.0,     1.0,      0.0,     -0.05],
        [ 0.1736,  0.0,      0.9848,  -0.05],
        [ 0.0,     0.0,      0.0,      1.00]
    ], dtype=float)

    max_correspondence_distance = 0.05

    # Executar o ICP personalizado
    registered_source_cloud, final_transformation = custom_icp(
        pcd1_ds, pcd2_ds, T_init, max_correspondence_distance
    )


    if registered_source_cloud is not None:
        # # Visualização DEPOIS (fonte registada verde, alvo azul)
        # registered_source_cloud.paint_uniform_color([0, 1, 0]) # Fonte registada: verde
        # print("Visualizando nuvens de pontos DEPOIS do registo ICP...")
        # o3d.visualization.draw_geometries(
        #     [registered_source_cloud, pcd2_ds, axes],
        #     front=view['trajectory'][0]['front'],
        #     lookat=view['trajectory'][0]['lookat'],
        #     up=view['trajectory'][0]['up'],
        #     zoom=view['trajectory'][0]['zoom'],
        # )

        print("\nMatriz de transformação final:")
        print(final_transformation)
    else:
        print("O ICP não foi concluído com sucesso.")


if __name__ == "__main__":
    main()