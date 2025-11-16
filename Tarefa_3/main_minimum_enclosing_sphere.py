import cv2
import numpy as np
import open3d as o3d
from copy import deepcopy
from scipy.optimize import minimize
from scipy.optimize import least_squares


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


# ========================================================
# Funções para a Otimização da Esfera Englobante Mínima
# ========================================================

# Função objetivo: minimizar o raio da esfera (retorna o valor do raio)
def objective_func(params):
    """
    Recebe um vetor de 4 parâmetros: [xc, yc, zc, r]
    onde os primeiros 3 são o centro (xc, yc, zc) da esfera,
    e o último é o raio (r) da esfera.
    """
    return params[3] 

# Função de restrição: garantir que todos os pontos estão dentro ou na superfície da esfera
def constraint_func(params, points):
    """
    Recebe o vetor x já descrito assim como os pontos da fonte e do alvo.
    Aplica a transformação incremental aos pontos da fonte,
    calcula as distâncias (dos pontos transformados da fonte e dos pontos alvo)
    ao centro da esfera e retorna r - distância para cada ponto.
    A restrição é formulada como: r - sqrt((x - xc)^2 + (y - yc)^2 + (z - zc)^2) >= 0
    """
    xc, yc, zc, r = params
    
    # Calcula a distância euclidiana de cada ponto ao centro da esfera
    distances_sq = (points[:, 0] - xc)**2 + \
                   (points[:, 1] - yc)**2 + \
                   (points[:, 2] - zc)**2
    
    # Retorna r - distância para cada ponto. Queremos que isto seja >= 0.
    return r - np.sqrt(distances_sq)


def find_minimum_enclosing_sphere(points):
    """
    Encontra os parâmetros (centro e raio) da esfera englobante mínima para um dado conjunto de pontos.
    Recebe um array do todos os pontos a englobar.
    Retorna o centro e o raio da esfera.
    """
    if len(points) == 0:
        print("Nenhum ponto fornecido para encontrar a esfera englobante.")
        return np.array([0,0,0]), 0.0

    # Estimativa inicial para os parâmetros da esfera:
    # Centro: Centróide dos pontos
    # Raio: Distância máxima do centróide a um ponto
    centroid = np.mean(points, axis=0)
    max_dist_from_centroid = np.max(np.linalg.norm(points - centroid, axis=1))
    
    # Parâmetros iniciais [xc, yc, zc, r]
    initial_params = np.array([centroid[0], centroid[1], centroid[2], max_dist_from_centroid])

    # Definir os limites (bounds) para os parâmetros:
    bounds = (
        (None, None),  # xc
        (None, None),  # yc
        (None, None),  # zc
        (1e-6, None)   # r (raio deve ser positivo)
    )

    # Definir as restrições
    # 'type': 'ineq' para desigualdade (>= 0)
    # 'fun': a função que define a restrição
    # 'args': argumentos para a função de restrição
    constraints = [
        {'type': 'ineq', 'fun': constraint_func, 'args': (points,)}
    ]

    print(f"Iniciando otimização da esfera englobante mínima com {len(points)} pontos...")
    
    # Realizar a otimização
    # Usar o minimize ao invés de least_squares, pois funciona melhor com múltiplas restrições
    res = minimize(
        objective_func,                         # Função objetivo
        initial_params,                         # Parâmetros iniciais
        method='SLSQP',                         # Sequential Least Squares Programming - bom para problemas com restrições
        bounds=bounds,                          # Limites para os parâmetros
        constraints=constraints,                # Restrições
        options={'disp': True, 'maxiter': 500}  # Não imprimir cada iteração; máximo de 500 iterações
    )
    
    if res.success:
        optimized_params = res.x
        center = optimized_params[0:3]
        radius = optimized_params[3]
        print(f"Otimização da esfera englobante concluída com sucesso.")
        print(f"Centro: {center}, Raio: {radius}")
        return center, radius
    else:
        print(f"Otimização da esfera englobante falhou: {res.message}")
        # Retorna a estimativa inicial ou uma esfera vazia em caso de falha
        return np.array([0,0,0]), 0.0
    


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
def custom_icp(source_cloud_for_viz, target_cloud, initial_transform, max_correspondence_distance, max_iterations=100, tolerance=1e-6):
    
    
    # Faz uma cópia da nuvem fonte para aplicar as transformações
    current_source_cloud = source_cloud_for_viz
    current_source_cloud.transform(initial_transform) # Aplica a transformação inicial

    # Armazena a transformação total acumulada
    total_transformation = initial_transform

    # Constrói a KDTree para a nuvem alvo (encontra os vizinhos mais próximos de cada ponto, necessário para correspondências)
    # Para o alvo não é necessário atualizar a KDTree, pois o alvo permanece fixo
    target_kdtree = o3d.geometry.KDTreeFlann(target_cloud)

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


    return current_source_cloud, total_transformation


def main():

    # =========================================================
    # 1) CARREGAMENTO (OpenCV) + FILTRAGEM DE PROFUNDIDADE
    # =========================================================
    # Imagem 1 (RGB, Depth)
    rgb1_cv  = cv2.cvtColor(cv2.imread("rgb/1.png", cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth1   = cv2.imread("depth/1 .png", cv2.IMREAD_UNCHANGED)  # Não alterar o número de bits (PNG de 16 bits)
    depth1   = cv2.medianBlur(depth1, 5)                                    # Filtro de mediana para reduzir ruído, compara cada píxel com a vizinhança numa matriz 5x5
    depth1_m = depth1.astype(np.float32) / 5000.0                           # uint16 para float32 e de milímetros para metros (Fator de escala assumido:5000)
    depth1_m[(depth1_m < 0.1) | (depth1_m > 3.0)] = 0.0                     # Remove valores de profundidade inválidos (< 0.1m ou > 3.0m), '|' é o operador 'ou' em numpy

    # Imagem 2 (RGB, Depth)
    rgb2_cv  = cv2.cvtColor(cv2.imread("rgb/2.png", cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth2   = cv2.imread("depth/2.png", cv2.IMREAD_UNCHANGED)
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

    # =========================================================
    # 4) VISUALIZAÇÃO ANTES DO ICP 
    # =========================================================
    
    axes = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5)
    pcd1_ds_initial = deepcopy(pcd1_ds) # Uma cópia para manter o estado original
    pcd1_ds_initial.paint_uniform_color([1, 0, 0])  # fonte: vermelho
    pcd2_ds.paint_uniform_color([0, 0, 1])  # alvo: azul


    geometries_to_draw_i = [pcd1_ds_initial, pcd2_ds, axes]

    
    print("Visualização inicial: Nuvem fonte (vermelho) e nuvem alvo (azul) ANTES do ICP.")
    o3d.visualization.draw_geometries(
        geometries_to_draw_i,
        window_name="ANTES do ICP ",
        front=view['trajectory'][0]['front'],
        lookat=view['trajectory'][0]['lookat'],
        up=view['trajectory'][0]['up'],
        zoom=view['trajectory'][0]['zoom'],
    )

    
    # =========================================================
    # 5) ICP Personalizado
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
        print("\nMatriz de transformação final:")
        print(final_transformation)
    
        # =========================================================
        # 6) OTIMIZAÇÃO DA ESFERA ENGLOBANTE MÍNIMA
        # =========================================================
        print("\nIniciando otimização da esfera englobante mínima...")

        # Juntar todos os pontos das DUAS nuvens de pontos ALINHADAS
        # Agora usamos 'registered_source_cloud' (resultado do ICP) e 'pcd2_ds' (a nuvem alvo)
        all_points_aligned = np.vstack((np.asarray(registered_source_cloud.points), np.asarray(pcd2_ds.points)))

        # Encontrar a esfera englobante mínima para todos os pontos alinhados 
        center, radius = find_minimum_enclosing_sphere(all_points_aligned)

        # Encontrar a esfera englobante mínima para todos os pontos do target 
        center_target, radius_target = find_minimum_enclosing_sphere(np.asarray(pcd2_ds.points))

        # =========================================================
        # 7) VISUALIZAÇÃO FINAL
        # =========================================================

        registered_source_cloud.paint_uniform_color([0, 1, 0]) # Fonte registada: verde
        # A nuvem alvo já está azul

        geometries_to_draw = [registered_source_cloud, pcd2_ds, axes]

        geometries_to_draw_target = [pcd2_ds, axes]  # alvo já está azul

        # Criar e adicionar a esfera, se o raio for válido
        #Esfera pontos alinhados
        if radius > 1e-6: # Considerar um raio muito pequeno como inválido
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            sphere.translate(center)
            sphere.paint_uniform_color([0.8, 0.8, 0.2]) # Cor amarela para a esfera
            sphere.compute_vertex_normals() 
            line_set_sphere = o3d.geometry.LineSet.create_from_triangle_mesh(sphere)
            line_set_sphere.paint_uniform_color([0.8, 0.8, 0.2]) # Cor da linha
            geometries_to_draw.append(line_set_sphere)

            # Adicionar um ponto para o centro da esfera
            center_sphere_pcd = o3d.geometry.PointCloud()
            center_sphere_pcd.points = o3d.utility.Vector3dVector([center])
            center_sphere_pcd.colors = o3d.utility.Vector3dVector([[1, 0.5, 0]]) # Laranja
            geometries_to_draw.append(center_sphere_pcd)
        else:
            print("Raio da esfera englobante inválido ou muito pequeno. Esfera não será desenhada.")


        #Esfera pontos alvo 
        if radius_target > 1e-6:  # mesmo critério
            sphere_t = o3d.geometry.TriangleMesh.create_sphere(radius=radius_target)
            sphere_t.translate(center_target)
            sphere_t.paint_uniform_color([0.2, 0.8, 0.8])  # cor (ciano) para distinguir, muda se quiseres
            sphere_t.compute_vertex_normals()

            line_set_t = o3d.geometry.LineSet.create_from_triangle_mesh(sphere_t)
            line_set_t.paint_uniform_color([0.2, 0.8, 0.8])
            geometries_to_draw_target.append(line_set_t)

            center_t_pcd = o3d.geometry.PointCloud()
            center_t_pcd.points = o3d.utility.Vector3dVector([center_target])
            center_t_pcd.colors = o3d.utility.Vector3dVector([[1, 0.5, 0]])  # laranja p/ centro
            geometries_to_draw_target.append(center_t_pcd)
        else:
            print("Raio da esfera do ALVO inválido ou muito pequeno. Esfera (alvo) não será desenhada.")


        #Visualização pontos alinhados 
        o3d.visualization.draw_geometries(
            geometries_to_draw,
            window_name = "ALINHADO",
            front=view['trajectory'][0]['front'],
            lookat=view['trajectory'][0]['lookat'],
            up=view['trajectory'][0]['up'],
            zoom=view['trajectory'][0]['zoom'],
        )

        #Visualização pontos alvo
        o3d.visualization.draw_geometries(
            geometries_to_draw_target,
            window_name = "ALVO",
            front=view['trajectory'][0]['front'],
            lookat=view['trajectory'][0]['lookat'],
            up=view['trajectory'][0]['up'],
            zoom=view['trajectory'][0]['zoom'],
        )

        # =========================================================
        # ANÁLISE DOS RESULTADOS
        # =========================================================
        print("\nAnálise dos Resultados:")
        print(f"Centro da esfera englobante mínima para pontos alinhados: {center}")
        print(f"Raio da esfera englobante mínima para pontos alinhados: {radius}")
        print(f"Centro da esfera englobante mínima para pontos alvo: {center_target}")
        print(f"Raio da esfera englobante mínima para pontos alvo: {radius_target}")
        
    else:
        print("O ICP não foi concluído com sucesso.")



if __name__ == "__main__":
    main()