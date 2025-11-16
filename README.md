# savi-2025-2026-trabalho1-grupo5
***
## Tarefa 1

A Tarefa 1 consiste em criar duas nuvens de pontos, uma fonte e uma alvo, cada uma gerada a partir de uma imagem RGB (que contém a informação de cor e coordenadas 2D) e de uma imagem de profundidade (com os valores de distância de cada pixel). 

O objetivo é alinhar estas duas nuvens de pontos recorrendo ao algoritmo **ICP (Iterative Closest Point)**, através das funções nativas disponibilizadas pelo Open3D. 
Neste caso, utilizou-se a variante Point-to-Plane, implementada no próprio Open3D, por ser mais eficiente e gerar resultados mais precisos do que o método Point-to-Point em nuvens de pontos mais complexas.

Enquanto o Point-to-Point procura reduzir diretamente a distância entre os pontos da fonte e do alvo sendo adequado para nuvens simples e com pouco ruído, o Point-to-Plane minimiza a distância entre cada ponto da fonte e o plano tangente correspondente na nuvem alvo, resultando num alinhamento mais suave, estável e realista.

Para tal é necessário:

**1. Carregar e filtrar as duas imagens** (imagem rgb e imagem profundidade) para a criação de cada uma das nuvens de pontos
   - Carregar a imagem rgb lembrando de alterar de bgr para rgb usando `_cv2.COLOR_BGR2RGB_`;
   - Carregar a imagem profundidade mantendo o número de bits (16 bits ao invés dos 8 bits típicos) usando o argumento `_cv2.IMREAD_UNCHANGED_`;
   - Aplicar um filtro de mediana à imagem profundidade para reduzir o ruído (`_cv2.medianBlur_`);
   - Converter a imagem profundidade para o tipo _float32_ e para metros, assumindo um fator de ampliação de 5000;
   - Remover valores de profundidade inválidos (inferiores a 0.1 m e superiores a 3 m).

**2. Criação das nuvens de pontos fonte e alvo usando o Open3D**
   - Para a criação das nuvens de pontos fonte e alvo, é necessário primeiro criar objetos de imagem do Open3D a partir das imagens RGB carregadas com o OpenCV, utilizando a função `_o3d.geometry.Image_`.
   - Em seguida, as imagens RGB e de profundidade são combinadas para formar um único objeto RGBD, que contém simultaneamente a cor de cada pixel e a respetiva distância, através da função `_o3d.geometry.RGBDImage.create_from_color_and_depth_`.
   - Posteriormente, definem-se os parâmetros intrínsecos da câmara segundo o modelo _pinhole_, recorrendo à função `_o3d.camera.PinholeCameraIntrinsic_` e utilizando valores _default_ (pré-definidos).
   - Finalmente, com base nas imagens RGBD e nos parâmetros intrínsecos da câmara, é criada a nuvem de pontos 3D com a função `_o3d.geometry.PointCloud.create_from_rgbd_image_`, em que cada pixel da imagem RGBD passa a representar um ponto no espaço tridimensional, contendo tanto a cor como as coordenadas espaciais (x, y, z) correspondentes.

**3. Pré-processamento** com duas operações principais às nuvens de pontos, com o objetivo de reduzir a sua complexidade e prepará-las para o alinhamento pelo método ICP
   - Foi realizado o _downsampling_ através da função `_voxel_down_sample_`, aplicada às nuvens de pontos criadas anteriormente. Esta operação reduz a densidade das nuvens ao agrupar pontos próximos dentro de um _voxel_ (cubo) de tamanho definido, neste caso de 1 cm. Assim, diminui-se o número total de pontos, o que acelera o processamento e reduz o ruído sem comprometer significativamente o detalhe da estrutura.
   - Em seguida, foram estimadas as normais através da função `_estimate_normals_`, que define uma superfície local e a respetiva normal (vetor perpendicular à superfície) para cada ponto da nuvem. Este cálculo é feito com base nos vizinhos mais próximos, sendo considerados no máximo 30 vizinhos num raio de 5 cm.

**4. Alinhamento das nuvens de pontos – ICP (Iterative Closest Point)**
O objetivo deste método é encontrar a transformação rígida (rotação e translação) que melhor alinha a nuvem de pontos fonte à nuvem de pontos alvo, minimizando a distância entre os pontos correspondentes.

Passos realizados:

  - Definição de parâmetros iniciais : Começamos por definir o _threshold_, que representa o limite máximo de correspondência entre pontos da fonte e do alvo (neste caso, 0.02 m).
  - Em seguida, foi criada a matriz de transformação inicial (_T_init_), onde se estimou uma translação de 80 cm no eixo x e -5 cm nos eixos y e z, bem como uma rotação de -10° em torno de y e -5° em torno de z. Quanto mais próxima esta transformação inicial estiver do valor real, mais rápido e estável será o processo de convergência do ICP.
  - Execução do ICP, função `_registration_icp_`: Dentro de um ciclo _for_, foi utilizada a função `_o3d.pipelines.registration.registration_icp_`, que requer:
    - as nuvens de pontos fonte e alvo (com as normais já estimadas);
    - os parâmetros definidos anteriormente (_threshold_ e _T_init_);
    - o método de correspondência, neste caso _Point-to-Plane_;
    - e o número máximo de iterações internas (500 por ciclo).

O método _Point-to-Plane_ calcula a distância entre cada ponto da fonte e o plano tangente do ponto correspondente na nuvem alvo e verifica se essa distância é inferior ao valor limite (_threshold_).
Este processo é repetido iterativamente para refinar o alinhamento.

A função devolve três valores principais:
   - Fitness → representa a percentagem de pontos da fonte que encontraram correspondência válida no alvo (quanto maior, melhor);
   - RMSE (Root Mean Square Error) → indica o erro médio das distâncias entre os pontos correspondentes (quanto menor, melhor);
   - Matriz de transformação → a melhor transformação (R,t) obtida até ao momento.

Ciclo de 20 iterações com refinamento sucessivo: Após cada iteração, se o valor de RMSE for inferior ao obtido anteriormente, o resultado é guardado como o melhor alinhamento.  
A matriz de transformação inicial (_T_init_) é então atualizada com a nova transformação encontrada, permitindo iniciar o próximo ciclo com uma estimativa mais precisa. 
Este processo é repetido 20 vezes, com o objetivo de refinar progressivamente a solução, melhorando a estabilidade e a precisão da convergência do ICP.

Visualização e validação do resultado: Por fim, foi aplicada a melhor matriz de transformação à nuvem de pontos fonte, de modo a gerar a imagem 3D transformada. 
Posteriormente, ambas as nuvens (fonte e alvo) foram visualizadas sobrepostas, permitindo verificar visualmente se a fonte convergiu corretamente para o alvo após o alinhamento.

***
Durante o desenvolvimento da tarefa, um dos principais desafios foi a definição adequada dos parâmetros do algoritmo. 
Valores como o número de iterações, o _threshold_, o tamanho do _voxel_ utilizado no _downsampling_ e os parâmetros dos filtros de ruído tiveram de ser ajustados por tentativa e erro, de modo a obter um equilíbrio entre precisão e tempo de processamento.

Outro desafio relevante foi a determinação de uma boa matriz de transformação inicial.

Como o método _Point-to-Plane_ minimiza a distância entre cada ponto da nuvem fonte e o plano tangente correspondente na nuvem alvo, uma estimativa inicial incorreta pode conduzir a resultados insatisfatórios, por exemplo, o algoritmo pode alinhar parcialmente as superfícies mas com desvio lateral, produzindo uma correspondência visualmente incorreta. 

Para ultrapassar esse problema, a matriz de transformação inicial foi calculada com recurso a inteligência artificial (IA), tendo por base a estimativa manual previamente obtida. Esta abordagem revelou-se eficaz, permitindo iniciar o ICP com uma transformação inicial mais precisa e, consequentemente, alcançar uma convergência mais estável e realista.

Seguem abaixo algumas imagens do resultado obtido:

(Antes e Depois)

<p float="left">

 <img src="https://github.com/user-attachments/assets/6e3baf8d-9cad-4a57-95ca-a9d7550b512c" width="400" />
 <img src="https://github.com/user-attachments/assets/9ab1e7f3-ff25-4421-a25f-8cbcecba439e" width="400" /> 

</p>

***
## Tarefa 2
O objetivo da Tarefa 2 é idêntico ao da Tarefa 1: alinhar duas nuvens de pontos (fonte e alvo) geradas a partir de imagens RGB e de profundidade. 

No entanto, nesta etapa não se utilizam as funções nativas do Open3D para o cálculo do ICP. Em vez disso, é desenvolvido um **ICP personalizado**, implementado manualmente, com o intuito de reproduzir (ou até melhorar) os resultados obtidos anteriormente. 

Além disso, é realizada a otimização da raiz do erro em cada iteração, recorrendo ao método dos Mínimos Quadrados (_Least Squares_), de forma a resolver o problema de minimização e a encontrar, de modo mais controlado, a melhor transformação rígida (rotação e translação) entre as duas nuvens de pontos.

Para atingir o resultado pretendido, é necessário seguir o seguinte conjunto de etapas:

**1. A primeira parte do processo, até ao início da implementação do ICP, é idêntica à da Tarefa 1.**
Ou seja, envolve o carregamento das imagens (com o OpenCV), a filtragem da imagem de profundidade, a criação das nuvens de pontos e respetivo pré-processamento. 
Estas etapas iniciais garantem que os dados de entrada estão corretamente preparados para a fase seguinte, onde é desenvolvido o ICP personalizado.

**2. De seguida, é iniciado o ICP personalizado.**
Tal como na Tarefa 1, é necessário definir uma matriz de transformação inicial e um valor de _threshold_ (limite máximo de correspondência entre pontos).

**3. Após esta preparação, é chamada a função `_custom_icp_`**, que é necessário ser criada e à qual são passados os seguintes argumentos:
   - os dados das nuvens de pontos da fonte e do alvo;
   - a matriz de transformação inicial;
   - e o valor do _threshold_.

Esta função é a principal responsável pela execução do ICP personalizado, realizando o processo iterativo de alinhamento entre as duas nuvens. 

**4. Além desta função principal, foram também criadas duas funções auxiliares, `_get_transformation_matrix_` e `_residuals_func_`**, que são chamadas internamente dentro da função `_custom_icp_` e são essenciais para o correto funcionamento do algoritmo.

  - A função `_residuals_func_` é essencial no ICP personalizado, pois define a função de erro utilizada pelo método de Mínimos Quadrados (_Least Squares_).

    Recebe como parâmetros:
    - o vetor x, que contém os seis valores da transformação incremental (rx, ry, rz, tx, ty, tz);
    - os pontos correspondentes da nuvem fonte e da nuvem alvo;
    - e as respetivas normais do alvo.
    
    O seu objetivo é calcular e devolver um vetor de resíduos, em que cada elemento representa a distância ponto-para-plano entre um ponto da fonte transformado e o plano definido pelo ponto e pela normal correspondentes na nuvem alvo.

    Para isso, converte-se o vetor x numa matriz de transformação 4×4, aplica-se essa transformação aos pontos da fonte e calcula-se o erro de cada correspondência segundo a expressão: r_i = ((Rs_i + t)-t_i)*n_i

    Estes resíduos são depois minimizados pela função least_squares, ajustando iterativamente a transformação até alinhar as duas nuvens de pontos.

  - A função `_get_transformation_matrix_` recebe um vetor _params_ com seis parâmetros incrementais, (rx, ry, rz, tx, ty, tz), e constrói a matriz homogénea 4×4 correspondente.
    - Os três primeiros valores definem a rotação incremental no formato eixo-ângulo (axis-angle), a partir da qual é obtida a matriz R usando `_o3d.geometry.get_rotation_matrix_from_axis_angle_`.
    - Os três últimos valores definem a translação t.
    - A função coloca R no bloco superior esquerdo da identidade 4×4 e t na última coluna, devolvendo a matriz transformação incremental.

      Esta transformação incremental é aplicada em cada iteração do ICP para atualizar a posição estimada de forma estável e compatível com pequenas rotações.

**5. Regressando à função principal _custom_icp_, dentro desta são implementados todos os passos do ICP personalizado.**
   - Transformação inicial: Começa-se por aplicar a matriz de transformação inicial à nuvem de pontos fonte, garantindo que esta já parte de uma estimativa aproximada da posição correta;
   - Criação da estrutura _KD-Tree_: Em seguida, é criada uma instância de `_o3d.geometry.KDTreeFlann()_` a partir da nuvem de pontos alvo. Esta estrutura constrói um índice espacial (_KD-Tree_) que permite realizar pesquisas rápidas de vizinhos mais próximos, o que será fundamental nas próximas etapas;
   - Ciclo de iterações: Inicia-se então um ciclo for de várias iterações, responsável por refinar progressivamente a transformação até atingir a convergência.
     - Correspondência de pontos: Para cada ponto da nuvem de pontos fonte (já transformada na iteração anterior), utiliza-se a função `_target_kdtree.search_knn_vector_3d(source_points[j], 1)_`.
       Esta função procura, no _KD-Tree_ do alvo, o vizinho mais próximo do ponto atual da fonte e devolve três valores: o número de vizinhos encontrados (_k_), o índice desses vizinhos na nuvem alvo (_idx_) e a distância ao quadrado (_dist_sq_).
     - Seleção de correspondências válidas: Caso a distância ao quadrado seja inferior ao _threshold²_, guarda-se essa correspondência em três _arrays_: os pontos da fonte correspondentes, os pontos do alvo e as normais do alvo. Estes três conjuntos de dados serão usados na etapa seguinte.
     - Em seguida, é chamada a função `_least_squares_, responsável por otimizar a transformação a aplicar à nuvem fonte. Este é o passo onde o código “ajusta” a pequena rotação e translação que melhor reduz os erros _point-to-plane_ nesta iteração.
       Em termos simples, a função procura os seis parâmetros [rx, ry, rz, tx, ty, tz] que fazem com que os pontos da fonte transformada fiquem o mais próximos possível dos pontos do alvo, na direção das normais.

       Para isso, a função `_least_squares_` recebe:
       - A função `_residuals_func_` (já explicada anteriormente), que calcula os resíduos individuais para todas as correspondências;
       - Um vetor inicial de parâmetros [rx, ry, rz, tx, ty, tz], inicializado a zero na primeira iteração;
       - As três _arrays_ de pontos e normais obtidas no passo anterior;
       - O método de otimização _Levenberg–Marquardt_, adequado para mínimos quadrados não lineares sem restrições e com mais equações do que incógnitas;
       - O parâmetro _verbose=0_, que desativa a impressão detalhada durante a otimização.
         
     - Verificação de convergência: Após a otimização, os parâmetros [rx, ry, rz, tx, ty, tz] são convertidos numa matriz de transformação 4×4 através da função `_get_transformation_matrix_` (já explicada).
     - Em seguida, verifica-se se a norma do vetor de parâmetros incrementais é inferior à tolerância definida (1e-6 no nosso caso). Se essa condição for satisfeita, o algoritmo considera que convergiu.
     - Atualização da transformação total: No final de cada iteração, a transformação incremental obtida é aplicada à nuvem de pontos fonte (já transformada pela estimativa anterior) e é acumulada na matriz de transformação global. Assim, a matriz inicial é atualizada a cada ciclo, tornando-se progressivamente mais próxima da transformação ideal. Desta forma, o ICP personalizado vai refinando a posição da fonte em direção à posição do alvo, até obter o alinhamento final ótimo.
***
Imagens das nuvens de pontos antes da otimização e depois, respetivamente. Assim como a matriz de transformação final.

<p float="left">

 <img src="https://github.com/user-attachments/assets/6e3baf8d-9cad-4a57-95ca-a9d7550b512c" width="400" />
 <img src="https://github.com/user-attachments/assets/2aa0b971-f1b8-4c5c-b3ad-f4a4aa87d44c" width="400" /> 
 <img src="https://github.com/user-attachments/assets/018a0163-bdad-4c3c-9696-5b3273ab0533" width="400" />
 <img src="https://github.com/user-attachments/assets/675fe1b9-09ab-4a40-b209-d05ebb108d7e" width="400" /> 

</p>


***
## Tarefa 3
O objetivo da Tarefa 3 é adicionar uma etapa extra de otimização ao código desenvolvido na Tarefa 2, permitindo avaliar a qualidade da convergência entre as duas nuvens de pontos. 

Nesta fase, introduz-se a criação de uma **esfera englobante mínima**, cujo propósito é encontrar a menor esfera possível que contenha todos os pontos das nuvens (fonte e alvo). 

A esfera é calculada para o conjunto das duas nuvens após a transformação, e também individualmente para a nuvem alvo. A ideia é comparar os raios obtidos quanto mais próximos forem os valores, melhor foi o alinhamento da nuvem de pontos fonte em relação ao alvo, indicando uma convergência mais precisa e estável no processo de ICP personalizado.

Para isso é então necessário realizar o seguinte:

**1. Primeiro, é preciso implementar um código idêntico ao da Tarefa 2**, ou seja, carregar as imagens, criar as nuvens de pontos, realizar o pré-processamento e executar o ICP personalizado para obter a nuvem fonte alinhada com a nuvem alvo.

**2. De seguida, para criar a esfera englobante**, são necessárias três novas funções:
   - `_find_minimum_enclosing_sphere()_`, que será responsável por encontrar os parâmetros (centro e raio) da esfera englobante mínima para um dado conjunto de pontos;
   - `_constraint_func()_`, que define a restrição garantindo que todos os pontos ficam dentro ou sobre a superfície da esfera;
   - `_objective_func()_`, que é a função objetivo utilizada pelo otimizador, devolvendo simplesmente o valor do raio — o parâmetro que pretendemos minimizar.

  Estas duas últimas funções são chamadas dentro de `_find_minimum_enclosing_sphere()_` e são fundamentais para que a otimização funcione corretamente.

  Na função `_objective_func(params)_, é fornecido o vetor _params_, composto pelos quatro parâmetros [xc, yc, zc, r], onde os três primeiros representam o centro da esfera e o último corresponde ao raio. 
  O objetivo desta função é apenas devolver o valor do raio, já que é este que será minimizado pelo processo de otimização.
  
  A função `_constraint_func(params, points)_ recebe os mesmos parâmetros descritos anteriormente, bem como o conjunto de pontos pertencentes à nuvem de pontos. 
  Esta função calcula, para cada ponto, a distância euclidiana ao centro da esfera, através da norma ||p_i - c||, sendo 'p_i' o vetor de coordenadas de cada ponto e 'c' o vetor das coordenadas do centro. 
  No final, devolve a diferença entre o raio da esfera e a distância ao centro de cada ponto. 
  Esta diferença deve ser maior ou igual a zero, garantindo que todos os pontos se encontram dentro ou na superfície da esfera.

**3. Agora passando à função principal `_find_minimum_enclosing_sphere(points)_**, à qual devemos fornecer a nuvem de pontos. Esta função é responsável por encontrar os parâmetros ideais da esfera englobante mínima e é implementada da seguinte forma:
   - Primeiro, é feita uma estimativa inicial dos parâmetros da esfera.
     - Calculamos o centróide da nuvem de pontos através da média das coordenadas de todos os pontos ([média_x, média_y, média_z]).
     - Em seguida, calculamos a distância entre cada ponto e o centróide e obtemos o valor máximo, esse será o raio inicial (utilizando `_np.max(np.linalg.norm(...))_`).
     Estes valores servirão como parâmetros iniciais da otimização.

   - De seguida, definimos limites (_bounds_) para os parâmetros a otimizar.
     O centro da esfera não possui restrições, mas o raio deve ser sempre positivo, por isso definimos o limite inferior como 1e-6 em vez de 0.
     Isto evita possíveis erros numéricos e garante que o raio nunca é nulo.
     
   - Depois, definimos as restrições através da variável: `_constraints = [{'type': 'ineq', 'fun': constraint_func, 'args': (points,)}]_`.
     - Aqui, utilizamos a função `_constraint_func(params, points)_` (já explicada anteriormente), que calcula a diferença para cada ponto da nuvem.
     - O parâmetro _'type': 'ineq'_ indica que essa diferença deve ser maior ou igual a zero, garantindo que todos os pontos ficam dentro ou na superfície da esfera.
   
   - Segue-se a etapa de otimização, onde minimizamos o tamanho da esfera.
     Utilizamos a função `_minimize_` (em vez de `_least_squares_`, pois esta é mais adequada para múltiplas restrições).

     Passamos como argumentos:
     - a função `_objective_func_`;
     - os parâmetros iniciais;
     - o método SLSQP (Sequential Least Squares Programming), apropriado para problemas com restrições;
     - os limites;
     - as restrições;
     - e o número máximo de iterações.
     
     Esta função ajusta iterativamente os parâmetros da esfera [xc, yc, zc, r], de modo a minimizar o raio e satisfazer as restrições (ou seja, garantir que todos os pontos estão contidos na esfera).

     O processo termina quando converge ou quando é atingido o número máximo de iterações.

     O resultado final é aceite apenas se for viável, ou seja, se obedecer às restrições impostas; caso contrário, o resultado retorna _success = False_.

     Com esta função, garantimos uma esfera otimizada que engloba todos os pontos da nuvem fornecida.
     
   - Assim, concluímos a função `_find_minimum_enclosing_sphere(points)_`, obtendo como resultado os parâmetros otimizados (centro e raio) da esfera que engloba a totalidade dos pontos.

**4.** Com as três funções criadas (`_objective_func_`, `_constraint_func_` e `_find_minimum_enclosing_sphere_`), já é possível determinar os parâmetros de uma esfera otimizada. 
   
   No nosso caso, pretende-se gerar duas esferas:
   - Uma para a nuvem de pontos alvo;
   - Outra para o conjunto formado pelas duas nuvens (alvo e fonte) após a transformação. Para isso, foi utilizada a função `_np.vstack()_` para juntar as duas nuvens de pontos num único `_array_`.

**5.** Por fim, com os parâmetros obtidos (centro e raio) para as esferas das duas nuvens, criamos a visualização final. 
   
   Esta inclui as nuvens de pontos e as respetivas esferas, permitindo observar visualmente o quão próximas estão, quanto mais semelhantes forem os raios, melhor foi a convergência obtida no processo de ICP.
***
A primeira imagem apresenta a esfera englobante da nuvem de pontos alvo. As duas imagens seguintes são da esfera englobante do conjunto das nuvens de pontos após o seu alinhamento. A útima imagem apresenta os resultados, incluindo as coordenadas dos respetivos centros e os rais das esferas.

<p float="left">

 <img src="https://github.com/user-attachments/assets/c936b1c7-5236-4ca6-97c2-43581bcb3302" width="400" />
 <img src="https://github.com/user-attachments/assets/732b3f31-c56a-40d7-bd99-1c6ac220a87d" width="400" /> 
 <img src="https://github.com/user-attachments/assets/4d32536f-c681-4a40-b273-4f1933781194" width="400" />
 <img src="https://github.com/user-attachments/assets/706a0f8f-db2b-47f8-8177-3ae09213f979" width="400" /> 

</p>

***
## Conclusão
Para concluir, observando a visualização final da Tarefa 2, verificamos que foi alcançada uma boa convergência da nuvem de pontos fonte para a nuvem alvo, uma vez que é possível observar uma sobreposição bastante coerente entre ambas. Contudo, ao analisarmos os resultados através do método da esfera englobante mínima, obtivemos um raio de aproximadamente 1.72 para o conjunto das duas nuvens (alvo + fonte) inicial e cerca de 1.90 para o conjunto após otimização, o que corresponde a um aumento de cerca de 10%. Isto indica que, apesar da boa sobreposição visual, o método da esfera, utilizado isoladamente, não é adequado como métrica principal de convergência. Por exemplo, se criássemos uma nova nuvem fonte que fosse apenas uma rotação da nuvem alvo em torno de um eixo central, sem qualquer alinhamento entre as duas, o método da esfera provavelmente devolveria um raio muito semelhante para ambas, mesmo que, visualmente, as nuvens estivessem completamente desalinhadas. Ou seja, este método só é útil depois de realizado o ICP, funcionando como uma avaliação complementar para verificar se a convergência obtida é suficientemente boa ou se ainda existe margem para melhoria.

***
Finalmente, é de notar que, em todas as três tarefas, poderiam ter sido obtidos resultados ainda melhores com alguns ajustes, como:
- aumentar o número de iterações do ICP;
- reduzir o valor do voxel no pré-processamento (voxel_down_sample(0.01)) para preservar uma maior densidade de pontos;
- afinar parâmetros como o _threshold_, raio de procura das normais, limites do otimizador, entre outros.
Ainda assim, os resultados obtidos demonstram um bom desempenho geral, tanto no alinhamento como na análise complementar da convergência.
