# savi-2025-2026-trabalho1-grupoX

## Tarefa 1
- Carregamento das duas imagens (imagem rgb e imagem profundidade) para a criação de cada uma das nuvens de pontos;
- Filtragem da imagem profundidade e passagem para metros;
- Criação das nuvens de pontos fonte e alvo usando o Open3D;
- Pré-processamento realizando um down_sample e uma estimativa de normais definindo uma superfície local de 30 pontos no máximo num raio de 5 cm;
- Através da visualização das nuvens de pontos sobrepostas antes de qualquer transformação, estimou-se uma translação de 80 cm no eixo x e -5 cm nos eixos y e z, assim como uma rotação de -10º em torno de y e -5º em torno de z;
- Aplicação do ICP com o parâmetro Point to Plane usando como transformação inicial a matriz resultante das transformações estimadas anteriormente;
- Realização de 20 iterações, com um threshold = 0.02 e um número máximo de 500 iterações para convergir;
- Visualização do melhor resultado.

Os parâmetros como número de iterações, threshold, valores a atribuir no down_sample e nos filtros usados foram estimados por tentativa e erro. A matriz de transformação inicial foi calculada por IA tendo por base a estimativa feita, como solução alternativa ao uso do cloud compare.

Seguem abaixo algumas imagens do resultado obtido:

<p float="left">

 <img src="https://github.com/user-attachments/assets/94c1618c-a598-4b2a-b352-b8fa25dfc2d6" width="400" />
 <img src="https://github.com/user-attachments/assets/70e0b708-aba7-4ee9-9339-9a8175d4e054" width="400" /> 
 <img src="https://github.com/user-attachments/assets/0e3490ee-9695-4137-9bda-f5c5550a079d" width="400" />
 <img src="https://github.com/user-attachments/assets/0d0eddb9-a63c-49f1-b680-205ae1228837" width="400" />

</p>

