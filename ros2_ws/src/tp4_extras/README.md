# tp4_extras

Itens 3 e 4 do TP4: percepcao veicular no simulador MetaDrive e uma CNN (TensorFlow/Keras) que
classifica o estado da via. E um pacote a parte do Robo Sniper (`projeto_bloco*`): nao usa topicos
nem nos do ROS 2, so compartilha o workspace e o `colcon build`.

| Executavel | Modulo | Faz |
|---|---|---|
| `detectar_faixas` | `deteccao_faixas.py` | faixas na camera sintetica: (a) HSV + mascara de cor (branco e amarelo), (b) Canny, (c) Hough (`cv2.HoughLinesP`); agrupa os segmentos em linhas, acha as duas linhas da faixa do veiculo e sobrepoe tudo ao frame |
| `detectar_formas` | `deteccao_formas.py` | formas geometricas em placas de transito: regioes pela cor, Hough Circle (`cv2.HoughCircles`) para a placa circular e Harris Corner (`cv2.cornerHarris`) para contar os cantos (3 = triangular, 4 = losango, 5 a 9 = octogonal) |
| `coletar_dataset` | `coletar_dataset.py` | 500 frames rotulados pelo estado do simulador em 3 classes (`via_livre`, `obstaculo`, `cruzamento`), balanceados, em treino/validacao/teste com sementes separadas |
| `treinar_cnn` | `treinar_cnn.py` | CNN com BatchNormalization e Dropout(0,5), treinada sem e com data augmentation (rotacao +-15 graus, zoom 0,2, flip horizontal); curvas de accuracy e loss e resumo da generalizacao; GPU se houver, senao CPU |

Apoio: `simulador.py` (MetaDrive com camera RGB sem janela, piloto IDM, projecao de pontos na
imagem, distancia ao obstaculo e ao cruzamento), `placas.py` (placas circular, octogonal,
triangular e losango como poligonos texturizados do panda3d na beira da pista, mais o gabarito de
onde cada uma aparece na imagem), `desenho.py` (rotulos e mosaicos) e `ambiente.py` (pasta do
workspace, pasta de saida, venv e bibliotecas CUDA).

## Dependencias

MetaDrive, TensorFlow, OpenCV e matplotlib ficam num venv proprio, `ros2_ws/.venv_tp4`
(`requirements_tp4_extras.txt`, versoes fixas), separado do `ros2_ws/.venv` do YOLO: os dois pedem
versoes diferentes do numpy. O `scripts/setup.sh` cria o venv; com placa NVIDIA ele instala tambem
o extra `and-cuda` do TensorFlow (CUDA 12 e cuDNN 9 pelo pip, ~4 GB). Sem placa o treino roda na
CPU. Na primeira execucao o MetaDrive baixa os assets (~170 MB).

O MetaDrive desenha a camera por OpenGL e precisa de um servidor X: no WSL2 e o WSLg; numa
maquina sem tela, `xvfb-run -a`.

## Rodar

Tudo de uma vez, na raiz do repositorio (cria o venv se faltar; depois disso, cerca de 9 min com
GPU e 11 min so com CPU, quase tudo na coleta do dataset):

```bash
./scripts/reproduzir_tp4_extras.sh                 # faixas, formas, dataset e cnn
./scripts/reproduzir_tp4_extras.sh faixas formas   # so algumas etapas
DISPOSITIVO=cpu ./scripts/reproduzir_tp4_extras.sh cnn
```

Um executavel por vez, depois do `colcon build` (o pacote acha o venv sozinho e o poe na frente
do venv do YOLO, entao funciona tambem num terminal com o `source ros2_ws/ambiente.sh`):

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 run tp4_extras detectar_faixas                  # --video grava faixas.mp4; --mostrar abre janela
ros2 run tp4_extras detectar_formas
ros2 run tp4_extras coletar_dataset
ros2 run tp4_extras treinar_cnn                      # --dispositivo auto|gpu|cpu  --epocas 60
```

Sem o ROS: `PYTHONPATH=ros2_ws/src/tp4_extras ros2_ws/.venv_tp4/bin/python -m tp4_extras.deteccao_faixas`.
Cada executavel aceita `--help`.

## Saidas (`ros2_ws/log/tp4_extras/`, ou `--saida`)

| Etapa | Arquivos |
|---|---|
| faixas | `faixas/faixas_etapas.jpg` (camera, mascara HSV, Canny, Hough sobreposto), `faixas_NN.jpg`, `faixas_resumo.json` (taxa de frames com as duas linhas da faixa do veiculo, por tipo de bloco, e erro em px contra a geometria da pista no simulador) |
| formas | `formas/formas_etapas.jpg` (camera, Hough Circle e Harris no frame inteiro, formas classificadas), `formas_NN.jpg`, `formas_resumo.json` (acertos por forma e por tamanho aparente, falsos positivos) |
| dataset | `dataset/<particao>/<classe>/*.jpg` (320x180), `dataset/manifesto.csv`, `dataset_amostras.jpg`, `dataset_resumo.json` |
| cnn | `cnn/cnn_curvas.png`, `cnn_resumo.txt`, `cnn_resumo.json`, `cnn_historico.csv`, `cnn_teste_perturbado.jpg`, `cnn_sem_augmentation.keras`, `cnn_com_augmentation.keras` |

## Como os rotulos do dataset sao decididos

O rotulo vem do estado do simulador, nao da imagem (limiares em `coletar_dataset.LIMIARES`):

- `obstaculo`: veiculo, cone ou barreira na faixa do carro a ate 20 m, medidos ao longo da faixa;
- `cruzamento`: carro a ate 18 m da entrada de um cruzamento em X ou em T, ou dentro dele;
- `via_livre`: nada na faixa do carro em 40 m e a via atual nao termina num cruzamento.

Os casos de fronteira sao descartados. Treino, validacao e teste usam sementes diferentes (mapas
diferentes), entao a validacao e o teste medem generalizacao para pistas novas.

Com o piloto IDM do simulador quase nao ha obstaculo perto: ele freia ou troca de faixa antes.
Por isso a maior parte dos frames de `obstaculo` vem de um obstaculo que o coletor poe na faixa
do carro so para a foto (cones, barreira, carro parado ou triangulo, de 7 a 19 m a frente) e tira
antes do passo seguinte; a cada 8 m de via livre ele sorteia se o frame vai para `via_livre` ou,
com o obstaculo, para `obstaculo`. O manifesto diz a origem de cada um (`colocado` ou `cena`).

## Como a CNN e avaliada

As duas variantes (sem e com augmentation) partem dos mesmos pesos e ficam com os pesos da epoca
de maior accuracy de validacao. O resumo traz a accuracy em treino, validacao e teste, a
diferenca treino - validacao e treino - teste e a accuracy no teste perturbado: os frames de
teste com rotacao de +-15 graus, zoom de 0,2 e flip sorteados uma unica vez. Com 75 frames de
teste, cada frame vale 1,3 ponto percentual: diferencas de poucos pontos entre duas rodadas sao
ruido.

## Limitacoes

- Hough ajusta retas: nas curvas fechadas as linhas da faixa do veiculo saem erradas com mais
  frequencia (a taxa por tipo de bloco esta no `faixas_resumo.json`); dentro dos cruzamentos nao
  ha linhas pintadas.
- O MetaDrive nao tem placas: elas sao acrescentadas a cena por este pacote. Placas com menos de
  ~10 px de raio aparente nao sao classificadas com seguranca (um octogono pequeno parece um
  circulo).
- A CNN e treinada e avaliada so em frames do simulador, e com poucos dados (350 frames de
  treino, 75 de validacao e 75 de teste).
