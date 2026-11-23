# projeto_bloco

ROS 2 Humble (Python) do Robo Sniper. Pacotes do workspace:

| Pacote | Conteudo |
|---|---|
| `projeto_bloco` | nos, launch files (`navegacao.launch.py` do TP4, `bringup.launch.py` do TP3, `tp2.launch.py`, `projeto_bloco.launch.py` do TP1), `config/*.yaml`, `maps/` (mapa da sala), RViz, `modelos/*.pt` (detector de alvos v8s, detector leve v8n para o UNO Q e segmentador v8n-seg) |
| `projeto_bloco_interfaces` | msg `DeteccaoObj`, `DeteccaoArray`, `Face`, `FaceArray`; srv `ControleArma`, `VisionStatus`, `FaceStatus`; action `EngajarAlvo` |
| `projeto_bloco_description` | URDF/xacro do robo, RViz, Gazebo Classic 11 (mundos `alvos.world`, `sala_alvos.world` e `sala_patrulha.world`) e o modelo `alvo_impresso` (folha com a arte real do alvo) |
| `tp4_extras` | itens do TP4 que nao sao do robo: percepcao veicular no MetaDrive (faixas, formas) e CNN de estados da via; tem README proprio |

## TP4: navegacao autonoma com Nav2, patrulha dos postos de tiro, behavior tree e bag

```
                 navegacao.launch.py (um comando; config/params.yaml + navegacao.yaml + nav2.yaml)
 1. gzserver/gzclient (sala_patrulha.world) + robot_state_publisher (URDF)   2. spawn do robo na origem
 3. (depois do spawn) Nav2 = nav2_bringup/bringup_launch.py com config/nav2.yaml:
      map_server (maps/sala_alvos.yaml) -> /map      amcl: /scan + /map -> TF map->odom, /amcl_pose
      planner_server (NavFn + costmap global)  controller_server (Rotation Shim + DWB + costmap local)
      behavior_server, bt_navigator (action /navigate_to_pose), velocity_smoother -> /cmd_vel
    patrulha --goal NavigateToPose--> arvore_navegacao (/navegar_com_pausa) --goal--> /navigate_to_pose
       |  em cada posto de tiro: goal /engajar_alvo (TP2)        ^ /scan: obstaculo a menos de 0,5 m
       v                                                          |   -> cancela, espera 2 s, reenvia
    servidor_engajar <- rastreador_yolo <- camera do Gazebo      obstaculo_surpresa -> /spawn_entity
 4. ros2 bag record (argumento bag:=) grava a sessao   5. encerrar:=true fecha tudo ao fim da patrulha
```

| No | Faz | Publica / serve |
|---|---|---|
| `patrulha` | envia **goals de pose em sequencia** (`nav2_msgs/action/NavigateToPose`), um por posto da lista `postos` (`<posto>.x`, `.y`, `.yaw_graus`, `.engajar`); espera o Nav2 ativo e o AMCL localizado; em posto de tiro arma a torre e envia um goal `/engajar_alvo`; repete goal abortado ate `tentativas` vezes | `/patrulha/rota` (`nav_msgs/Path`), `/patrulha/status` (String), CSV opcional |
| `arvore_navegacao` | **behavior tree em Python puro** (classes `Estado`, `No`, `Sequence`, `Fallback`, condicao e acoes; tick por timer): com o lidar vendo um obstaculo novo a menos de `distancia_parada_m` (0,5 m) no setor frontal, **cancela o goal no Nav2, espera `espera_s` (2 s) parado e reenvia o mesmo goal** | action server `/navegar_com_pausa` (`NavigateToPose`), `/arvore/estado` (OCIOSO, NAVEGANDO, PAUSADO, CHEGOU, FALHOU), `/arvore/distancia_frontal` |
| `obstaculo_surpresa` | quando o robo entra andando na zona de gatilho, cria no Gazebo (`/spawn_entity`) uma caixa a 0,55 m do lidar, na frente do robo: obstaculo fora do mapa que surge ja perto e entra nos 0,5 m da arvore em seguida | `/obstaculo_surpresa/status` |
| `analise_bag` | **validacao offline**: ouve um `ros2 bag play`, refaz a conta da distancia frontal no `/scan` gravado e confere, pelo `/odom`, que cada deteccao a menos de 0,5 m parou o robo por pelo menos 2 s antes de ele retomar; conta os postos alcancados | `trajetoria.png`, `pausa_obstaculo.png`, `validacao.txt`, `validacao.json` |

Arvore do `arvore_navegacao` (o tick desce da raiz a cada 0,1 s):

```
Sequence "Raiz"
  Fallback "CaminhoSeguro"
    CaminhoLivre               Condition  /scan, setor frontal de 60 graus, distancia >= 0,5 m
    Sequence "PausaPorObstaculo"
      InterromperNavegacao     Action     cancela o goal no Nav2 e espera a odometria zerar
      Esperar                  Action     RUNNING ate completar 2 s (relogio do Gazebo)
      RetomarGoal              Action     libera a retomada
  NavegarParaPose              Action     mantem o goal NavigateToPose no Nav2; RUNNING ate chegar
```

### Rodar

Na raiz do repositorio: `./scripts/reproduzir.sh` (compila, sobe a navegacao gravando o bag, mostra o
`ros2 bag info`, reproduz o bag para o `analise_bag` e chama os extras do TP4). Manualmente, cada
terminal com `source ros2_ws/ambiente.sh`:

```bash
cd ros2_ws && colcon build --symlink-install && source ambiente.sh
ros2 launch projeto_bloco navegacao.launch.py                    # patrulha com a behavior tree
ros2 launch projeto_bloco navegacao.launch.py arvore:=false surpresa:=false   # so Nav2 + patrulha
ros2 launch projeto_bloco navegacao.launch.py patrulhar:=false surpresa:=false  # goals a mao:
#   RViz: botao "2D Goal Pose" (vai direto ao Nav2), ou pela arvore:
ros2 action send_goal /navegar_com_pausa nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.3}, orientation: {w: 1.0}}}}" --feedback
# gravar a sessao e validar offline (sem Gazebo nem Nav2 na segunda parte)
ros2 launch projeto_bloco navegacao.launch.py bag:=$HOME/bag_patrulha encerrar:=true
ros2 bag info $HOME/bag_patrulha
ros2 run projeto_bloco analise_bag --ros-args -p pasta:=$HOME/analise &   # espera a reproducao
ros2 bag play $HOME/bag_patrulha --clock
```

### Notas do TP4

- **Mapa**: `maps/sala_alvos.{pgm,yaml}` e o mapa que o SLAM Toolbox construiu no TP3 (5 cm por celula,
  origem do mapa = origem do mundo). O mundo `sala_patrulha.world` e a mesma sala, sem a bola (fica
  abaixo do plano do lidar) e com um caixote que nao esta no mapa.
- **Localizacao**: AMCL com `set_initial_pose` (o robo nasce na origem) e o modelo de movimento
  **omnidirecional**. Com o modelo diferencial a localizacao se perdia em todo giro no lugar: a esteira
  escorrega de lado e esse passo lateral era lido como rotacao de 90 graus (detalhes em
  `docs/decisoes.md`).
- **Costmaps**: global = mapa estatico + camada de obstaculos do lidar + inflacao (0,40 m); local =
  janela de 3 x 3 m no frame `odom`, camada de obstaculos + inflacao. Robo como circulo de 0,20 m.
- **Planejador e controlador**: NavFn (Dijkstra) no costmap global; Rotation Shim + DWB no local (gira
  no lugar ate alinhar com o caminho, depois o DWB segue e desvia). A arvore interna do Nav2 e a que so
  replaneja quando o caminho fica invalido.
- **Duas arvores**: a do `bt_navigator` (XML do Nav2) cuida de planejar, seguir e recuperar; a
  `arvore_navegacao` (Python, deste pacote) decide a pausa de 2 s. Depois de uma pausa, o mesmo
  obstaculo so gera outra se sair do setor e voltar (histerese de 0,1 m), para o Nav2 poder contorna-lo.
- **Odometria**: no Gazebo o plugin das esteiras publica em `/odom` a pose real do robo no mundo; a
  analise usa isso como referencia para medir o erro do AMCL.
- **DDS**: `config/fastdds_profile.xml` agora usa so memoria compartilhada; terminais manuais precisam
  do `source ros2_ws/ambiente.sh` para enxergar os nos. O `ambiente.sh` tambem sobe o daemon da CLI do
  ros2: no WSL2 com rede espelhada o primeiro `ros2 topic list` travava por minutos sem isso.

## TP3: bringup unico (Gazebo + URDF + SLAM Toolbox + percepcao + RViz) e segmentacao por instancia

```
                      bringup.launch.py (um comando; parametros em config/params.yaml + bringup.yaml + slam.yaml)
 1. gzserver/gzclient (sala_alvos.world) + robot_state_publisher (URDF)   2. spawn do robo
 3. (depois do spawn) joint_state_publisher <- /gazebo/joint_states + /torre/joint_states -> /joint_states
    slam_toolbox: /scan -> /map, TF map->odom          (odom->base_footprint vem do diff drive do Gazebo)
    camera do Gazebo -> /camera/image_raw -+-> rastreador_yolo -> /vision/deteccoes -> servidor_engajar -> /torre/visao
                                            +-> segmentador_yolo -> /vision/segmented, /vision/segmentacao
 4. rviz2 -d rviz/bringup.rviz (Map, LaserScan, TF, RobotModel, Odometry, as 3 imagens)
 5. piloto_demo -> /cmd_vel (rota fixa: quadrado de 1,4 m + giro completo) -> mapa construido
```

| No | Faz | Publica / serve |
|---|---|---|
| `segmentador_yolo` | YOLOv8n-seg treinado para a classe `alvo` (mesmo dataset sintetico, poligonos do contorno; 416 px) em `frequencia_hz` sobre o frame mais recente; pinta as mascaras **por classe** com transparencia `alpha` (0,5), contorno por instancia, rotulo e o centroide da mascara (ponto de mira) | `/vision/segmented` (imagem), `/vision/segmentacao` (`DeteccaoArray`: caixa, confianca, id da instancia, cx/cy = centroide da mascara) |
| `piloto_demo` | publica `/cmd_vel` numa rota de passos `reto <m> <m/s>`, `girar <graus> <graus/s>`, `parar <s>` (parametro `rota`), com o relogio do no (tempo simulado), e para no fim | `/cmd_vel` |
| `slam_toolbox` (`async_slam_toolbox_node`, pacote `slam_toolbox`) | SLAM 2D online com o `/scan` do lidar simulado; `config/slam.yaml` (frames `map`/`odom`/`base_footprint`, resolucao 5 cm, alcance 12 m) | `/map`, TF `map -> odom` |

### Rodar

Na raiz do repositorio: `./scripts/reproduzir.sh` (compila, sobe o bringup, gera a arvore de TF, salva
o mapa e engaja um alvo). Manualmente, cada terminal com `source ros2_ws/ambiente.sh` (os launch
files `bringup.launch.py` e `tp2.launch.py` acham o venv do YOLO sozinhos, entao para eles basta o
`source install/setup.bash`; o `ambiente.sh` continua necessario para `ros2 run` dos nos do YOLO):

```bash
cd ros2_ws && colcon build --symlink-install && source ambiente.sh
ros2 launch projeto_bloco bringup.launch.py                    # tudo (gui:=false sem a janela do Gazebo)
#   10 s depois do spawn o cliente_engajar arma a torre e envia 3 goals (bringup.yaml): a torre mira e
#   dispara nos alvos (/torre/visao: MIRANDO -> TRAVADO -> DISPARO); engajar:=false desliga isso
ros2 launch projeto_bloco bringup.launch.py piloto:=false      # e dirigir a mao:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# outro terminal
ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true          # frames.pdf: map -> odom -> base_footprint -> ...
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f mapa --ros-args -p use_sim_time:=true   # mapa.pgm + mapa.yaml
ros2 run projeto_bloco cliente_engajar --ros-args -p use_sim_time:=true # engaja um alvo visto pela camera simulada
ros2 param set /segmentador_yolo alpha 0.8                              # mascaras mais opacas
ros2 param load /rastreador_yolo $(ros2 pkg prefix projeto_bloco)/share/projeto_bloco/config/params_uno_q.yaml  # perfil do UNO Q
```

### Frames de referencia (arvore de TF do bringup)

`map` (fixo no mundo; origem onde o SLAM comecou) -> `odom` (odometria das esteiras: continua, mas
deriva; o SLAM publica `map -> odom` com a correcao) -> `base_footprint` (projecao do robo no chao,
REP 105) -> `base_link` (chassi, na altura dos eixos) -> `camera_link` -> `camera_optical_link` (z
para a frente, x direita, y baixo: convencao das imagens), `mastro_link` -> `lidar_link` (origem do
`/scan`), `torre_pan_link` -> `torre_tilt_link` -> `emissor_link` (origem do feixe), rodas e esteiras.
Fixos vem do URDF; `torre_*` do `servidor_engajar`; rodas do Gazebo; `odom -> base_footprint` do
plugin diff drive; `map -> odom` do SLAM Toolbox.

### Notas do TP3

- **Gazebo Classic 11, nao Harmonic**: decisao registrada em `docs/decisoes.md` (e o simulador
  instalado e validado no TP2). O mundo `sala_alvos.world` tem obstaculos (pilar, caixas, muro) e tres
  folhas A4 com a arte real do alvo como textura (`models/alvo_impresso`, exportado via
  `gazebo_model_path` no `package.xml`), que o detector e o segmentador reconhecem.
- **Tempo simulado**: `config/bringup.yaml` poe `use_sim_time: true` em todos os nos (`/**`). O
  `servidor_engajar` usa `get_clock()` para a latencia das deteccoes e `/torre/joint_states`, entao os
  carimbos ficam coerentes com o `/clock` do Gazebo.
- **Juntas**: o plugin do Gazebo publica so as rodas em `/gazebo/joint_states`; o `joint_state_publisher`
  junta com `/torre/joint_states` e publica `/joint_states`. Dois publicadores em `/joint_states` faziam a
  torre piscar no RViz.
- **Segmentacao**: as mascaras sao amodais (cobrem o alvo inteiro mesmo com algo na frente), como as
  caixas do detector; o centroide da mascara e o ponto de mira. O modelo COCO `yolov8n-seg.pt` tambem
  roda no no (`ros2 param set /segmentador_yolo modelo yolov8n-seg.pt`), mas nao tem a classe `alvo`.
- **CPU**: na simulacao o rastreador roda a 5 Hz e o segmentador a 3 Hz (`bringup.yaml`), porque o Gazebo
  e o SLAM tambem usam a CPU; na CPU do PC o v8n-seg leva ~27 ms por quadro em 416 px.

## TP2: rastreamento de alvos (YOLO) + action de engajamento + URDF

```
midia/alvos_dv20.mp4 -> camera_publisher -/camera/image_raw-> rastreador_yolo -/vision/deteccoes-> servidor_engajar
                                                              (YOLOv8s de alvos     (action /engajar_alvo,
                                                               + ByteTrack + Kalman) servico /arma/controle)
  cliente_engajar --goal/cancel--> /engajar_alvo --feedback/result--> cliente_engajar (log + CSV)
  servidor_engajar -/torre/joint_states-> joint_state_publisher -> robot_state_publisher (URDF, TF) -> RViz
```

| No | Faz | Publica / serve |
|---|---|---|
| `rastreador_yolo` | YOLOv8s treinado para a classe `alvo` (alvo de tiro impresso) + ByteTrack em `frequencia_hz` sobre o frame mais recente; filtro de Kalman (velocidade constante, `filtro_kalman.py`) suaviza o centro de cada id; desenha caixa, trilha bruta (fina, branca) e filtrada (grossa, colorida) | `/vision/deteccoes` (`DeteccaoArray`), `/vision/rastreamento` (imagem) |
| `servidor_engajar` | **action server** `/engajar_alvo`: mira a torre pan/tilt (simulada) no alvo da classe pedida e "dispara" o emissor quando o erro fica <= `tolerancia_px` por `tempo_travar_s`. Controle P + feedforward da velocidade do Kalman; feedback a cada ciclo; aceita cancelamento; aborta por timeout ou desarme | `/torre/joint_states`, `/torre/visao` (imagem com a mira), `/arma/disparo` (`DeteccaoObj`), servico `/arma/controle` (`ControleArma`: trava de seguranca) |
| `cliente_engajar` | **action client**: arma a torre, envia um goal por classe, imprime cada feedback, registra o result (log + CSV) e, com `cancelar_apos_s`, cancela no meio | - |

**Action `EngajarAlvo`**: goal `classe`, `confianca_min`, `tolerancia_px`, `tempo_travar_s`,
`tempo_max_s` (0 = padrao do servidor); feedback `estado` (PROCURANDO, MIRANDO, TRAVADO, DISPARO),
`progresso` 0-1, `erro_px`, `pan/tilt`, `id_alvo`, tempo; result com metricas (disparou, tempos ate
detectar e ate disparar, ciclos com alvo, erro medio e final, posicao final da torre).

### Rodar

Na raiz do repositorio: `./scripts/reproduzir.sh` (instala o que faltar, compila, sobe tudo e roda os
clientes e a demo de parametros). Manualmente, cada terminal com `source ros2_ws/ambiente.sh`
(ROS + workspace + venv do YOLO):

```bash
cd ros2_ws && colcon build --symlink-install && source ambiente.sh
ros2 launch projeto_bloco tp2.launch.py show:=true rviz:=true   # sem source:= usa midia/alvos_dv20.mp4
# outro terminal: arma a torre, engaja um alvo, mostra feedback e result
ros2 run projeto_bloco cliente_engajar
ros2 run projeto_bloco cliente_engajar --ros-args -p tempo_travar_s:=5.0 -p cancelar_apos_s:=2.0
# ou pela CLI
ros2 service call /arma/controle projeto_bloco_interfaces/srv/ControleArma "{armar: true}"
ros2 action send_goal /engajar_alvo projeto_bloco_interfaces/action/EngajarAlvo "{classe: 'alvo'}" --feedback
```

### Parametros (`config/params.yaml`)

Todos os nos leem o YAML pelo launch. O `rastreador_yolo` aceita mudancas em execucao
(`add_on_set_parameters_callback`): `confianca`, `frequencia_hz` (recria o timer), `classes`, `modelo`
/ `dispositivo` (recarrega a rede), `kalman.sigma_a/sigma_z`, `imgsz`... Valores fora da faixa ou
classes desconhecidas sao recusados. O `servidor_engajar` le os seus a cada goal.
`./scripts/demo_parametros.sh` mostra `ros2 param get/set/load/dump`:

```bash
ros2 param set /rastreador_yolo confianca 0.7
ros2 param set /rastreador_yolo frequencia_hz 3.0         # double: com ponto decimal
ros2 param set /rastreador_yolo imgsz 960                   # entrada maior: alvos menores, mais lento
ros2 param set /rastreador_yolo classes "['*']"             # o YOLO11n (COCO) nao tem a classe "alvo"
ros2 param set /rastreador_yolo modelo yolo11n.pt           # troca o modelo em execucao (baixa na 1a vez)
ros2 param load /rastreador_yolo $(ros2 pkg prefix projeto_bloco)/share/projeto_bloco/config/params_economia.yaml
```

No YAML os nos aparecem como `/nome:` porque o `ros2 param load` do Humble so aceita o nome completo.

### URDF (`projeto_bloco_description`)

Esteira tipo tanque (2 motores: roda motriz traseira + roda guia dianteira por lado), camera na frente
(`camera_link` + `camera_optical_link`), lidar num mastro traseiro e torre pan/tilt com o emissor de luz
na ponta do cano (`emissor_link`). No `tp2.launch.py` as juntas da torre vem do `servidor_engajar`.

```bash
ros2 launch projeto_bloco_description display.launch.py      # RViz + sliders das juntas
ros2 launch projeto_bloco_description gazebo.launch.py       # Gazebo 11: /camera/image_raw, /scan, /cmd_vel -> /odom
xacro $(ros2 pkg prefix projeto_bloco_description)/share/projeto_bloco_description/urdf/robo_sniper.urdf.xacro > /tmp/robo.urdf && check_urdf /tmp/robo.urdf
ros2 run tf2_tools view_frames                               # arvore de TF (com o tp2.launch.py no ar)
```

### Notas do TP2

- **YOLO num venv**: `scripts/setup.sh` cria `ros2_ws/.venv` com as versoes de `requirements.txt`
  (torch de CPU, ultralytics 8.4, numpy 1.26 porque o cv_bridge do Humble nao funciona com numpy 2).
  O `ambiente.sh` poe o venv no PYTHONPATH antes do Python do sistema. Pesos oficiais usados so na
  troca de modelo (`yolo11n.pt`, 5,4 MB) vao para `~/.cache/projeto_bloco/modelos`. Para GPU: instale o torch com CUDA no venv
  (`ros2_ws/.venv/bin/pip install torch==2.14.0 torchvision==0.29.0`) e use `dispositivo: cuda:0`.
- **Modelo de alvos** (`modelos/alvo_yolov8s_640.pt`, 22,5 MB): YOLOv8s com fine-tuning para uma
  classe, `alvo` (alvo de tiro redondo impresso, aneis vermelhos e brancos), treinado em 640 px num
  dataset sintetico gerado no Blender (projeto separado; origem em `ARTEFATOS.md`). Validacao
  sintetica: mAP50 0,951 e mAP50-95 0,834; o limiar de melhor F1 e 0,57. Um nome de modelo sem
  caminho e procurado primeiro em `share/projeto_bloco/modelos` e depois em `pasta_modelos`.
- **Video** (`midia/alvos_dv20.mp4`): 1280x720, 30 fps, 24 s. O robo anda, para e gira num campo de
  tiro com 5 alvos (3 fardos, uma folha A4 numa caixa e um recorte numa placa). E render do Blender
  com a "cara" da camera DV20 do robo (campo de visao de 70 graus, por isso `hfov_graus` 70). Na CPU,
  o YOLOv8s leva ~54 ms por quadro.
- **Kalman** calibrado nas trilhas desse video (`sigma_a` 200, `sigma_z` 15): trilha 72% mais lisa,
  com ~5 px de atraso. O no loga essa comparacao a cada 10 s.
- **CPU**: o torch roda com `threads_cpu` = 4 para nao disputar CPU com os outros nos.
- **Problema conhecido**: a cada ~34 s o `/camera/image_raw` perde ~2,5 s de quadros no transporte de
  memoria compartilhada do Fast-DDS (QoS best effort), e o rastreador e a mira param nesse tempo.
  Detalhes em `docs/diario.md`.
- **Janelas**: no `servidor_engajar` o executor e multithread; o `cv2.imshow` fica na thread
  principal porque o HighGUI (Qt) chamado de varias threads travava o servico `/arma/controle`.

## TP1: segmentacao por cor e rostos

```
ros2_ws/midia/demo.mp4 -> camera_publisher -> /camera/image_raw --+-> color_segmenter -> /vision/status
                                                                    +-> face_features   -> /vision/face_status
```

| No | Faz | Publica / serve |
|---|---|---|
| `camera_publisher` | `cv2.VideoCapture` de camera (`source:=0`) ou video; publica `sensor_msgs/Image` (frame `frame_id`) | `/camera/image_raw` |
| `color_segmenter` | BGR -> **HSV**; para cada cor da lista `cores`, `inRange` na faixa `<cor>.lower..<cor>.upper`, morfologia, contornos = objetos | `/vision/mask` (pintada por cor), `/vision/segmented`, servico **`/vision/status`** (`num_objects` total + `colors`/`counts` por cor) |
| `face_features` | **Haar cascade** de rostos; por rosto: olhos (Haar), keypoints + descritores **ORB**, vetor **HOG** | `/vision/faces` (`FaceArray`), `/vision/faces_image`, servico `/vision/face_status` |

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch projeto_bloco projeto_bloco.launch.py source:=$PWD/midia/demo.mp4 show:=true
ros2 service call /vision/status      projeto_bloco_interfaces/srv/VisionStatus
ros2 service call /vision/face_status projeto_bloco_interfaces/srv/FaceStatus
ros2 run rqt_image_view rqt_image_view     # /camera/image_raw, /vision/segmented, /vision/faces_image
```

`ros2_ws/midia/demo.mp4` (79 s em loop, 768x432, 12 fps) alterna blocos de **5 s**: bola azul
quicando (segmentacao: `azul=1` ou `2`, rostos 0) e pessoas (rostos 1-3; `azul=1` na camiseta do
homem). Vermelho, verde e amarelo (cores padrao) so aparecem com webcam ou outro video. Para
acrescentar uma cor: um nome em `cores` + `<cor>.lower`/`<cor>.upper` em `config/params.yaml`
(vermelho usa `lower[H] > upper[H]` para dar a volta pelo 0; `laranja` esta comentado la porque
no demo pega tons de pele e o piso de madeira ao sol).

## Notas gerais

- Sem `/dev/video*` no WSL2 (webcam precisa de `usbipd`); por isso a demo usa video.
- `config/fastdds_profile.xml` (aplicado pelos launch files e pelo `ambiente.sh`) aumenta o segmento
  de memoria compartilhada do Fast-DDS; sem ele frames grandes chegam a ~5 fps.
- `ros2_ws/midia/demo.mp4`: `bola.mp4` (criado pelo Gemini) intercalado com trechos de
  `head-pose-face-detection-{female,male}.mp4` e `face-demographics-walking.mp4` de
  [intel-iot-devkit/sample-videos](https://github.com/intel-iot-devkit/sample-videos) (CC-BY-4.0).
