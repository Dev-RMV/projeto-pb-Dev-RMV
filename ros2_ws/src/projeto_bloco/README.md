# projeto_bloco

ROS 2 Humble (Python) do Robo Sniper. Pacotes do workspace:

| Pacote | Conteudo |
|---|---|
| `projeto_bloco` | nos, launch files, `config/params.yaml`, RViz do TP2, `modelos/alvo_yolov8s_640.pt` (detector de alvos) |
| `projeto_bloco_interfaces` | msg `DeteccaoObj`, `DeteccaoArray`, `Face`, `FaceArray`; srv `ControleArma`, `VisionStatus`, `FaceStatus`; action `EngajarAlvo` |
| `projeto_bloco_description` | URDF/xacro do robo, RViz e Gazebo Classic 11 (mundo com alvos) |

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
