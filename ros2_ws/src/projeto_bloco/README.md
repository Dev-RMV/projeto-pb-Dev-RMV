# projeto_bloco

ROS 2 Humble + OpenCV 4.5.4 (Python). Um video, um launch, tres nos:

```
ros2_ws/midia/demo.mp4 -> camera_publisher -> /camera/image_raw --+-> color_segmenter -> /vision/status
                                                                    +-> face_features   -> /vision/face_status
```

| No | Faz | Publica / serve |
|---|---|---|
| `camera_publisher` | `cv2.VideoCapture` de camera (`source:=0`) ou video; publica `sensor_msgs/Image` | `/camera/image_raw` |
| `color_segmenter` | BGR -> **HSV**; para cada cor da lista `cores`, `inRange` na faixa `<cor>.lower..<cor>.upper`, morfologia, contornos = objetos | `/vision/mask` (pintada por cor), `/vision/segmented`, servico **`/vision/status`** (`num_objects` total + `colors`/`counts` por cor) |
| `face_features` | **Haar cascade** de rostos; por rosto: olhos (Haar), keypoints + descritores **ORB**, vetor **HOG** | `/vision/faces` (`FaceArray`), `/vision/faces_image`, servico `/vision/face_status` |

Interfaces (pacote `projeto_bloco_interfaces`): `msg/Face`, `msg/FaceArray`, `srv/VisionStatus`, `srv/FaceStatus`.

## Rodar

Na raiz do repositorio: `./scripts/reproduzir.sh` (instala dependencias se faltarem, compila, sobe os
3 nos com janelas OpenCV e imprime os servicos a cada 5 s). Manualmente:

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch projeto_bloco projeto_bloco.launch.py source:=$PWD/midia/demo.mp4 show:=true
```
Em outro terminal (`source ros2_ws/install/setup.bash`):
```bash
ros2 service call /vision/status      projeto_bloco_interfaces/srv/VisionStatus
ros2 service call /vision/face_status projeto_bloco_interfaces/srv/FaceStatus
ros2 topic echo /vision/faces --no-arr
ros2 run rqt_image_view rqt_image_view     # /camera/image_raw, /vision/segmented, /vision/faces_image
```

`ros2_ws/midia/demo.mp4` (79 s em loop, 768x432, 12 fps) alterna blocos de **5 s**: bola azul
quicando (segmentacao: `azul=1` ou `2`, rostos 0) e pessoas (rostos 1-3; `azul=1` na camiseta do
homem). Vermelho, verde e amarelo (cores padrao) so aparecem com webcam ou outro video. Para
acrescentar uma cor: um nome em `cores` + `<cor>.lower`/`<cor>.upper` em `config/params.yaml`
(vermelho usa `lower[H] > upper[H]` para dar a volta pelo 0; `laranja` esta comentado la porque
no demo pega tons de pele e o piso de madeira ao sol).

## Notas

- Sem `/dev/video*` no WSL2 (webcam precisa de `usbipd`); por isso a demo usa video.
- `config/fastdds_profile.xml` (aplicado pelo launch) aumenta o segmento de memoria compartilhada
  do Fast-DDS; sem ele frames grandes chegam a ~5 fps. Para `ros2 run` manual:
  `export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix projeto_bloco)/share/projeto_bloco/config/fastdds_profile.xml`
- `ros2_ws/midia/demo.mp4`: `bola.mp4` (criado pelo Gemini) intercalado com trechos de
  `head-pose-face-detection-{female,male}.mp4` e `face-demographics-walking.mp4` de
  [intel-iot-devkit/sample-videos](https://github.com/intel-iot-devkit/sample-videos) (CC-BY-4.0).
