# Evidências do TP2 (geradas em 28/09/2026, WSL2 Ubuntu 22.04 + ROS 2 Humble, CPU)

Detector: YOLOv8s treinado para a classe `alvo` (`ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt`). Vídeo de entrada: `ros2_ws/midia/alvos_dv20.mp4`, com o robô andando e girando num campo de tiro com 5 alvos.

| Arquivo | O que mostra |
|---|---|
| `reproduzir_saida.txt` | Saída completa do `./scripts/reproduzir.sh` num clone limpo: build, nós, interfaces (`ros2 interface list`), action e serviço no ar. **Action client**: cada feedback (estado, progresso, erro, pan/tilt) e o result com métricas de um goal que termina em DISPARO, mais um 2º goal **cancelado** no meio. **Parâmetros**: `ros2 param get/set/load/dump`, com troca de limiar, frequência (10 → 3 → 2 → 10 Hz medidos), tamanho de entrada e modelo (alvos → YOLO11n → alvos) em execução, e valores inválidos recusados |
| `engajamentos.csv` | Results registrados pelo action client (uma linha por goal) |
| `mira_estados.jpg` | `/torre/visao`: a mira da torre em MIRANDO → TRAVADO → DISPARO (a barra de baixo é o progresso do feedback) |
| `rastreamento_yolo_kalman.jpg` | `/vision/rastreamento`: os 5 alvos detectados, com id do ByteTrack e confiança; trilha branca = centro bruto do YOLO, colorida = filtro de Kalman (à direita, durante um giro do robô) |
| `tf_frames.pdf` | `ros2 run tf2_tools view_frames` com o `tp2.launch.py` no ar: árvore de frames do URDF (torre dinâmica pelo `/joint_states`) |
| `gazebo_camera_simulada.png` | Câmera simulada no Gazebo 11 (plugin do URDF, 70°), vendo os alvos e a bola do mundo `alvos.world`. O detector treinado **não** reconhece esses alvos: são 4 anéis geométricos, diferentes da arte usada no treino |

Ainda faltam as capturas de tela do servidor e do cliente rodando juntos (dois terminais) e do RViz com o URDF. Elas são feitas na hora de gravar o vídeo.
