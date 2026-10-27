# Evidências do TP3 (geradas em 27/10/2026, WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11, CPU)

Bringup: `./scripts/reproduzir.sh` num clone limpo (`gui:=false rviz:=false`, câmera do Gazebo renderizando no display do WSLg). Modelos: detector `alvo_yolov8s_640.pt` (TP2) e segmentador `alvo_yolov8n_seg_416.pt` (TP3), ambos em `ros2_ws/src/projeto_bloco/modelos/`.

| Arquivo | O que mostra |
|---|---|
| `reproduzir_saida.txt` | Saída completa do `./scripts/reproduzir.sh`: build, nós e tópicos no ar, taxas (`/scan`, `/camera/image_raw`, `/vision/segmented`, `/vision/deteccoes`), **action client** engajando um alvo da simulação, `view_frames`, `tf2_echo map base_footprint` antes e depois da rota, mapa salvo e quadros gravados |
| `reproduzir.log` | Log dos nós do launch (ordem de subida, SLAM Toolbox, piloto passo a passo, servidor de engajamento) |
| `frames_tp3.pdf` / `frames_tp3.gv` / `frames_tp3.png` | `ros2 run tf2_tools view_frames` com o bringup no ar: `map → odom → base_footprint → base_link → sensores/torre/rodas` |
| `mapa_tp3.pgm` + `mapa_tp3.yaml` / `mapa_tp3.png` | `/map` do SLAM Toolbox salvo com `map_saver_cli` ao fim da rota do `piloto_demo` (5 cm por célula; o png é o pgm ampliado 3×) |
| `rviz_bringup.jpg` | **Captura de tela** do RViz2 com a configuração salva (`rviz/bringup.rviz`) e o bringup no ar, ao fim da rota: mapa, scan, TF, robô, trilha da odometria e as três imagens da percepção (janela capturada pelo X do WSLg com `xwd`) |
| `gazebo_bringup.jpg` | Captura de tela da janela do Gazebo Classic 11 com o mundo `sala_alvos.world` e o robô (se presente; senão, está no vídeo) |
| `gazebo_camera.jpg` | `/camera/image_raw` da câmera simulada (640×480, 15 Hz), no início do bringup, com o robô olhando para os alvos |
| `gazebo_segmented.jpg` | `/vision/segmented` sobre a câmera do Gazebo: máscaras da classe `alvo` a 50%, contorno e centroide (3 instâncias) |
| `gazebo_rastreamento.jpg` | `/vision/rastreamento` (TP2) sobre a câmera do Gazebo |
| `gazebo_torre_visao.jpg` | `/torre/visao`: a mira do `servidor_engajar` na simulação |
| `engajamentos_tp3.csv` | Result do action client na simulação (uma linha por goal) |
| `segmentacao_video.jpg` | `/vision/segmented` no vídeo do robô (`alvos_dv20.mp4`, 1280×720): 4 instâncias com confiança 0,90-0,94, 25 ms por quadro |
| `labels_seg_mosaico.jpg` | Conferência dos polígonos de segmentação gerados sem re-render (`C:\_fotos\gerador\gerar_labels_seg.py`): máscara vermelha a 50% e contorno amarelo em imagens de validação |
| `treino_seg_results.png` | Curvas do treino do YOLOv8n-seg (`treino/runs/alvo_yolov8n_seg_416/results.png`) |
| `arquitetura_veicular.svg` | Figura própria da arquitetura E/E veicular (Ethernet / CAN FD / LIN) e do Robô Sniper, usada na análise da tarefa 4 |

As capturas do Gazebo e do RViz em uso, com o terminal do comando único, aparecem também no vídeo.
