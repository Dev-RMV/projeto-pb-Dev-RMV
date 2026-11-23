# Artefatos e Vídeos — links e fontes

> ⚠️ **Todo link abaixo deve estar acessível publicamente** (teste em **aba anônima**, sem login). Link quebrado/privado = artefato/vídeo **inexistente** para a correção. Responsabilidade sua.
> Vídeos: YouTube "público" ou "não listado" (nunca "privado") **ou** link de drive com "qualquer pessoa com o link".

## Vídeos por TP (formato fixo — não altere os marcadores)
| TP | Link (YouTube/drive) | Testado em aba anônima |
|---|---|---|
| TP1 | <!-- PB:VIDEO-TP1 -->https://www.youtube.com/watch?v=WrpPA5194_8<!-- /PB:VIDEO-TP1 --> | ⬜ |
| TP2 | <!-- PB:VIDEO-TP2 -->https://youtu.be/VQyR84piW8Q<!-- /PB:VIDEO-TP2 --> | ⬜ |
| TP3 | <!-- PB:VIDEO-TP3 -->—<!-- /PB:VIDEO-TP3 --> | ⬜ |
| TP4 | <!-- PB:VIDEO-TP4 -->—<!-- /PB:VIDEO-TP4 --> | ⬜ |
| TP5 | <!-- PB:VIDEO-TP5 -->—<!-- /PB:VIDEO-TP5 --> | ⬜ |
| Final | <!-- PB:VIDEO-FINAL -->—<!-- /PB:VIDEO-FINAL --> | ⬜ |

_Preencha entre os marcadores, ex.:_ `<!-- PB:VIDEO-TP1 -->https://youtu.be/abc123<!-- /PB:VIDEO-TP1 -->` _(e replique o link no front-matter do README)._

## Artefatos grandes (fora do git)
Todo artefato derivado tem **fonte**: o comando/script que o gera (em `scripts/`) e/ou o link público.

| Artefato | TP | Tamanho | Como é gerado (comando) | Link público | Testado ⬜ |
|---|---|---|---|---|---|
| `ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt` (detector de alvos: YOLOv8s, 1 classe `alvo`) | TP2 | 22,5 MB | **versionado no repositório**. Treinado no projeto separado `C:\_fotos` (dataset sintético do Blender, 2000 + 400 imagens): `python treino/treinar.py` (yolov8s.pt, imgsz 640, 100 épocas, degradação tipo câmera DV20); é o `treino/runs/alvo_yolov8s_640/weights/best.pt`. mAP50 0,951 na validação sintética | no próprio repositório | ✅ |
| `ros2_ws/src/projeto_bloco/modelos/alvo_yolov8n_seg_416.pt` (segmentação por instância: YOLOv8n-seg, 1 classe `alvo`) | TP3 | 6,5 MB | **versionado no repositório**. Treinado em `C:\_fotos` sobre o mesmo dataset sintético, com polígonos gerados sem re-render: `blender --background --factory-startup --python gerador\gerar_labels_seg.py -- --out dataset_seg` e depois `python treino/treinar.py --modelo yolov8n-seg.pt --data dataset_seg/data.yaml --imgsz 416 --epocas 100`; é o `treino/runs/alvo_yolov8n_seg_416/weights/best.pt` | no próprio repositório | ✅ |
| `ros2_ws/src/projeto_bloco/modelos/alvo_yolov8n_416.pt` (detector leve para o Arduino UNO Q: YOLOv8n 416, 1 classe `alvo`; perfil `config/params_uno_q.yaml`) | TP3 | 6,2 MB | **versionado no repositório**. `python treino/treinar.py --modelo yolov8n.pt --imgsz 416 --epocas 100` no `C:\_fotos` (23/10/2026); é o `treino/runs/alvo_yolov8n_416/weights/best.pt`. mAP50 0,893 na validação sintética | no próprio repositório | ✅ |
| `ros2_ws/src/projeto_bloco_description/models/alvo_impresso/materials/textures/alvo_impresso.png` (arte real do alvo como textura no Gazebo) | TP3 | 0,24 MB | **versionado**. `gerador/arte/alvo_principal.png` do `C:\_fotos` (gerado por `gerador/arte_alvo.py`) reduzido para 1024 px: `magick alvo_principal.png -resize 1024x alvo_impresso.png` | no próprio repositório | ✅ |
| `ros2_ws/midia/alvos_dv20.mp4` (vídeo de teste: robô no campo de tiro, 1280x720, 30 fps, 24 s) | TP2 | 9,6 MB | **versionado no repositório**. Render do Blender no projeto `C:\_fotos`: `blender --background --python video/gerar_video.py -- --samples 32` e depois `python video/montar_videos.py` (versão com degradação fixa tipo DV20) | no próprio repositório | ✅ |
| yolo11n.pt (YOLO11 nano, COCO; pesos oficiais, só na demo de troca de modelo) | TP2 | 5,4 MB | baixado pelo `scripts/reproduzir.sh` (ultralytics) para `~/.cache/projeto_bloco/modelos` | https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt | ✅ |
| venv do YOLO (`ros2_ws/.venv`: torch CPU + ultralytics) | TP2 | ~1,4 GB | `scripts/setup.sh` (`pip install -r requirements.txt`) | PyPI + https://download.pytorch.org/whl/cpu | ✅ |
| `ros2_ws/src/projeto_bloco/maps/sala_alvos.pgm` + `.yaml` (mapa de ocupação da sala, 119 × 119 células de 5 cm; mapa estático do Nav2) | TP4 | 14 kB | **versionado no repositório**. É o `/map` do SLAM Toolbox salvo pelo bringup do TP3: `./scripts/reproduzir_tp3.sh` (passo 3: `ros2 run nav2_map_server map_saver_cli -f ros2_ws/log/mapa_tp3`), copiado como `sala_alvos.{pgm,yaml}` | no próprio repositório | ✅ |
| `ros2_ws/log/tp4/bag_patrulha/` (bag da sessão de navegação: cerca de 85 s, 11 mil mensagens em 14 tópicos) | TP4 | ~7 MB | gerado a cada execução do `./scripts/reproduzir.sh` (`ros2 launch projeto_bloco navegacao.launch.py bag:=<pasta> encerrar:=true`, que roda o `ros2 bag record`). A sessão usada no relatório está versionada em `docs/evidencias/tp4/bag_patrulha/` | no próprio repositório (cópia da sessão do relatório) | ✅ |
| `ros2_ws/log/tp4/trajetoria.png`, `pausa_obstaculo.png`, `validacao.txt`, `validacao.json`, `patrulha.csv` (gráfico da trajetória, pausa no tempo e validação offline) | TP4 | < 1 MB | gerados a cada execução do `./scripts/reproduzir.sh`: `ros2 bag play <bag> --clock` para o nó `ros2 run projeto_bloco analise_bag`; cópias da sessão do relatório em `docs/evidencias/tp4/` | no próprio repositório (cópias) | ✅ |
| venv do `tp4_extras` (`ros2_ws/.venv_tp4`: MetaDrive 0.4.3 + TensorFlow 2.21) | TP4 | 3,2 GB (só CPU) ou 7,3 GB (com as bibliotecas CUDA) | `scripts/setup.sh` ou `./scripts/reproduzir_tp4_extras.sh --preparar` (`pip install -r requirements_tp4_extras.txt`; com placa NVIDIA, também `tensorflow[and-cuda]==2.21.0`) | PyPI | ✅ |
| assets do MetaDrive (modelos e texturas do simulador) | TP4 | 168 MB | baixados pelo próprio simulador na primeira execução (`python -m metadrive.pull_asset`, chamado pelo script acima) | https://github.com/metadriverse/metadrive/releases/download/MetaDrive-0.4.3/assets.zip | ✅ |
| `ros2_ws/log/tp4_extras/dataset/` (500 frames 320 × 180 do MetaDrive em 3 classes, treino / validação / teste, + `manifesto.csv`) | TP4 | 7,1 MB | gerado por `./scripts/reproduzir_tp4_extras.sh dataset` (`ros2 run tp4_extras coletar_dataset`; sementes fixas, cerca de 6,5 min). O manifesto da coleta do relatório está em `docs/evidencias/tp4/extras/dataset_manifesto.csv` | gerado pelo script (não versionado) | ✅ |
| `ros2_ws/log/tp4_extras/cnn/cnn_sem_augmentation.keras` e `cnn_com_augmentation.keras` (CNN de estados da via, 1,06 milhão de parâmetros) | TP4 | 12,8 MB cada | gerados por `./scripts/reproduzir_tp4_extras.sh cnn` (`ros2 run tp4_extras treinar_cnn`; 60 épocas por variante: cerca de 1 min com GPU, 3,5 min só com CPU). Curvas e resumo da rodada do relatório em `docs/evidencias/tp4/extras/` | gerados pelo script (não versionados) | ✅ |
