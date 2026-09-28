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
| ex.: cnn_faixas.h5 | TP4 | 45 MB | `python3 scripts/treinar_cnn.py --epochs 50` | https://… | ⬜ |
| `ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt` (detector de alvos: YOLOv8s, 1 classe `alvo`) | TP2 | 22,5 MB | **versionado no repositório**. Treinado no projeto separado `C:\_fotos` (dataset sintético do Blender, 2000 + 400 imagens): `python treino/treinar.py` (yolov8s.pt, imgsz 640, 100 épocas, degradação tipo câmera DV20); é o `treino/runs/alvo_yolov8s_640/weights/best.pt`. mAP50 0,951 na validação sintética | no próprio repositório | ✅ |
| `ros2_ws/midia/alvos_dv20.mp4` (vídeo de teste: robô no campo de tiro, 1280x720, 30 fps, 24 s) | TP2 | 9,6 MB | **versionado no repositório**. Render do Blender no projeto `C:\_fotos`: `blender --background --python video/gerar_video.py -- --samples 32` e depois `python video/montar_videos.py` (versão com degradação fixa tipo DV20) | no próprio repositório | ✅ |
| yolo11n.pt (YOLO11 nano, COCO; pesos oficiais, só na demo de troca de modelo) | TP2 | 5,4 MB | baixado pelo `scripts/reproduzir.sh` (ultralytics) para `~/.cache/projeto_bloco/modelos` | https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt | ✅ |
| venv do YOLO (`ros2_ws/.venv`: torch CPU + ultralytics) | TP2 | ~1,4 GB | `scripts/setup.sh` (`pip install -r requirements.txt`) | PyPI + https://download.pytorch.org/whl/cpu | ✅ |
