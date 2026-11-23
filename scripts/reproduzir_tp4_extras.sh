#!/usr/bin/env bash
# reproduzir_tp4_extras.sh — TP4, itens 3 e 4 (pacote tp4_extras): percepcao veicular no MetaDrive
# e CNN de estados da via. Nao depende do ROS: roda com o venv ros2_ws/.venv_tp4.
#
# A partir de um clone limpo, basta ./scripts/reproduzir_tp4_extras.sh. Ele:
#   0) garante o venv ros2_ws/.venv_tp4 (requirements_tp4_extras.txt: MetaDrive, TensorFlow,
#      OpenCV, matplotlib; com placa NVIDIA, tambem as bibliotecas CUDA do TensorFlow) e os assets
#      do MetaDrive (~170 MB, baixados uma vez);
#   1) faixas   detectar_faixas: HSV + mascara de cor, Canny e Hough na camera sintetica, com as
#               faixas sobrepostas ao frame;
#   2) formas   detectar_formas: placas na cena, Hough Circle e Harris Corner;
#   3) dataset  coletar_dataset: 500 frames rotulados (via livre / obstaculo / cruzamento);
#   4) cnn      treinar_cnn: CNN em Keras com e sem data augmentation, curvas e resumo.
# Tudo sem janela; as saidas ficam em ros2_ws/log/tp4_extras/ (o dataset e os modelos sao GERADOS
# aqui, nao estao versionados). Tempo medido, fora a criacao do venv: ~9 min com GPU e ~11 min so
# com CPU (coleta do dataset ~6,5 min; treino ~1 min na GPU, ~3,5 min na CPU).
#
# Uso: ./scripts/reproduzir_tp4_extras.sh                    (as quatro etapas)
#      ./scripts/reproduzir_tp4_extras.sh faixas formas      (so essas etapas)
#      ./scripts/reproduzir_tp4_extras.sh --preparar         (so o venv e os assets)
#      DISPOSITIVO=cpu EPOCAS=20 ./scripts/reproduzir_tp4_extras.sh cnn
#        DISPOSITIVO: auto (padrao: GPU se o TensorFlow a encontrar, senao CPU), gpu ou cpu
set -e
cd "$(dirname "$0")/.."
RAIZ="$(pwd)"
WS="$RAIZ/ros2_ws"
VENV="$WS/.venv_tp4"
SAIDA="$WS/log/tp4_extras"
TF_VERSAO="$(grep -oP '^tensorflow==\K[0-9.]+' requirements_tp4_extras.txt)"
# O PYTHONPATH herdado e descartado: com o ros2_ws/ambiente.sh carregado (caso do reproduzir.sh)
# ele traz o venv do YOLO (numpy 1.26, OpenCV 4.11), que entraria na frente do numpy 2 e do
# OpenCV deste venv.
unset PYTHONPATH

# ---------- 0. DEPENDENCIAS (so se faltar algo) ----------
tem_gpu() { command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; }
if ! "$VENV/bin/python" -c "import metadrive, tensorflow, cv2, matplotlib" >/dev/null 2>&1; then
  echo "[tp4_extras] criando o venv $VENV (requirements_tp4_extras.txt; ~3 GB, alguns minutos)"
  if ! python3 -c "import venv, ensurepip" >/dev/null 2>&1; then
    echo "[tp4_extras] falta o python3-venv: sudo apt install -y python3-venv"; exit 1
  fi
  [ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
  touch "$VENV/COLCON_IGNORE"                  # o colcon nao varre o venv
  "$VENV/bin/pip" install -q -U pip
  "$VENV/bin/pip" install -q -r requirements_tp4_extras.txt
fi
if tem_gpu && ! "$VENV/bin/python" -c "import nvidia.cudnn" >/dev/null 2>&1; then
  echo "[tp4_extras] placa NVIDIA encontrada: instalando as bibliotecas CUDA do TensorFlow (~4 GB)"
  "$VENV/bin/pip" install -q "tensorflow[and-cuda]==$TF_VERSAO" \
    || echo "[tp4_extras] nao foi possivel instalar o extra and-cuda: o treino vai usar a CPU"
fi
export PYTHONPATH="$WS/src/tp4_extras"
"$VENV/bin/python" -m metadrive.pull_asset >/dev/null 2>&1 || true   # assets (so na 1a vez)
if [ "$1" = "--preparar" ]; then
  echo "[tp4_extras] venv pronto: $VENV"; exit 0
fi

# O MetaDrive renderiza a camera por OpenGL e precisa de um servidor X (no WSL, o WSLg). Sem X:
# usa o xvfb-run, se estiver instalado (sudo apt install -y xvfb).
RODAR=()
if [ -z "$DISPLAY" ] && [ ! -S /tmp/.X11-unix/X0 ]; then
  if command -v xvfb-run >/dev/null 2>&1; then
    RODAR=(xvfb-run -a -s "-screen 0 1280x720x24")
  else
    echo "[tp4_extras] aviso: sem servidor X (DISPLAY vazio) e sem xvfb-run; o MetaDrive pode nao abrir"
  fi
fi
modulo() { "${RODAR[@]}" "$VENV/bin/python" -m "tp4_extras.$1" "${@:2}"; }

ETAPAS="${*:-faixas formas dataset cnn}"
mkdir -p "$SAIDA"
for etapa in $ETAPAS; do
  echo; echo "[tp4_extras] ===== $etapa ====="
  case "$etapa" in
    faixas)  modulo deteccao_faixas ;;
    formas)  modulo deteccao_formas ;;
    dataset) modulo coletar_dataset ;;
    cnn)     [ -f "$SAIDA/dataset/manifesto.csv" ] || modulo coletar_dataset
             modulo treinar_cnn --dispositivo "${DISPOSITIVO:-auto}" --epocas "${EPOCAS:-60}" ;;
    *)       echo "[tp4_extras] etapa desconhecida: $etapa (faixas, formas, dataset, cnn)"; exit 1 ;;
  esac
done

echo; echo "[tp4_extras] saidas em $SAIDA:"
for f in faixas/faixas_etapas.jpg faixas/faixas_resumo.json formas/formas_etapas.jpg \
         formas/formas_resumo.json dataset/manifesto.csv dataset_amostras.jpg dataset_resumo.json \
         cnn/cnn_curvas.png cnn/cnn_resumo.txt cnn/cnn_resumo.json \
         cnn/cnn_sem_augmentation.keras cnn/cnn_com_augmentation.keras; do
  [ -e "$SAIDA/$f" ] && echo "   ros2_ws/log/tp4_extras/$f"
done
echo "[tp4_extras] concluido"
