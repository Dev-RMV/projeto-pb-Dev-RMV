#!/usr/bin/env bash
# reproduzir.sh — reproduz o seu trabalho do zero. O PROFESSOR CORRIGE EXECUTANDO ISTO.
#
# Contrato: executado na raiz do repositório recém-clonado (após ./scripts/setup.sh), deve:
#   1) compilar; 2) obter/gerar todos os artefatos necessários; 3) lançar a demo do TP corrente.
# Todo artefato derivado (modelo, dataset processado, mapa) precisa ter FONTE: ou é gerado
# aqui pelo script que o produz, ou é baixado do link PÚBLICO registrado em ARTEFATOS.md —
# nesse caso, deixe documentado ao lado o comando que o gerou.
#
# Este projeto: basta ./scripts/reproduzir.sh — se faltar dependência ele chama setup.sh sozinho.
#   Sobe camera_publisher -> color_segmenter (/vision/status) + face_features (/vision/face_status)
#   com o video ros2_ws/midia/demo.mp4, abre as janelas OpenCV (se houver DISPLAY) e imprime os
#   dois serviços a cada 5 s. Ctrl+C encerra.
#   Uso: ./scripts/reproduzir.sh                 (demo.mp4)
#        ./scripts/reproduzir.sh source:=0       (camera 0; ou source:=/caminho/outro.mp4)
set -e
cd "$(dirname "$0")/.."
RAIZ="$(pwd)"

# ---------- 0. DEPENDÊNCIAS (setup.sh, só se faltar algo) ----------
if [ ! -f /opt/ros/humble/setup.bash ] || \
   ! dpkg -s ros-humble-cv-bridge python3-opencv opencv-data python3-colcon-common-extensions build-essential >/dev/null 2>&1; then
  echo "[reproduzir] faltam dependências: rodando ./scripts/setup.sh (pede sudo; alguns minutos)"
  ./scripts/setup.sh
fi
source /opt/ros/humble/setup.bash

# ---------- 1. COMPILAR ----------
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..

# ---------- 2. OBTER INSUMOS ----------
# Nenhum artefato derivado: o video de entrada ros2_ws/midia/demo.mp4 esta versionado no repositorio
# (768x432, 12 fps, 79 s: blocos de 5 s alternando bola azul e pessoas — fontes no README do pacote).
VIDEO="$RAIZ/ros2_ws/midia/demo.mp4"
[ -f "$VIDEO" ] || { echo "[reproduzir] video nao encontrado: $VIDEO"; exit 1; }

# ---------- 3. DEMONSTRAÇÃO DO TP CORRENTE ----------
SHOW=true
[ -z "$DISPLAY" ] && { SHOW=false; echo "[reproduzir] sem DISPLAY: rodando sem janelas (veja /vision/segmented e /vision/faces_image no rqt_image_view)"; }
mkdir -p ros2_ws/log
ros2 launch projeto_bloco projeto_bloco.launch.py "source:=$VIDEO" show:=$SHOW "$@" > ros2_ws/log/reproduzir.log 2>&1 &
LAUNCH=$!
trap 'echo; echo "[reproduzir] encerrando"; kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null; exit 0' INT TERM
for i in $(seq 1 30); do
  ros2 service list 2>/dev/null | grep -q "/vision/face_status" && break
  kill -0 $LAUNCH 2>/dev/null || { echo "[reproduzir] launch morreu:"; tail -20 ros2_ws/log/reproduzir.log; exit 1; }
  sleep 1
done
echo "[reproduzir] nós:";     ros2 node list | sed 's/^/   /'
echo "[reproduzir] tópicos:"; ros2 topic list | grep -E "camera|vision" | sed 's/^/   /'
echo "[reproduzir] serviços a cada 5 s (Ctrl+C para sair):"
while kill -0 $LAUNCH 2>/dev/null; do
  S=$(timeout 5 ros2 service call /vision/status      projeto_bloco_interfaces/srv/VisionStatus 2>/dev/null | grep -o "message='[^']*'" | tr -d "'" | cut -d= -f2-)
  F=$(timeout 5 ros2 service call /vision/face_status projeto_bloco_interfaces/srv/FaceStatus   2>/dev/null | grep -o "message='[^']*'" | tr -d "'" | cut -d= -f2-)
  printf '%s  /vision/status: %s | /vision/face_status: %s\n' "$(date +%H:%M:%S)" "${S:-sem resposta}" "${F:-sem resposta}"
  sleep 5
done
echo "[reproduzir] concluído"
