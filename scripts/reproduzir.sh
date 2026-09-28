#!/usr/bin/env bash
# reproduzir.sh — reproduz o seu trabalho do zero. O PROFESSOR CORRIGE EXECUTANDO ISTO.
#
# Contrato: executado na raiz do repositório recém-clonado (após ./scripts/setup.sh), deve:
#   1) compilar; 2) obter/gerar todos os artefatos necessários; 3) lançar a demo do TP corrente.
# Todo artefato derivado (modelo, dataset processado, mapa) precisa ter FONTE: ou é gerado
# aqui pelo script que o produz, ou é baixado do link PÚBLICO registrado em ARTEFATOS.md —
# nesse caso, deixe documentado ao lado o comando que o gerou.
#
# Este projeto (TP2): basta ./scripts/reproduzir.sh — se faltar dependência ele chama setup.sh sozinho.
#   Sobe o tp2.launch.py: camera_publisher (ros2_ws/midia/alvos_dv20.mp4, robo no campo de tiro)
#   -> rastreador_yolo (YOLOv8s treinado para alvos + ByteTrack + filtro de Kalman) -> servidor_engajar
#   (action /engajar_alvo + servico /arma/controle) e o URDF do robo no robot_state_publisher; com
#   DISPLAY abre as janelas OpenCV e o RViz. Neste terminal roda em seguida:
#     1) cliente_engajar: arma a torre e engaja um "alvo" (feedback a cada ciclo, result com
#        metricas no log e em ros2_ws/log/engajamentos.csv);
#     2) cliente_engajar com cancelamento no meio do goal;
#     3) scripts/demo_parametros.sh: ros2 param get/set/load/dump no /rastreador_yolo.
#   Depois o sistema fica no ar ate Ctrl+C (outro terminal: source ros2_ws/ambiente.sh).
#   Uso: ./scripts/reproduzir.sh                 (alvos_dv20.mp4)
#        ./scripts/reproduzir.sh source:=0       (camera 0; ou source:=/caminho/outro.mp4)
#        SEM_DEMO=1 ./scripts/reproduzir.sh      (so sobe o sistema, sem os clientes)
set -e
cd "$(dirname "$0")/.."
RAIZ="$(pwd)"
VENV=ros2_ws/.venv

# ---------- 0. DEPENDÊNCIAS (setup.sh, só se faltar algo) ----------
if [ ! -f /opt/ros/humble/setup.bash ] || \
   ! dpkg -s ros-humble-cv-bridge python3-opencv opencv-data python3-colcon-common-extensions build-essential \
             ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
             ros-humble-tf2-tools >/dev/null 2>&1 || \
   ! "$VENV/bin/python" -c "import ultralytics, lap" >/dev/null 2>&1; then
  echo "[reproduzir] faltam dependências: rodando ./scripts/setup.sh (pede sudo; alguns minutos)"
  ./scripts/setup.sh
fi
source /opt/ros/humble/setup.bash

# ---------- 1. COMPILAR ----------
cd ros2_ws
colcon build --symlink-install
cd ..
source ros2_ws/ambiente.sh          # workspace + venv do YOLO no PYTHONPATH + perfil Fast-DDS

# ---------- 2. OBTER INSUMOS ----------
# Video e modelo versionados no repositorio (origem e comando de geracao em ARTEFATOS.md):
#   ros2_ws/midia/alvos_dv20.mp4  1280x720, 30 fps, 24 s: robo andando e girando num campo de tiro
#                                 com 5 alvos (render do Blender com a "cara" da camera DV20)
#   ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt  YOLOv8s treinado para a classe "alvo"
VIDEO="$RAIZ/ros2_ws/midia/alvos_dv20.mp4"
MODELO="$RAIZ/ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt"
for f in "$VIDEO" "$MODELO"; do
  [ -f "$f" ] || { echo "[reproduzir] arquivo nao encontrado: $f"; exit 1; }
done
echo "[reproduzir] modelo: $(ls -sh "$MODELO")"
# Pesos YOLO11n (COCO) oficiais da Ultralytics: so para a demo de troca de modelo em execucao
# (demo_parametros.sh). Baixados do release publico para a pasta_modelos do config/params.yaml.
MODELOS="$HOME/.cache/projeto_bloco/modelos"
mkdir -p "$MODELOS"
"$VENV/bin/python" -c "from ultralytics import YOLO; YOLO('$MODELOS/yolo11n.pt')" >/dev/null

# ---------- 3. DEMONSTRAÇÃO DO TP CORRENTE ----------
SHOW=true
[ -z "$DISPLAY" ] && { SHOW=false; echo "[reproduzir] sem DISPLAY: sem janelas nem RViz (imagens em /vision/rastreamento e /torre/visao)"; }
mkdir -p ros2_ws/log
# set -m so para iniciar o launch: sem controle de jobs o bash inicia o job em 2o plano com SIGINT
# ignorado e o `kill -INT` do trap nao o encerraria
set -m
ros2 launch projeto_bloco tp2.launch.py "source:=$VIDEO" show:=$SHOW rviz:=$SHOW "$@" > ros2_ws/log/reproduzir.log 2>&1 &
LAUNCH=$!
set +m
trap 'echo; echo "[reproduzir] encerrando"; kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null; exit 0' INT TERM
for i in $(seq 1 60); do   # action server no ar e rastreador com o modelo carregado
  ros2 action list 2>/dev/null | grep -q "/engajar_alvo" && \
    ros2 topic list 2>/dev/null | grep -q "/vision/deteccoes" && break
  kill -0 $LAUNCH 2>/dev/null || { echo "[reproduzir] launch morreu:"; tail -20 ros2_ws/log/reproduzir.log; exit 1; }
  sleep 1
done
echo "[reproduzir] nós:";        ros2 node list --no-daemon --spin-time 2 | sed 's/^/   /'
echo "[reproduzir] interfaces:"; ros2 interface list | grep projeto_bloco_interfaces | sed 's/^ */   /'
echo "[reproduzir] action:";     ros2 action list -t | sed 's/^/   /'
echo "[reproduzir] serviço:";    ros2 service list -t | grep arma | sed 's/^/   /'
echo "[reproduzir] log dos nós: ros2_ws/log/reproduzir.log"

if [ -z "$SEM_DEMO" ]; then
  CSV="$RAIZ/ros2_ws/log/engajamentos.csv"
  echo; echo "[reproduzir] ===== 1) action client: engajar um alvo ====="
  ros2 run projeto_bloco cliente_engajar --ros-args -p "arquivo_csv:=$CSV" || true
  echo; echo "[reproduzir] ===== 2) action client: goal cancelado depois de 2 s ====="
  ros2 run projeto_bloco cliente_engajar --ros-args \
    -p tempo_travar_s:=5.0 -p cancelar_apos_s:=2.0 -p "arquivo_csv:=$CSV" || true
  echo; echo "[reproduzir] ===== 3) parametros: YAML + alteracao dinamica ====="
  bash ./scripts/demo_parametros.sh || true
  echo; echo "[reproduzir] results registrados em $CSV"
fi
echo; echo "[reproduzir] sistema no ar (Ctrl+C encerra). Em outro terminal: source ros2_ws/ambiente.sh"
echo "   ros2 run projeto_bloco cliente_engajar"
echo "   ros2 action send_goal /engajar_alvo projeto_bloco_interfaces/action/EngajarAlvo \"{classe: 'alvo'}\" --feedback"
wait $LAUNCH
echo "[reproduzir] concluído"
