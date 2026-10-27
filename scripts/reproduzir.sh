#!/usr/bin/env bash
# reproduzir.sh — reproduz o seu trabalho do zero. O PROFESSOR CORRIGE EXECUTANDO ISTO.
#
# Contrato: executado na raiz do repositório recém-clonado (após ./scripts/setup.sh), deve:
#   1) compilar; 2) obter/gerar todos os artefatos necessários; 3) lançar a demo do TP corrente.
# Todo artefato derivado (modelo, dataset processado, mapa) precisa ter FONTE: ou é gerado
# aqui pelo script que o produz, ou é baixado do link PÚBLICO registrado em ARTEFATOS.md —
# nesse caso, deixe documentado ao lado o comando que o gerou.
#
# Este projeto (TP3): basta ./scripts/reproduzir.sh — se faltar dependência ele chama setup.sh sozinho.
#   Sobe o bringup.launch.py com UM comando: Gazebo Classic 11 (mundo sala_alvos.world: sala com
#   obstáculos e alvos impressos com a arte real) + robot_state_publisher (URDF) + spawn do robô
#   + SLAM Toolbox (/scan -> /map, TF map->odom) + percepção do TP2 (rastreador_yolo com Kalman,
#   servidor_engajar) + segmentador_yolo do TP3 (/vision/segmented) + RViz2 (rviz/bringup.rviz)
#   + piloto_demo (rota fixa do config/params.yaml para o mapa ser construído). Parâmetros via YAML
#   (config/params.yaml, bringup.yaml, slam.yaml). Sem DISPLAY, sobe sem as janelas.
#   Neste terminal roda em seguida, depois de o SLAM publicar o mapa:
#     1) cliente_engajar: arma a torre e engaja um "alvo" visto pela câmera simulada, com o robô
#        ainda parado na origem (o piloto espera 30 s antes de andar); CSV no log;
#     2) ros2 run tf2_tools view_frames  -> ros2_ws/log/frames_tp3.pdf (árvore map->odom->base...);
#     3) espera a rota do piloto e salva o mapa (map_saver_cli) -> ros2_ws/log/mapa_tp3.{pgm,yaml};
#     4) salvar_imagens: um quadro de /vision/segmented, /vision/rastreamento, /torre/visao e da
#        câmera -> ros2_ws/log/evidencias/.
#   Depois o sistema fica no ar até Ctrl+C (outro terminal: source ros2_ws/ambiente.sh).
#   Uso: ./scripts/reproduzir.sh                 (bringup completo)
#        ./scripts/reproduzir.sh gui:=false      (sem a janela do Gazebo; qualquer arg:= vai ao launch)
#        SEM_DEMO=1 ./scripts/reproduzir.sh      (só sobe o sistema, sem view_frames/mapa/cliente)
#        ./scripts/reproduzir_tp2.sh             (demo do TP2: vídeo + action + parâmetros)
set -e
cd "$(dirname "$0")/.."
RAIZ="$(pwd)"
VENV=ros2_ws/.venv

# ---------- 0. DEPENDÊNCIAS (setup.sh, só se faltar algo) ----------
if [ ! -f /opt/ros/humble/setup.bash ] || \
   ! dpkg -s ros-humble-cv-bridge python3-opencv opencv-data python3-colcon-common-extensions build-essential \
             ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
             ros-humble-tf2-tools ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox ros-humble-nav2-map-server \
             >/dev/null 2>&1 || \
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
# Modelos versionados no repositório (origem e comando de geração em ARTEFATOS.md):
#   ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt      detector de alvos (TP2, padrão no PC)
#   ros2_ws/src/projeto_bloco/modelos/alvo_yolov8n_416.pt      detector para o Arduino UNO Q (params_uno_q.yaml)
#   ros2_ws/src/projeto_bloco/modelos/alvo_yolov8n_seg_416.pt  segmentação por instância (TP3)
# Mundo, modelo do alvo impresso (arte real como textura) e URDF: projeto_bloco_description.
for f in ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt ros2_ws/src/projeto_bloco/modelos/alvo_yolov8n_seg_416.pt \
         ros2_ws/src/projeto_bloco_description/models/alvo_impresso/materials/textures/alvo_impresso.png \
         ros2_ws/midia/alvos_dv20.mp4; do
  [ -f "$f" ] || { echo "[reproduzir] arquivo nao encontrado: $f"; exit 1; }
done
echo "[reproduzir] modelos: $(ls -sh ros2_ws/src/projeto_bloco/modelos/*.pt | tr '\n' ' ')"
# Modelos do ground_plane e do sun do Gazebo: vêm com o gazebo11 (/usr/share/gazebo-11/models).

# ---------- 3. DEMONSTRAÇÃO DO TP CORRENTE ----------
GUI=true
[ -z "$DISPLAY" ] && { GUI=false; echo "[reproduzir] sem DISPLAY: Gazebo sem janela e sem RViz (imagens em /vision/segmented, /vision/rastreamento, /torre/visao)"; }
mkdir -p ros2_ws/log
# set -m so para iniciar o launch: sem controle de jobs o bash inicia o job em 2o plano com SIGINT
# ignorado e o `kill -INT` do trap nao o encerraria
set -m
# engajar:=false: o proprio script roda o cliente_engajar abaixo (com CSV) em vez do automatico do launch
ros2 launch projeto_bloco bringup.launch.py gui:=$GUI rviz:=$GUI show:=false engajar:=false "$@" > ros2_ws/log/reproduzir.log 2>&1 &
LAUNCH=$!
set +m
# Ctrl+C, kill ou qualquer saída do script (set -e) encerram o launch inteiro (Gazebo, SLAM, nós)
encerrar() {
  trap - INT TERM EXIT; echo; echo "[reproduzir] encerrando"
  kill -INT $LAUNCH 2>/dev/null
  for i in $(seq 1 20); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done   # o Gazebo demora a sair
  kill -0 $LAUNCH 2>/dev/null && { echo "[reproduzir] forçando"; kill -KILL -- -$LAUNCH 2>/dev/null; }
  wait $LAUNCH 2>/dev/null; exit 0
}
trap encerrar INT TERM EXIT
echo "[reproduzir] subindo Gazebo, SLAM e percepção (log em ros2_ws/log/reproduzir.log)"
for i in $(seq 1 180); do   # SLAM com mapa, segmentador e action server no ar
  ros2 topic list 2>/dev/null | grep -q "^/map$" && \
    ros2 topic list 2>/dev/null | grep -q "^/vision/segmented$" && \
    ros2 action list 2>/dev/null | grep -q "/engajar_alvo" && break
  kill -0 $LAUNCH 2>/dev/null || { echo "[reproduzir] launch morreu:"; tail -20 ros2_ws/log/reproduzir.log; exit 1; }
  sleep 1
done
echo "[reproduzir] nós:";    ros2 node list --no-daemon --spin-time 3 | sed 's/^/   /'
echo "[reproduzir] tópicos:"; ros2 topic list | grep -E "^/(map|scan|odom|tf|camera/image_raw|vision|torre|joint_states|cmd_vel)" | sed 's/^/   /'
echo "[reproduzir] taxas:"
for t in /scan /camera/image_raw /vision/segmented /vision/deteccoes; do
  printf '   %-24s' "$t"; timeout 6 ros2 topic hz $t 2>/dev/null | grep -m1 average || echo "sem dados (a câmera do Gazebo precisa de um DISPLAY para renderizar)"
done

if [ -z "$SEM_DEMO" ]; then
  echo; echo "[reproduzir] ===== 1) action client: engajar um alvo visto pela câmera simulada (robô ainda parado) ====="
  timeout 60 ros2 run projeto_bloco cliente_engajar --ros-args -p "arquivo_csv:=$RAIZ/ros2_ws/log/engajamentos_tp3.csv" -p use_sim_time:=true || true
  echo; echo "[reproduzir] ===== 2) árvore de TF (tf2_tools view_frames) ====="
  ( cd ros2_ws/log && timeout 30 ros2 run tf2_tools view_frames -o frames_tp3 --ros-args -p use_sim_time:=true >/dev/null 2>&1 || true )
  ls ros2_ws/log/frames_tp3.pdf >/dev/null 2>&1 && echo "   ros2_ws/log/frames_tp3.pdf" || echo "   (view_frames nao gerou o pdf)"
  echo "   map -> base_footprint agora:"
  timeout 6 ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true 2>/dev/null | grep -m2 -E "Translation|RPY" | sed 's/^/   /' || true
  echo; echo "[reproduzir] ===== 3) rota do piloto_demo (quadrado de 1,4 m + giro; ~45 s) e mapa do SLAM ====="
  for i in $(seq 1 7); do sleep 10; printf '   %ds...' $((i * 10)); done; echo
  timeout 6 ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true 2>/dev/null | grep -m2 -E "Translation|RPY" | sed 's/^/   /' || true
  timeout 25 ros2 run nav2_map_server map_saver_cli -f ros2_ws/log/mapa_tp3 --ros-args -p use_sim_time:=true 2>&1 | grep -E "Map saved|Failed|error" | sed 's/^/   /' || true
  ls ros2_ws/log/mapa_tp3.pgm >/dev/null 2>&1 && echo "   ros2_ws/log/mapa_tp3.pgm + .yaml"
  echo; echo "[reproduzir] ===== 4) quadros das imagens publicadas (ros2_ws/log/evidencias) ====="
  timeout 40 ros2 run projeto_bloco salvar_imagens --ros-args -p "pasta:=$RAIZ/ros2_ws/log/evidencias" -p timeout_s:=30.0 2>&1 | grep -E "\->|timeout" | sed 's/^/   /' || true
fi
echo; echo "[reproduzir] sistema no ar (Ctrl+C encerra). Em outro terminal: source ros2_ws/ambiente.sh"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard      # dirigir (piloto_demo ja parou)"
echo "   ros2 run projeto_bloco cliente_engajar"
echo "   ros2 param load /rastreador_yolo \$(ros2 pkg prefix projeto_bloco)/share/projeto_bloco/config/params_uno_q.yaml"
wait $LAUNCH
echo "[reproduzir] concluído"
