#!/usr/bin/env bash
# reproduzir.sh — reproduz o seu trabalho do zero. O PROFESSOR CORRIGE EXECUTANDO ISTO.
#
# Contrato: executado na raiz do repositório recém-clonado (após ./scripts/setup.sh), deve:
#   1) compilar; 2) obter/gerar todos os artefatos necessários; 3) lançar a demo do TP corrente.
# Todo artefato derivado (modelo, dataset processado, mapa) precisa ter FONTE: ou é gerado
# aqui pelo script que o produz, ou é baixado do link PÚBLICO registrado em ARTEFATOS.md —
# nesse caso, deixe documentado ao lado o comando que o gerou.
#
# Este projeto (TP4): basta ./scripts/reproduzir.sh — se faltar dependência ele chama setup.sh sozinho.
#   Parte 1, navegação autônoma do Robô Sniper (pacote projeto_bloco):
#     1) sobe o navegacao.launch.py com UM comando: Gazebo Classic 11 (mundo sala_patrulha.world: a
#        sala do mapa e um caixote que não está nele) + URDF + Nav2 (map_server com o mapa salvo no
#        TP3, AMCL, costmaps global e local, planejador NavFn, controlador DWB, bt_navigator) +
#        percepção do TP2 + RViz2 + arvore_navegacao (behavior tree em Python: lidar a menos de
#        0,5 m interrompe a navegação, espera 2 s e retoma o goal) + obstaculo_surpresa (caixa que
#        aparece na frente do robô) + patrulha (goals de pose em sequência pelos postos de tiro;
#        em cada posto engaja o alvo). A sessão inteira é gravada com `ros2 bag record` e o launch
#        termina sozinho quando a patrulha acaba;
#     2) `ros2 bag info` da sessão gravada;
#     3) reprodução offline (`ros2 bag play`, sem Gazebo nem Nav2) para o nó analise_bag, que
#        confere a pausa de 2 s, os postos alcançados e gera o gráfico da trajetória.
#   Parte 2, itens de percepção veicular do TP4 (pacote tp4_extras, MetaDrive + CNN):
#     ./scripts/reproduzir_tp4_extras.sh, chamado no fim (SEM_EXTRAS=1 pula essa parte).
#   Saídas em ros2_ws/log/tp4/: bag_patrulha/ (o bag), patrulha.csv, validacao.txt e .json,
#   trajetoria.png, pausa_obstaculo.png e reproduzir.log (log de todos os nós).
#   Uso: ./scripts/reproduzir.sh                 (tudo; sem DISPLAY sobe sem janelas)
#        ./scripts/reproduzir.sh gui:=false      (sem a janela do Gazebo; qualquer arg:= vai ao launch)
#        ./scripts/reproduzir.sh arvore:=false surpresa:=false   (só Nav2 + patrulha, sem a árvore)
#        SEM_EXTRAS=1 ./scripts/reproduzir.sh    (só a navegação)
#        ./scripts/reproduzir_tp3.sh             (demo do TP3: bringup com SLAM e segmentação)
#        ./scripts/reproduzir_tp2.sh             (demo do TP2: vídeo + action + parâmetros)
set -e
cd "$(dirname "$0")/.."
RAIZ="$(pwd)"
VENV=ros2_ws/.venv
SAIDA=ros2_ws/log/tp4
TEMPO_MAX=${TEMPO_MAX:-600}        # s: limite da patrulha antes de encerrar o launch à força
TAXA_PLAY=${TAXA_PLAY:-2.0}        # velocidade da reprodução offline do bag

# ---------- 0. DEPENDÊNCIAS (setup.sh, só se faltar algo) ----------
if [ ! -f /opt/ros/humble/setup.bash ] || \
   ! dpkg -s ros-humble-cv-bridge python3-opencv opencv-data python3-colcon-common-extensions build-essential \
             ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
             ros-humble-tf2-tools ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox ros-humble-nav2-map-server \
             ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rosbag2 python3-matplotlib \
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
# Versionados no repositório (origem e comando de geração em ARTEFATOS.md):
#   ros2_ws/src/projeto_bloco/maps/sala_alvos.{pgm,yaml}   mapa do SLAM Toolbox (TP3), usado pelo AMCL
#   ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt  detector de alvos (TP2), para o disparo nos postos
# O bag da sessão, os gráficos e a validação são gerados aqui, a cada execução.
for f in ros2_ws/src/projeto_bloco/maps/sala_alvos.pgm ros2_ws/src/projeto_bloco/maps/sala_alvos.yaml \
         ros2_ws/src/projeto_bloco/config/nav2.yaml ros2_ws/src/projeto_bloco/modelos/alvo_yolov8s_640.pt; do
  [ -f "$f" ] || { echo "[reproduzir] arquivo nao encontrado: $f"; exit 1; }
done

# ---------- 3. DEMONSTRAÇÃO DO TP CORRENTE ----------
GUI=true
[ -z "$DISPLAY" ] && { GUI=false; echo "[reproduzir] sem DISPLAY: Gazebo sem janela e sem RViz; a câmera simulada não renderiza, então os postos ficam sem disparo (a navegação usa só o lidar)"; }
mkdir -p "$SAIDA"
rm -rf "$SAIDA/bag_patrulha" "$SAIDA/patrulha.csv"
BAG="$RAIZ/$SAIDA/bag_patrulha"

echo; echo "[reproduzir] ===== 1) patrulha com Nav2 + behavior tree, gravando o bag (log em $SAIDA/reproduzir.log) ====="
# set -m so para iniciar o launch: sem controle de jobs o bash inicia o job em 2o plano com SIGINT
# ignorado e o `kill -INT` do trap nao o encerraria
set -m
ros2 launch projeto_bloco navegacao.launch.py gui:=$GUI rviz:=$GUI show:=false encerrar:=true \
     bag:="$BAG" csv:="$RAIZ/$SAIDA/patrulha.csv" "$@" > "$SAIDA/reproduzir.log" 2>&1 &
LAUNCH=$!
set +m
# Ctrl+C, kill ou qualquer saída do script (set -e) encerram o launch inteiro (Gazebo, Nav2, nós, bag)
encerrar() {
  trap - INT TERM EXIT; echo; echo "[reproduzir] encerrando"
  kill -INT $LAUNCH 2>/dev/null
  for i in $(seq 1 20); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done   # o Gazebo demora a sair
  kill -0 $LAUNCH 2>/dev/null && { echo "[reproduzir] forçando"; kill -KILL -- -$LAUNCH 2>/dev/null; }
  wait $LAUNCH 2>/dev/null; exit 0
}
trap encerrar INT TERM EXIT
# limite de tempo: se a patrulha travar, encerra o launch (o bag fecha) e a análise mostra o que houve
( sleep "$TEMPO_MAX"; kill -0 $LAUNCH 2>/dev/null && { echo "[reproduzir] passou de ${TEMPO_MAX} s: encerrando o launch"; kill -INT $LAUNCH; } ) &
VIGIA=$!
# acompanha na tela só os nós do projeto, até o launch terminar (fim da patrulha)
tail -n +1 -F "$SAIDA/reproduzir.log" --pid=$LAUNCH 2>/dev/null \
  | grep --line-buffered -E "\[(patrulha|arvore_navegacao|obstaculo_surpresa)\]: |\[navegacao\]|Managed nodes are active" \
  | grep --line-buffered -v "feedback:" \
  | sed -u -E 's/\x1b\[[0-9;]*m//g; s/^\[[a-z_0-9.-]+\] //; s/\[[0-9]+\.[0-9]+\] //; s/^/   /' || true
wait $LAUNCH 2>/dev/null || true
trap - INT TERM EXIT
pkill -P $VIGIA 2>/dev/null || true; kill $VIGIA 2>/dev/null || true     # o sleep do limite de tempo
[ -f "$BAG/metadata.yaml" ] || { echo "[reproduzir] o bag não foi gravado; fim do log:"; tail -30 "$SAIDA/reproduzir.log"; exit 1; }

echo; echo "[reproduzir] ===== 2) ros2 bag info $SAIDA/bag_patrulha ====="
ros2 bag info "$BAG" | sed 's/^/   /'

echo; echo "[reproduzir] ===== 3) reprodução offline (ros2 bag play a ${TAXA_PLAY}x, sem Gazebo nem Nav2) + analise_bag ====="
ros2 run projeto_bloco analise_bag --ros-args --params-file ros2_ws/src/projeto_bloco/config/params.yaml \
     -p "pasta:=$RAIZ/$SAIDA" > "$SAIDA/analise_bag.log" 2>&1 &
ANALISE=$!
sleep 3                             # o nó assina os tópicos antes de a reprodução começar
ros2 bag play "$BAG" --clock --rate "$TAXA_PLAY" > "$SAIDA/bag_play.log" 2>&1 || true
RESULTADO=0
wait $ANALISE || RESULTADO=$?
if [ -f "$SAIDA/validacao.txt" ]; then sed 's/^/   /' "$SAIDA/validacao.txt"; else tail -15 "$SAIDA/analise_bag.log"; fi
echo "[reproduzir] saídas em $SAIDA:"; ls "$SAIDA" | sed 's/^/   /'
[ "$RESULTADO" = "0" ] || echo "[reproduzir] ATENÇÃO: a validação offline não passou (detalhes acima)"

if [ -z "$SEM_EXTRAS" ]; then
  echo; echo "[reproduzir] ===== 4) itens de percepção veicular do TP4 (MetaDrive + CNN) ====="
  bash scripts/reproduzir_tp4_extras.sh
fi
echo; echo "[reproduzir] para navegar à mão (outro terminal: source ros2_ws/ambiente.sh):"
echo "   ros2 launch projeto_bloco navegacao.launch.py patrulhar:=false surpresa:=false   # e use 2D Goal Pose no RViz"
echo "   ros2 run projeto_bloco patrulha --ros-args --params-file ros2_ws/src/projeto_bloco/config/params.yaml -p use_sim_time:=true"
echo "[reproduzir] concluído"
exit $RESULTADO
