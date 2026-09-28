# Ambiente do workspace para terminais manuais:  source ros2_ws/ambiente.sh
#   ROS 2 Humble + este workspace (se compilado) + venv do YOLO (ros2_ws/.venv, criado pelo
#   scripts/setup.sh) no PYTHONPATH + perfil Fast-DDS para imagens grandes.
# O venv vem ANTES no PYTHONPATH: numpy 1.26 do venv (ultralytics/torch) no lugar do 1.21 do
# sistema; continua < 2, que e o que o cv_bridge do Humble aceita.
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_WS/install/setup.bash" ] && source "$_WS/install/setup.bash"
if [ -d "$_WS/.venv/lib/python3.10/site-packages" ]; then
  case ":$PYTHONPATH:" in
    *":$_WS/.venv/lib/python3.10/site-packages:"*) ;;
    *) export PYTHONPATH="$_WS/.venv/lib/python3.10/site-packages:$PYTHONPATH" ;;
  esac
fi
_PERFIL="$_WS/src/projeto_bloco/config/fastdds_profile.xml"
[ -f "$_PERFIL" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$_PERFIL"
unset _WS _PERFIL
