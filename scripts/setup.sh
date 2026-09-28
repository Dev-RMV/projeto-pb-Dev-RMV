#!/usr/bin/env bash
# setup.sh — instala TUDO que o seu projeto precisa ALÉM do setup padrão da disciplina.
# (Ubuntu 22.04/WSL2 + ROS 2 Humble + Python 3.10 já instalados — ver tutoriais da disciplina.)
# O professor executa este script num ambiente limpo, antes de reproduzir.sh.
#
# Este projeto: se o ROS 2 Humble NAO estiver instalado, instala também (Ubuntu 22.04 limpo -> tudo).
# Idempotente (pode rodar de novo). Precisa de sudo (ou root) para o apt. reproduzir.sh chama este
# script sozinho quando detecta que falta algo.
# TP2: RViz/URDF/Gazebo Classic pelo apt e o YOLO (ultralytics + torch CPU, versoes fixas em
# requirements.txt) num venv em ros2_ws/.venv — o ROS continua no Python do sistema e o venv entra
# no PYTHONPATH pelo ros2_ws/ambiente.sh.
set -e
cd "$(dirname "$0")/.."

. /etc/os-release
if [ "$VERSION_ID" != "22.04" ]; then
  echo "[setup] este projeto exige Ubuntu 22.04 (ROS 2 Humble). Detectado: $PRETTY_NAME"; exit 1
fi
SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
export DEBIAN_FRONTEND=noninteractive

# 1. Dependências de sistema
echo "[setup] 1/4 pacotes basicos"
$SUDO apt-get update -qq
$SUDO apt-get install -y -qq locales curl gnupg ca-certificates lsb-release software-properties-common >/dev/null
$SUDO locale-gen en_US.UTF-8 >/dev/null
$SUDO add-apt-repository -y universe >/dev/null 2>&1

echo "[setup] 2/4 repositorio apt do ROS 2 (se ainda nao houver)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1 && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  VER=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -oP '"tag_name": "\K[^"]+' || true)
  if [ -n "$VER" ] && curl -fsSL -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${VERSION_CODENAME}_all.deb"; then
    $SUDO apt-get install -y -qq /tmp/ros2-apt-source.deb >/dev/null      # metodo oficial atual
  else                                                                     # fallback: chave manual
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
      | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  $SUDO apt-get update -qq
fi

# 2. Dependências ROS + OpenCV (as mesmas declaradas nos package.xml)
echo "[setup] 3/4 ROS 2 Humble (ros-base), cv_bridge, OpenCV, colcon, g++; RViz, URDF (xacro, check_urdf),"
echo "              robot/joint_state_publisher, tf2_tools, Gazebo Classic 11 (alguns minutos na 1a vez)"
$SUDO apt-get install -y -qq ros-humble-ros-base ros-humble-cv-bridge build-essential \
                             python3-colcon-common-extensions python3-opencv python3-numpy opencv-data \
                             python3-venv ros-humble-rviz2 ros-humble-xacro ros-humble-urdfdom \
                             ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                             ros-humble-joint-state-publisher-gui ros-humble-tf2-tools \
                             ros-humble-gazebo-ros-pkgs >/dev/null

# 3. Dependências Python do YOLO: venv ros2_ws/.venv (sem sudo; ~1,4 GB com o torch de CPU)
echo "[setup] 4/4 venv ros2_ws/.venv: ultralytics + torch CPU (requirements.txt)"
VENV=ros2_ws/.venv
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
touch "$VENV/COLCON_IGNORE"                  # o colcon nao varre o venv
"$VENV/bin/pip" install -q -U pip
"$VENV/bin/pip" install -q -r requirements.txt

echo "[setup] concluído"
