#!/usr/bin/env bash
# setup.sh — instala TUDO que o seu projeto precisa ALÉM do setup padrão da disciplina.
# (Ubuntu 22.04/WSL2 + ROS 2 Humble + Python 3.10 já instalados — ver tutoriais da disciplina.)
# O professor executa este script num ambiente limpo, antes de reproduzir.sh.
set -e

# 1. Dependências de sistema (exemplos — descomente/adicione conforme SEU projeto)
# sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# 2. Dependências ROS declaradas nos package.xml
# rosdep install --from-paths ros2_ws/src --ignore-src -r -y

# 3. Dependências Python (padrão da disciplina: uv — https://docs.astral.sh/uv/)
# [ -d .venv ] || uv venv --system-site-packages .venv   # venv que enxerga o rclpy do ROS
# . .venv/bin/activate && uv pip install -r requirements.txt   # nunca sudo uv / --system

echo "[setup] concluído"
