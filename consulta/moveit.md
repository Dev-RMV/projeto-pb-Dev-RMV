# MoveIt 2 — manipulação 3D (TP5)

```bash
sudo apt install ros-humble-moveit
ros2 launch moveit_setup_assistant setup_assistant.launch.py   # gerar config do manipulador
ros2 launch SEU_moveit_config demo.launch.py                    # RViz2 + planejamento
```
Conceitos: **planning group**, **cinemática inversa (IK)**, **planejamento de trajetória**, **Collision Object** (evitar colisão), execução com **ros2_control**. Pela API Python (`moveit_py`) ou C++ (`MoveGroupInterface`): definir pose destino → `plan()` → `execute()`. Demonstre ≥3 poses com desvio de obstáculos (pedido do TP5).
