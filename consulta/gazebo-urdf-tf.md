# Gazebo · RViz2 · URDF · TF — referência (TP2–TP3)

## URDF (modelo do robô)
Estrutura: `<link>` (corpos) + `<joint>` (articulações, type: fixed/continuous/revolute) + `<visual>/<collision>/<inertial>`. Sensores via `<gazebo>`/plugins. Verificar: `check_urdf robo.urdf`; visualizar: `ros2 launch urdf_tutorial display.launch.py model:=robo.urdf`.

## TF (transformadas)
```bash
ros2 run tf2_tools view_frames        # gera frames.pdf da árvore (map→odom→base_link→sensores)
ros2 run tf2_ros tf2_echo base_link laser   # transformada entre 2 frames
```
`robot_state_publisher` publica as TFs a partir do URDF + estados das juntas.

## Gazebo Harmonic + RViz2
```bash
ros2 launch ros_gz_sim gz_sim.launch.py            # simulador
ros2 run robot_state_publisher robot_state_publisher robo.urdf
rviz2                                              # adicione RobotModel, TF, LaserScan, Image...
```
Orquestre tudo num `bringup.launch.py` (Gazebo + RSP + RViz2 + seus nós), parâmetros via YAML.
