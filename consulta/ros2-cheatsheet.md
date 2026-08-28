# Consulta rápida — ROS 2 Humble

## Workspace
```bash
cd ros2_ws && colcon build                 # compilar tudo
colcon build --packages-select meu_pacote  # compilar um pacote
source install/setup.bash                  # SEMPRE após compilar (por terminal)
ros2 pkg create --build-type ament_python meu_pacote
```

## Executar e inspecionar
```bash
ros2 run <pacote> <no>
ros2 launch <pacote> bringup.launch.py
ros2 node list / ros2 node info /meu_no
ros2 topic list / ros2 topic echo /camera/image_raw / ros2 topic hz /topico
ros2 service list / ros2 service call /vision/status <tipo> "{}"
ros2 param list / ros2 param set /meu_no limiar 0.7
ros2 interface list | grep meu_pacote      # conferir .msg/.srv/.action compilados
ros2 doctor                                # diagnóstico do ambiente
```

## Visualizar e gravar
```bash
rqt_graph                                  # grafo de nós/tópicos (capture p/ o relatório!)
rviz2
ros2 run tf2_tools view_frames             # TF tree em PDF
ros2 bag record -a  /  ros2 bag play <pasta>
```
