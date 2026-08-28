# SLAM Toolbox · Nav2 — referência (TP3–TP4)

## SLAM (mapear enquanto navega)
```bash
sudo apt install ros-humble-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py     # constrói o mapa; veja no RViz2
ros2 run tf2_tools view_frames                       # confira map→odom→base_link
```

## Nav2 (navegação autônoma 2D)
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2.yaml
```
Peças: **AMCL** (localização no mapa), **costmaps** local/global, **planner** + **controller**, **behavior tree** (recuperação). Enviar metas: RViz2 "Nav2 Goal" ou um nó Python publicando `nav2_msgs/action/NavigateToPose`. Gravar sessão: `ros2 bag record -a` / reproduzir: `ros2 bag play <pasta>`.
