# exemplos/ — código-base de conveniência

Estruturas mínimas com **boas práticas sugeridas** para você usar como base e referência: **copie para `ros2_ws/src/`, renomeie e adapte** à realidade do seu projeto e às suas preferências. Nada aqui é camisa de força — é ponto de partida.

- `pacote_minimo/` — pacote ROS 2 (`ament_python`) completo: publisher parametrizado, subscriber e launch file. Para experimentar:

```bash
cp -r exemplos/pacote_minimo ros2_ws/src/
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch pacote_minimo exemplo.launch.py
```

Mais exemplos (por TP): repositório público de material da disciplina — [PBRoboticos_prof_dacio](https://github.com/Prof-Dacio-INFNET/PBRoboticos_prof_dacio).
