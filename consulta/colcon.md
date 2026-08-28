# ⭐ Colcon — workspace e build

```bash
cd ~/pb-SEU-USUARIO/ros2_ws        # sempre compile na RAIZ do workspace
colcon build                        # compila tudo
colcon build --packages-select PKG  # compila 1 pacote (mais rápido)
colcon build --symlink-install      # edita Python sem recompilar
source install/setup.bash           # SEMPRE após compilar (por terminal)
rosdep install --from-paths src --ignore-src -r -y   # instala dependências declaradas
```
Criar pacote: `ros2 pkg create --build-type ament_python meu_pacote`
Limpeza: `rm -rf build install log` e recompile. Nada de `build/ install/ log/` no git.
