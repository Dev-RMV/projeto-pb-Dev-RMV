# MetaDrive · highway-env · Stable Baselines3 · PID — referência (TP4–TP5)

Venv (uv): `uv pip install metadrive-simulator highway-env stable-baselines3 simple-pid scipy`
```python
# MetaDrive (percepção veicular; roda em CPU)
import metadrive, gymnasium as gym
env = gym.make("MetaDrive-validation-v0", config={"use_render": False})
obs,_ = env.reset(); obs,r,term,trunc,info = env.step(env.action_space.sample())
```
```python
# PID (controle clássico)
from simple_pid import PID
pid = PID(1.0, 0.1, 0.05, setpoint=0.0)   # ex.: manter centro da faixa
correcao = pid(erro_lateral)
```
```python
# PPO (aprendizado por reforço) com Stable Baselines3
from stable_baselines3 import PPO
model = PPO("MlpPolicy", "highway-fast-v0", verbose=1)
model.learn(total_timesteps=300_000)      # ~30 min CPU
model.save("ppo_highway")
```
Behavioral Cloning: colete pares (frame → ação) de pilotagem humana e treine uma CNN (ver deep-learning.md). Compare PID × BC × PPO em taxa de colisão, velocidade média e conclusão de rota (TP5).
