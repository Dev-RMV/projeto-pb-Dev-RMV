# Registro de decisões e mudanças do projeto

> Projetos reais mudam — escopo, sensores, arquitetura, abordagem. O que a disciplina espera é o que se espera de um engenheiro: **rastreabilidade**. Registre aqui cada mudança relevante em relação ao planejado (o `PROJETO.md` sempre reflete o plano ATUAL; o histórico vive aqui). Refatorar com registro é maturidade; mudar silenciosamente parece improviso — e isso pesa na avaliação.

Formato de cada entrada (mais recente no topo):

---

## AAAA-MM-DD · TPn — _título curto da mudança_

- **O que mudou:** _(ex.: troquei detecção por cor HSV por YOLO no módulo de percepção)_
- **Onde:** _(pacote/arquivo/módulo afetado)_
- **Por quê:** _(motivo técnico: limitação encontrada, resultado de teste, feedback do professor…)_
- **Impacto:** _(o que foi refeito, o que ficou obsoleto, efeito no plano dos próximos TPs)_

---

## 2026-09-28 · TP2 — Detector de alvos de tiro treinado (YOLOv8s) no lugar do YOLO do COCO

- **O que mudou:** o `rastreador_yolo` passou a usar um YOLOv8s com fine-tuning para uma única classe, `alvo` (o alvo de tiro impresso do projeto, com anéis vermelhos e brancos), no lugar do YOLO11n pré-treinado no COCO (`sports ball`/`person`). A demo do TP2 agora roda um vídeo do robô num campo de tiro (`alvos_dv20.mp4`) em vez do `demo.mp4`. A action, o filtro de Kalman e os parâmetros continuam os mesmos; mudaram os valores: `classes: ["alvo"]`, `confianca` 0,5, `hfov_graus` 70 e `kalman.sigma_a` 200.
- **Onde:** `projeto_bloco/modelos/alvo_yolov8s_640.pt` (instalado em `share/projeto_bloco/modelos`), `ros2_ws/midia/alvos_dv20.mp4`, `config/params.yaml`, `params_economia.yaml`, `rastreador_yolo.py` (procura o modelo no pacote antes da pasta de pesos oficiais), `scripts/reproduzir.sh`, `scripts/demo_parametros.sh`, câmera do Gazebo (70°).
- **Por quê:** o robô atira em alvos, e nenhuma classe do COCO representa um alvo de tiro. O modelo e o vídeo vêm de um projeto separado (`C:\_fotos`), que gera um dataset sintético no Blender (2000 imagens de treino e 400 de validação, com degradação que imita a câmera DV20 do robô) e treinou o YOLOv8s (escolhido por rodar a 491 fps no Hailo-8 do Raspberry Pi 5 de destino). Na validação sintética: mAP50 0,951 e mAP50-95 0,834. No vídeo, o nó detecta os 5 alvos com confiança ≥ 0,72, a ~54 ms por quadro na CPU. O Kalman foi recalibrado nas trilhas desse vídeo, em que a câmera vibra e gira: com `sigma_a` 200 e `sigma_z` 15 a trilha fica 72% mais lisa, com ~5 px de atraso. O campo de visão de 70° é o da câmera do vídeo (a DV20 ainda não teve o FOV medido).
- **Impacto:** modelo (22,5 MB) e vídeo (9,6 MB) entram no repositório, como o `demo.mp4`. O YOLO11n do COCO fica só na demo de troca de modelo em execução. O TP1 continua com o `demo.mp4`. Todas as métricas do modelo são em dados sintéticos: falta testar com o alvo impresso e a câmera real.

## 2026-09-28 · TP2 — Ajustes no que veio do TP1

- **O que mudou:** o `camera_publisher` ganhou o parâmetro `frame_id` (padrão `camera_optical_link`, o frame óptico do URDF; antes era fixo `camera`). No `params.yaml` os nós passaram a ser escritos como `/nome:`. O `reproduzir.sh` agora roda a demo do TP2; a do TP1 continua em `ros2 launch projeto_bloco projeto_bloco.launch.py`.
- **Onde:** `projeto_bloco/camera_publisher.py`, `config/params.yaml`, `scripts/reproduzir.sh`, `scripts/setup.sh`.
- **Por quê:** a imagem precisa estar num frame que exista na árvore de TF. O `ros2 param load` do Humble recusa o arquivo quando o nó está sem a barra (“Param file does not contain parameters for /rastreador_yolo”); o launch aceita as duas formas.
- **Impacto:** nenhum nó do TP1 mudou de comportamento. O `setup.sh` instala mais pacotes (RViz, xacro, Gazebo) e o venv do YOLO.

## 2026-09-28 · TP2 — URDF com Gazebo Classic 11 e atrito anisotrópico nas rodas

- **O que mudou:** novo pacote `projeto_bloco_description` com o URDF (xacro) do robô: chassi, duas esteiras (roda motriz e roda guia por lado), câmera com frame óptico, lidar num mastro e a torre pan/tilt com o emissor. Os sensores são simulados com os plugins do Gazebo Classic 11 (`gazebo_ros`): câmera em `/camera/image_raw`, lidar em `/scan` e as esteiras como diff drive de 2 pares de rodas (`/cmd_vel` → `/odom`).
- **Onde:** `ros2_ws/src/projeto_bloco_description/`.
- **Por quê:** o Gazebo Classic 11 é o que está instalado e integrado ao Humble nesta máquina. O cheatsheet da disciplina cita o Gazebo Harmonic (`ros_gz`); a migração fica para quando a simulação virar o foco. Com atrito igual em todas as direções, o robô não girava no lugar (1° de yaw com 1 rad/s por 3 s): o entre-eixos (0,20 m) é maior que a bitola (0,19 m) e o atrito lateral das 4 rodas trava o giro. Com atrito lateral 0,2 (`mu1` + `fdir1` no eixo da roda), o giro chega a 158° dos 172° ideais, parecido com uma lagarta, que escorrega de lado.
- **Impacto:** a câmera simulada publica no mesmo tópico do `camera_publisher`, então o rastreador roda igual sobre a simulação. A torre ainda não é acionada no Gazebo (fica para `ros2_control`).

## 2026-09-28 · TP2 — Torre pan/tilt e action de engajamento como tarefa central

- **O que mudou:** o robô ganhou uma torre pan/tilt (2 servos) que aponta o emissor de luz; antes a mira seria só girando o robô na esteira. A tarefa central do projeto virou a action `EngajarAlvo`: procurar o alvo, mirar, travar e disparar. O feedback traz estado, progresso e erro; o result traz métricas. O serviço `ControleArma` funciona como trava de segurança: desarmada, a action rejeita goals, e desarmar aborta o goal em curso.
- **Onde:** `projeto_bloco_interfaces` (`DeteccaoObj`, `DeteccaoArray`, `ControleArma`, `EngajarAlvo`), `projeto_bloco/servidor_engajar.py`, `projeto_bloco/cliente_engajar.py`, URDF.
- **Por quê:** separar mira (torre) de locomoção (esteira) deixa o robô atirar parado ou andando. No controle, um P puro fica sempre atrasado de v/kp em alvo móvel (erro médio de 25 px, sem disparo em 20 s na bola). Com feedforward da velocidade estimada pelo Kalman, todos os goals de teste dispararam. A predição foi limitada a 0,25 s e só dispara com o alvo visível no frame, porque extrapolar o quique da bola gerava “trava” num ponto vazio. O ByteTrack troca o id da bola com frequência (25 ids em 80 s), então um id novo perto do alvo anterior herda a trava.
- **Impacto:** a lista de hardware passa a incluir a torre (ver `PROJETO.md`). Por enquanto a torre é simulada: publica `/torre/joint_states` para o URDF no RViz.

## 2026-09-28 · TP2 — Rastreamento com YOLO11n + ByteTrack + filtro de Kalman, com o YOLO num venv

- **O que mudou:** percepção nova no `rastreador_yolo`: YOLO11n pré-treinado (COCO, classes `sports ball` e `person`, configuráveis), ids do ByteTrack e filtro de Kalman 2D de velocidade constante por id. A segmentação HSV e os rostos do TP1 continuam no pacote. O ultralytics e o torch (CPU) ficam num venv `ros2_ws/.venv` (`requirements.txt`, versões fixas); o ROS continua no Python do sistema.
- **Onde:** `projeto_bloco/rastreador_yolo.py`, `projeto_bloco/filtro_kalman.py`, `config/params.yaml`, `requirements.txt`, `ros2_ws/ambiente.sh`, `scripts/setup.sh`.
- **Por quê:** no `demo.mp4`, o `yolo11n` detecta a bola (cerca de 20 ms por frame na CPU) e o `yolo26n` não a detectou nenhuma vez. O Kalman foi calibrado nas trilhas reais do vídeo: com `sigma_a` 400 e `sigma_z` 6 a trilha quase não suavizava (-6%); com 100 e 15, a rugosidade cai 28% na bola e 42% nas pessoas, com 4 a 6 px de atraso. O venv evita o numpy 2 (o `cv_bridge` do Humble é compilado com numpy 1.x) e não mexe no Python dos outros workspaces; por isso numpy fica em 1.26 e opencv-python em 4.11. O torch roda com 4 threads (`threads_cpu`) para não disputar CPU com os outros nós.
- **Impacto:** o `setup.sh` baixa cerca de 1,4 GB (torch de CPU). Os pesos (5,4 MB) são baixados do release público da Ultralytics (ver `ARTEFATOS.md`). Um detector de alvo de tiro treinado por nós (classe própria) fica para um TP de treinamento.
