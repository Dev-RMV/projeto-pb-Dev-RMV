# Proposta do Projeto — Robô Sniper

> _A proposta é fazer um robô que detecta "atira" em alvos (do estilo de competições de tiro).

## Objetivo e justificativa
_O objetivo é que o robô consiga navegar, detectar e "atirar" em alvos._

## Aplicação prática
_Cenário de conflito armado e criação de memes._

## Funcionalidades previstas
_Navegação com LIDAR e esteira do tipo "tanque", percepção com câmera e "atirar" um feixe de luz na mosca de um alvo detectado._

## Sensores e hardware (real ou simulado)
_Câmera USB DV20 (640×480 MJPG a 30 fps), lidar 2D (tipo RPLIDAR A1), emissor de luz numa torre pan/tilt (2 servos), **Arduino UNO Q** (Qualcomm QRB2210, 4 núcleos Cortex-A53, 4 GB, Debian 13 com ROS 2 Humble em Docker; sem acelerador de IA), bateria, esteira tipo "tanque" (2 motores). Até 22/10/2026 o computador previsto era um Raspberry Pi 5 com Hailo-8 (ver `docs/decisoes.md`); a troca obrigou um detector mais leve (YOLOv8n em 416 px, perfil `params_uno_q.yaml`). Modelo do robô: URDF em `ros2_ws/src/projeto_bloco_description` (câmera, lidar e esteiras simulados no Gazebo Classic 11; as dimensões ainda são provisórias, à espera do chassi real)._

## Arquitetura prevista (ROS 2)
_Nó cameraPub que publica dados de imagens, nó radar para o lidar que publica o que for mapeado, 2 nós motor que publicam dados dos motores e assinam um nó de controleMotor, nó processaImagem que assina de cameraPub, entre outros._

**Implementado até o TP3:** `bringup.launch.py` sobe com um comando o Gazebo Classic 11 (mundo `sala_alvos.world`, com obstáculos e alvos impressos com a arte real), o URDF no `robot_state_publisher`, o SLAM Toolbox (`/scan` → `/map`, TF `map → odom → base_footprint`), os nós do TP2 e o novo `segmentador_yolo` (YOLOv8n-seg treinado para `alvo`, máscaras com transparência 0,5 em `/vision/segmented`), o RViz2 com configuração salva e um `piloto_demo` que percorre uma rota fixa para o mapa ser construído. Parâmetros em `config/params.yaml`, `bringup.yaml` e `slam.yaml`.

**Implementado até o TP2:** `camera_publisher` → `rastreador_yolo` (YOLOv8s treinado para detectar **alvos de tiro** + ByteTrack + filtro de Kalman, publica `DeteccaoArray`) → `servidor_engajar` (action `EngajarAlvo`: mira a torre e dispara o emissor; serviço `ControleArma` como trava de segurança) ← `cliente_engajar`; a torre publica os estados das juntas para o `robot_state_publisher` (URDF/TF no RViz). Parâmetros de todos os nós em `config/params.yaml`. Detalhes: [`ros2_ws/src/projeto_bloco/README.md`](ros2_ws/src/projeto_bloco/README.md).

## Plano por TP
| TP | O que será implementado neste projeto |
|---|---|
| TP1 |Similação de detecção de rostos e objetos - sem detecção de alvo ainda|
| TP2 |Interfaces próprias (msg/srv/action), detecção e rastreamento de alvos de tiro com YOLOv8s treinado (dataset sintético) + filtro de Kalman, action de engajamento do alvo (mira da torre + disparo) com cliente, parâmetros em YAML alteráveis em execução, URDF do robô (esteiras, câmera, lidar, torre)|
| TP3 |Launch único (`bringup.launch.py`: Gazebo + URDF + SLAM Toolbox + percepção + RViz, parâmetros em YAML), mapa 2D com o lidar e árvore de TF `map → odom → base_footprint → sensores`, segmentação por instância dos alvos (YOLOv8n-seg treinado no dataset sintético com polígonos) publicada em `/vision/segmented`, análise de arquitetura de percepção veicular (CAN/Ethernet) e da integração dos sensores do robô (câmera USB, lidar) no ROS 2|
| TP4 |Previsto: navegação autônoma com Nav2 sobre o mapa do TP3 (localização, planejamento e desvio de obstáculos) e controle da torre no Gazebo|
| TP5 |Não sei - cedo demais para dizer|

## Riscos e alternativas
_Existe risco REAL E ALTO de complicações na parte mecânica, elétrica/eletrônica e de alimentação, além da falta de compreensão de como o hardware envolvido nesses três pilares funciona. Caso essas dificuldades não sejam superadas, o robô será simulado. Risco novo (TP3): o Arduino UNO Q não tem acelerador de IA, então o detector roda a 1-2 Hz na placa; a mira contínua depende do filtro de Kalman e de um rastreador barato entre inferências. Se não bastar, a alternativa é voltar a um computador com acelerador (Raspberry Pi 5 + Hailo-8)._

---

> **Evolução do projeto:** este documento reflete o plano **atual**. Não apague o passado ao atualizá-lo: cada mudança relevante de escopo, sensor, arquitetura ou abordagem deve ter uma entrada em [`docs/decisoes.md`](docs/decisoes.md) — o que mudou, onde, por quê e o impacto. Rastreabilidade conta a favor na avaliação.
