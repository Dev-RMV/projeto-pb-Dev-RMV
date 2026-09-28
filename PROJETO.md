# Proposta do Projeto — Robô Sniper

> _A proposta é fazer um robô que detecta "atira" em alvos (do estilo de competições de tiro).

## Objetivo e justificativa
_O objetivo é que o robô consiga navegar, detectar e "atirar" em alvos._

## Aplicação prática
_Cenário de conflito armado e criação de memes._

## Funcionalidades previstas
_Navegação com LIDAR e esteira do tipo "tanque", percepção com câmera e "atirar" um feixe de luz na mosca de um alvo detectado._

## Sensores e hardware (real ou simulado)
_Câmera, lidar, emissor de luz numa torre pan/tilt (2 servos), Raspberry Pi, bateria, esteira tipo "tanque" (2 motores). Modelo do robô: URDF em `ros2_ws/src/projeto_bloco_description` (câmera, lidar e esteiras simulados no Gazebo)._

## Arquitetura prevista (ROS 2)
_Nó cameraPub que publica dados de imagens, nó radar para o lidar que publica o que for mapeado, 2 nós motor que publicam dados dos motores e assinam um nó de controleMotor, nó processaImagem que assina de cameraPub, entre outros._

**Implementado até o TP2:** `camera_publisher` → `rastreador_yolo` (YOLOv8s treinado para detectar **alvos de tiro** + ByteTrack + filtro de Kalman, publica `DeteccaoArray`) → `servidor_engajar` (action `EngajarAlvo`: mira a torre e dispara o emissor; serviço `ControleArma` como trava de segurança) ← `cliente_engajar`; a torre publica os estados das juntas para o `robot_state_publisher` (URDF/TF no RViz). Parâmetros de todos os nós em `config/params.yaml`. Detalhes: [`ros2_ws/src/projeto_bloco/README.md`](ros2_ws/src/projeto_bloco/README.md).

## Plano por TP
| TP | O que será implementado neste projeto |
|---|---|
| TP1 |Similação de detecção de rostos e objetos - sem detecção de alvo ainda|
| TP2 |Interfaces próprias (msg/srv/action), detecção e rastreamento de alvos de tiro com YOLOv8s treinado (dataset sintético) + filtro de Kalman, action de engajamento do alvo (mira da torre + disparo) com cliente, parâmetros em YAML alteráveis em execução, URDF do robô (esteiras, câmera, lidar, torre)|
| TP3 |Não sei - cedo demais para dizer|
| TP4 |Não sei - cedo demais para dizer|
| TP5 |Não sei - cedo demais para dizer|

## Riscos e alternativas
_Existe risco REAL E ALTO de complicações na parte mecânica, elétrica/eletrônica e de alimentação, além da falta de compreensão de como o hardware envolvido nesses três pilares funciona. Caso essas dificuldades não sejam superadas, o robô será simulado_

---

> **Evolução do projeto:** este documento reflete o plano **atual**. Não apague o passado ao atualizá-lo: cada mudança relevante de escopo, sensor, arquitetura ou abordagem deve ter uma entrada em [`docs/decisoes.md`](docs/decisoes.md) — o que mudou, onde, por quê e o impacto. Rastreabilidade conta a favor na avaliação.
