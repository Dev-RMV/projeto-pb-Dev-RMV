# Evidências do TP4 (geradas em 23/11/2026, WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + Nav2 1.1)

## Navegação autônoma, behavior tree e bag (pacote `projeto_bloco`)

Tudo vem de **uma execução** de `./scripts/reproduzir.sh` num clone limpo, com as janelas do Gazebo e do RViz abertas e a percepção ligada (detector `alvo_yolov8s_640.pt`, CPU). As capturas `gazebo_*.jpg` são de outra execução igual, sem o RViz na frente.

| Arquivo | O que mostra |
|---|---|
| `reproduzir_saida.txt` | Saída completa do `./scripts/reproduzir.sh`: build, linha do tempo da patrulha (goals, chegadas, disparos, pausa da behavior tree), `ros2 bag info` e a validação offline |
| `reproduzir.log` | Log de todos os nós do `navegacao.launch.py` (Gazebo, Nav2, percepção, árvore, patrulha, gravação) |
| `bag_patrulha/` | **O bag da sessão**: `bag_patrulha_0.db3` + `metadata.yaml` (84 s, 10 668 mensagens, 6,8 MB). Reproduzir: `ros2 bag play docs/evidencias/tp4/bag_patrulha --clock` |
| `bag_info.txt` | Saída de `ros2 bag info` desse bag: 14 tópicos (`/scan`, `/odom`, `/tf`, `/tf_static`, `/cmd_vel`, `/amcl_pose`, `/plan`, `/map`, `/arvore/estado`, `/arvore/distancia_frontal`, `/patrulha/status`, `/patrulha/rota`, `/obstaculo_surpresa/status`, `/arma/disparo`) |
| `patrulha.csv` | Uma linha por posto: duração do trecho, erro de posição e de orientação ao chegar, recuperações do Nav2, disparo e tempo até o disparo |
| `validacao.txt` / `validacao.json` | Resultado do nó `analise_bag` sobre a **reprodução offline** do bag (sem Gazebo nem Nav2): detecção do obstáculo a 0,494 m, robô parado em 0,19 s, 2,14 s parado, estado PAUSADO da árvore por 2,21 s, 5 de 5 postos, erro do AMCL de 6,3 cm em média; `RESULTADO: APROVADO` |
| `trajetoria.png` | **Gráfico da trajetória** gerado do bag: pose real (`/odom` do Gazebo) e estimativa do AMCL sobre o mapa, os cinco postos, o ponto da pausa, a caixa surpresa e o caixote que não está no mapa |
| `pausa_obstaculo.png` | A pausa vista no tempo, também do bag: distância frontal do lidar cruzando 0,5 m, velocidade indo a zero por 2 s e o estado da árvore (NAVEGANDO → PAUSADO → NAVEGANDO) |
| `rviz_1_desvio_caixote.jpg` | RViz (`rviz/navegacao.rviz`) no primeiro trecho: mapa, costmaps global e local, plano do Nav2 (verde) contornando o caixote que só o lidar vê, rota da patrulha (roxo), partículas do AMCL e as imagens da câmera |
| `rviz_2_disparo_posto_alvo_2.jpg` | RViz com o robô parado no posto de tiro do alvo 2, no instante do disparo |
| `rviz_3_pausa_caixa_surpresa.jpg` | RViz durante a pausa: a caixa surpresa enche a câmera e já aparece no costmap local |
| `rviz_4_contornando_a_caixa.jpg` | RViz depois da pausa: plano novo contornando a caixa |
| `gazebo_1_desvio_caixote.jpg` | Gazebo no primeiro trecho: o robô passando pelo caixote amarelo (fora do mapa) |
| `gazebo_2_pausa_caixa_surpresa.jpg` | Gazebo durante a pausa: o robô parado diante da caixa vermelha criada pelo `obstaculo_surpresa` |
| `gazebo_3_contornando_a_caixa.jpg` | Gazebo depois da pausa: o robô contornando a caixa |
| `disparo_posto_alvo_2.jpg`, `disparo_posto_alvo_1.jpg`, `disparo_posto_alvo_3.jpg` | `/torre/visao` no instante de cada disparo (gravado ao chegar a mensagem em `/arma/disparo`): estado DISPARO, erro de mira e ângulos da torre |
| `rastreamento_posto_alvo_1.jpg` | `/vision/rastreamento` no posto do alvo 1: detecção e trilha do alvo pelo `rastreador_yolo` |

Números dessa execução (`patrulha.csv` e `validacao.txt`):

| Posto | Trecho (s) | Erro de posição (m) | Erro de orientação (°) | Disparo |
|---|---|---|---|---|
| posto_alvo_2 | 14,8 | 0,211 | 8,6 | em 1,1 s |
| posto_alvo_1 | 14,1 | 0,255 | −8,6 | em 0,5 s |
| posto_alvo_3 | 11,6 | 0,153 | −7,9 | em 0,7 s |
| vigia_oeste (com a pausa de 2 s e o contorno da caixa) | 17,0 | 0,218 | 5,0 | — |
| base | 10,1 | 0,170 | 8,0 | — |

Total: 9,45 m em 80 s de simulação, velocidade máxima de 0,25 m/s, nenhuma recuperação do Nav2.

## Percepção veicular no MetaDrive e CNN (pacote `tp4_extras`)

As evidências dos itens 3 e 4 (faixas, formas, dataset e CNN com e sem augmentation) estão em [`extras/`](extras/LEIAME.md), com a sua própria tabela de arquivos e a comparação dos treinos.
