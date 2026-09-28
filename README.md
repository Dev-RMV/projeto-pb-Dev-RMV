---
# ═══════════════════════════════════════════════════════════════════════
# METADADOS DO PROJETO — preencha os campos entre aspas. NÃO renomeie chaves.
# Este bloco é lido automaticamente na correção/acompanhamento (YAML válido!).
# ═══════════════════════════════════════════════════════════════════════
aluno: "Rodrigo das Mercês Vianna"
github: "Dev-RMV"
disciplina: "PB Sistemas Robóticos 2026.2"
turma: "GRPEDCR3C1-M1-P1"
projeto: "Robô Sniper"
entregas:
  tp1:   { entregue: false, branch: "entrega-tp1",   tag: "tp1",   video: "https://www.youtube.com/watch?v=WrpPA5194_8", data: "" }
  tp2:   { entregue: false, branch: "entrega-tp2",   tag: "tp2",   video: "https://youtu.be/VQyR84piW8Q", data: "" }
  tp3:   { entregue: false, branch: "entrega-tp3",   tag: "tp3",   video: "", data: "" }
  tp4:   { entregue: false, branch: "entrega-tp4",   tag: "tp4",   video: "", data: "" }
  tp5:   { entregue: false, branch: "entrega-tp5",   tag: "tp5",   video: "", data: "" }
  final: { entregue: false, branch: "entrega-final", tag: "final", video: "", data: "" }
---
# Projeto de Bloco: Sistemas Robóticos — <!-- PB:ALUNO -->Rodrigo das Mercês Vianna<!-- /PB:ALUNO -->

> ⚠️ **Entrega oficial = MOODLE** (ZIP de códigos + PDF + links). **A entrega no GitHub é COMPLEMENTAR e obrigatória** — não é opcional nem mero apoio: o professor corrige o código no estado da sua **branch/tag de entrega**, e a qualidade do repositório é critério de avaliação. Moodle **e** GitHub, sempre os dois. Repositório criado pelo GitHub Classroom.

## Identificação
- **Nome:** <!-- PB:ALUNO --> _Rodrigo das Mercês Vianna_
- **Usuário GitHub:** · **Disciplina:** PB Sistemas Robóticos (GRPEDCR3C1-M1-P1)

## Sobre o projeto
_Robô Sniper: O projeto pretende criar um robô que simula um tanque que atira em alvos (estilo os de competições), simulando um cenário de conflito. Para isso, o controle será feito em um grafo ROS2, usando sensores como lidar e câmera, e atuadores como motores e um emissor de luz, que irá acertar a mosca do alvo. Proposta e planejamento completos: [PROJETO.md](PROJETO.md)._

## Como compilar e executar (reprodutibilidade!)
O professor corrige **executando** num clone limpo — mantenha isto funcionando a cada TP:
```bash
./scripts/setup.sh        # dependências além do setup padrão da disciplina
./scripts/reproduzir.sh   # compila, obtém/gera artefatos e roda a demo do TP corrente
```
Demo atual (TP2): detecção de alvos de tiro (YOLOv8s treinado) com rastreamento e filtro de Kalman, action de engajamento (servidor + cliente), parâmetros dinâmicos e URDF no RViz. Comandos manuais e detalhes: [`ros2_ws/src/projeto_bloco/README.md`](ros2_ws/src/projeto_bloco/README.md).

## Fluxo de branches (leia o [consulta/git.md](consulta/git.md))
- **`main`** — estado atual e estável do projeto (evolui TP a TP; sempre compilável).
- **`dev`** — onde você trabalha no dia a dia (crie feature branches à vontade a partir dela).
- **`entrega-tpN`** — a **fotografia** de cada entrega: no momento da entrega ela fica **idêntica à `main`** e **não deve ser mais alterada** (vale a data da última alteração). Uma tag `tpN` marca o mesmo commit.
> Rode `./scripts/init-branches.sh` no 1º dia para criar as branches. **Nunca commite direto nas `entrega-*`** — elas só recebem a `main` no momento da entrega.

## Status por TP
| TP | Branch | Tag | Entregue no Moodle | Vídeo (ver ARTEFATOS.md) |
|---|---|---|---|---|
| TP1 | `entrega-tp1` | `tp1` | ⬜ | ⬜ |
| TP2 | `entrega-tp2` | `tp2` | ⬜ | ⬜ |
| TP3 | `entrega-tp3` | `tp3` | ⬜ | ⬜ |
| TP4 | `entrega-tp4` | `tp4` | ⬜ | ⬜ |
| TP5 | `entrega-tp5` | `tp5` | ⬜ | ⬜ |
| Final | `entrega-final` | `final` | ⬜ | ⬜ |

## Estrutura (não desmonte — é avaliada)
`scripts/` (setup, reproduzir, init-branches) · [`ARTEFATOS.md`](ARTEFATOS.md) (links de vídeos/artefatos) · `ros2_ws/src/` (pacotes) · `docs/` (relatórios, evidências, diário, decisões) · `media/` · `consulta/` (cheatsheets) · `exemplos/` (código-base para adaptar).

## Referências da disciplina
- Material, tutoriais, exemplos e cheatsheets vivos: [PBRoboticos_prof_dacio](https://github.com/Prof-Dacio-INFNET/PBRoboticos_prof_dacio)
- Regras e prazos oficiais: **Moodle** (resumo em [consulta/regras-entrega.md](consulta/regras-entrega.md))
