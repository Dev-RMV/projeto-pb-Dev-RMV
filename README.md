---
# ═══════════════════════════════════════════════════════════════════════
# METADADOS DO PROJETO — preencha os campos entre aspas. NÃO renomeie chaves.
# Este bloco é lido automaticamente na correção/acompanhamento (YAML válido!).
# ═══════════════════════════════════════════════════════════════════════
aluno: "SEU NOME COMPLETO"
github: "seu-usuario"
disciplina: "PB Sistemas Robóticos 2026.2"
turma: "GRPEDCR3C1-M1-P1"
projeto: "Título do seu projeto"
entregas:
  tp1:   { entregue: false, branch: "entrega-tp1",   tag: "tp1",   video: "", data: "" }
  tp2:   { entregue: false, branch: "entrega-tp2",   tag: "tp2",   video: "", data: "" }
  tp3:   { entregue: false, branch: "entrega-tp3",   tag: "tp3",   video: "", data: "" }
  tp4:   { entregue: false, branch: "entrega-tp4",   tag: "tp4",   video: "", data: "" }
  tp5:   { entregue: false, branch: "entrega-tp5",   tag: "tp5",   video: "", data: "" }
  final: { entregue: false, branch: "entrega-final", tag: "final", video: "", data: "" }
---
# Projeto de Bloco: Sistemas Robóticos — <!-- PB:ALUNO -->Seu Nome Aqui<!-- /PB:ALUNO -->

> ⚠️ **Entrega oficial = MOODLE** (ZIP de códigos + PDF + links). **A entrega no GitHub é COMPLEMENTAR e obrigatória** — não é opcional nem mero apoio: o professor corrige o código no estado da sua **branch/tag de entrega**, e a qualidade do repositório é critério de avaliação. Moodle **e** GitHub, sempre os dois. Repositório criado pelo GitHub Classroom.

## Identificação
- **Nome:** <!-- PB:ALUNO --> _(preencha também no front-matter acima)_
- **Usuário GitHub:** · **Disciplina:** PB Sistemas Robóticos (GRPEDCR3C1-M1-P1)

## Sobre o projeto
_Título e descrição curta (2–3 frases). Proposta e planejamento completos: [PROJETO.md](PROJETO.md)._

## Como compilar e executar (reprodutibilidade!)
O professor corrige **executando** num clone limpo — mantenha isto funcionando a cada TP:
```bash
./scripts/setup.sh        # dependências além do setup padrão da disciplina
./scripts/reproduzir.sh   # compila, obtém/gera artefatos e roda a demo do TP corrente
```

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
