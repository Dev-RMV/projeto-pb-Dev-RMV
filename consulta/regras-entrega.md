# Consulta rápida — Regras de entrega

## 1. Entrega oficial = MOODLE (fonte da verdade)
Até a **sexta da semana de entrega, 23h59**: **PDF do relatório** + **ZIP(s) com os códigos** + **link do repositório** + **link do vídeo**, identificados com nome e código da disciplina. **O que não está no Moodle não foi entregue.** Limite do Moodle: **4 arquivos × 20 MB**. Não coube? Priorize código-fonte, launch/config e evidências nos ZIPs; o excedente (datasets, modelos, vídeos) fica no GitHub/links do `ARTEFATOS.md` — só divida quando necessário.

## 2. Entrega no GitHub = COMPLEMENTAR e obrigatória
Não é apoio dispensável: o professor corrige o **código** no estado da sua entrega e a qualidade do repositório é avaliada. No prazo, faça a **fotografia**: `entrega-tpN` idêntica à `main` + tag `tpN` (ver [git-cheatsheet.md](git-cheatsheet.md)). **Não altere a branch/tag de entrega depois** — vale a data da última alteração.

| TP | Branch/Tag | Prazo (sexta, 23h59) |
|---|---|---|
| TP1 | `entrega-tp1` / `tp1` | 28/08 |
| TP2 | `entrega-tp2` / `tp2` | 25/09 |
| TP3 | `entrega-tp3` / `tp3` | 23/10 |
| TP4 | `entrega-tp4` / `tp4` | **21/11 (sáb) 12h00** — 20/11 é feriado |
| TP5 | `entrega-tp5` / `tp5` | 27/11 |
| Final | `entrega-final` / `final` | 04/12 |

## 3. Vídeos e links — responsabilidade sua
Vídeo comumente no **YouTube** ("público" ou "não listado", nunca "privado") **ou** por **link de drive** (Google Drive/OneDrive com "qualquer pessoa com o link"). **Em qualquer caso, é obrigação do aluno garantir que o link esteja acessível** — teste em aba anônima. Inacessível = inexistente para a correção.

## 4. Rastreabilidade (proteja-se)
Commits e envios no Moodle têm carimbo de data — sua linha do tempo fica provada. Links "vivos" de drive não registram quando o conteúdo entrou; prefira deixar a entrega commitada/anexada até o prazo.

## 5. Confira duas vezes
Arquivo errado, corrompido ou incompleto é responsabilidade do aluno: após o upload, **baixe e abra** o próprio ZIP/PDF.

## Checklist antes de fechar a entrega
1. `dev`→`main` mergeado · 2. `entrega-tpN` == main + tag `tpN` · 3. front-matter do README atualizado (entregue/branch/tag/video/data) · 4. `ARTEFATOS.md` com vídeo + artefatos, links testados em aba anônima · 5. `setup.sh`+`reproduzir.sh` rodam num clone limpo · 6. `docs/decisoes.md` com as mudanças do TP · 7. **Moodle enviado E conferido**
