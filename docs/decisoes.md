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

## Exemplo (apague quando tiver a primeira entrada real)

## 2026-09-02 · TP2 — Migração do rastreamento para YOLO
- **O que mudou:** rastreamento por cor substituído por detecção YOLO + filtro de suavização.
- **Onde:** `ros2_ws/src/visao_pipeline/`.
- **Por quê:** a segmentação HSV perdia o objeto com variação de iluminação do laboratório (testes da semana 5).
- **Impacto:** nó `rastreador_cor` descontinuado; parâmetros novos em `config/visao.yaml`; plano do TP3 inalterado.
