# Consulta rápida — git na disciplina (fluxo de branches)

## Setup (uma vez, após clonar)
```bash
./scripts/init-branches.sh        # cria dev + entrega-tp1..final; te deixa em 'dev'
```

## O ciclo do dia a dia (trabalhe em 'dev')
```bash
git checkout dev
git pull                          # AO CHEGAR (lab ou casa)
git add . && git commit -m "O que a mudança faz"
git push                          # AO SAIR (nada fica só no lab!)
```
Quer isolar uma experiência? `git checkout -b feat/minha-ideia` (a partir de dev) — livre.

## Publicar um marco estável na main
Quando 'dev' está estável e compila:
```bash
git checkout main && git merge dev && git push
git checkout dev                  # volte a trabalhar
```
A **main** guarda sempre o estado atual e estável do projeto (ela evolui TP a TP).

## 📸 Entregar um TP (a fotografia da entrega)
No prazo (sexta 23h59), com a 'main' já contendo tudo do TP:
```bash
git checkout main && git merge dev && git push          # 1) main completa
git checkout entrega-tpN && git merge --ff-only main && git push   # 2) branch = main
git tag tpN && git push origin tpN                       # 3) tag imutável (tp1..tp5, final)
git checkout dev                                         # 4) volte a trabalhar
```
> ⚠️ **Depois de entregar, NÃO altere a branch `entrega-tpN` nem a tag `tpN`.** Elas são a fotografia da entrega — **vale a data da última alteração**. Mexer nelas depois do prazo conta como entrega fora do prazo. Continue evoluindo o projeto na `dev`/`main`.

## Ver/entender
```bash
git branch -a                     # todas as branches
git status · git log --oneline -10
git checkout entrega-tp1          # ver a entrega como ficou (voltar: git checkout dev)
git switch -                      # alterna para a branch anterior
```

## Socorro
```bash
git restore ARQUIVO               # descartar alteração não commitada
git pull --rebase                 # "rejected: fetch first" ao dar push
gh auth login                     # "Permission denied" nesta máquina
git merge --ff-only main          # se falhar em entrega-tpN: você commitou nela por engano — chame o professor
```

## Boas mensagens de commit
"Implementa serviço /vision/status" ✅ · "Corrige limiar HSV" ✅ · "mudanças" ❌ · "final2 agora vai" ❌
