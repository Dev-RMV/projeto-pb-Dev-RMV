#!/usr/bin/env bash
# init-branches.sh — cria as branches do projeto (rode UMA vez, após o clone).
# O GitHub Classroom só copia a 'main'; este script cria as demais no seu repo.
set -e
git checkout main 2>/dev/null || git checkout -b main
echo "Criando branches de trabalho e de entrega..."
for b in dev entrega-tp1 entrega-tp2 entrega-tp3 entrega-tp4 entrega-tp5 entrega-final; do
  if git show-ref --verify --quiet "refs/heads/$b"; then
    echo "  - $b (já existe)"
  else
    git branch "$b" main
    echo "  - $b (criada)"
  fi
  git push -u origin "$b" >/dev/null 2>&1 || git push -u origin "$b"
done
git checkout dev
echo ""
echo "Pronto! Você está na branch 'dev' — trabalhe aqui."
echo "NUNCA commite direto nas branches 'entrega-*': elas só recebem a 'main' no momento da entrega."
