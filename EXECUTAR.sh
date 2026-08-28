#!/usr/bin/env bash
# EXECUTAR.sh — atalho: compila, instala o que faltar e roda a demo (= ./scripts/reproduzir.sh).
# Uso: ./EXECUTAR.sh   |   ./EXECUTAR.sh source:=0   |   Ctrl+C encerra.
exec "$(dirname "$0")/scripts/reproduzir.sh" "$@"
