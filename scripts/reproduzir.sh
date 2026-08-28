#!/usr/bin/env bash
# reproduzir.sh — reproduz o seu trabalho do zero. O PROFESSOR CORRIGE EXECUTANDO ISTO.
#
# Contrato: executado na raiz do repositório recém-clonado (após ./scripts/setup.sh), deve:
#   1) compilar; 2) obter/gerar todos os artefatos necessários; 3) lançar a demo do TP corrente.
# Todo artefato derivado (modelo, dataset processado, mapa) precisa ter FONTE: ou é gerado
# aqui pelo script que o produz, ou é baixado do link PÚBLICO registrado em ARTEFATOS.md —
# nesse caso, deixe documentado ao lado o comando que o gerou.
set -e

# ---------- 1. COMPILAR ----------
cd ros2_ws
colcon build
source install/setup.bash
cd ..

# ---------- 2. OBTER INSUMOS ----------
# Opção A — gerar (preferida):
#   python3 scripts/gerar_dataset.py --saida datasets/frames
# Opção B — baixar artefato pesado (mesmo link público do ARTEFATOS.md):
#   mkdir -p modelos
#   wget -O modelos/cnn_faixas.h5 "https://drive.google.com/uc?export=download&id=SEU_ID"
#   # gerado por: python3 scripts/treinar_cnn.py --epochs 50 --augment  (≈2h em CPU)

# ---------- 3. DEMONSTRAÇÃO DO TP CORRENTE ----------
# ros2 launch meu_pacote bringup.launch.py

echo "[reproduzir] concluído"
