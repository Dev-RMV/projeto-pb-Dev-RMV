#!/usr/bin/env bash
# demo_parametros.sh — demonstra parametros do YAML e alteracao dinamica no /rastreador_yolo.
# Pre-requisito: o TP2 rodando (./scripts/reproduzir.sh ou ros2 launch projeto_bloco tp2.launch.py)
# e o ambiente carregado (source ros2_ws/ambiente.sh). Cada mudanca aparece no log do no e no
# cabecalho da imagem /vision/rastreamento (modelo, limiar, Hz).
NO=/rastreador_yolo
CFG="$(ros2 pkg prefix projeto_bloco)/share/projeto_bloco/config"
PAUSA=${PAUSA:-4}

passo() { echo; echo "[param] \$ $*"; "$@" 2>&1 | sed 's/^/    /'; }
# A camera perde ~2,5 s de quadros a cada ~34 s (Fast-DDS SHM, ver docs/diario.md): se isso cair na
# janela, a media sai menor e aparece como max ~2 s na linha min/max.
taxa() {   # taxa medida de /vision/deteccoes (~8 s de amostra)
  echo "[param] taxa medida de /vision/deteccoes:"
  timeout 9 ros2 topic hz /vision/deteccoes 2>/dev/null | tail -2 | sed 's/^s*/    /'
}

ros2 node list 2>/dev/null | grep -qx "$NO" || { echo "[param] $NO nao esta rodando"; exit 1; }

echo "[param] ===== 1. valores carregados do config/params.yaml pelo launch ====="
passo ros2 param get $NO modelo
passo ros2 param get $NO confianca
passo ros2 param get $NO frequencia_hz
passo ros2 param get $NO classes
taxa

echo; echo "[param] ===== 2. alteracao dinamica com ros2 param set ====="
passo ros2 param set $NO confianca 0.7
sleep "$PAUSA"
passo ros2 param set $NO frequencia_hz 3.0
sleep 2; taxa
passo ros2 param set $NO imgsz 960               # entrada maior: alvos menores, inferencia mais lenta
sleep "$PAUSA"
# troca de modelo em execucao: o YOLO11n do COCO nao tem a classe "alvo", entao antes libera todas
passo ros2 param set $NO classes "['*']"
passo ros2 param set $NO modelo yolo11n.pt       # peso oficial, baixado na 1a vez
sleep "$PAUSA"
passo ros2 param set $NO modelo alvo_yolov8s_640.pt
passo ros2 param set $NO classes "['alvo']"
sleep "$PAUSA"

echo; echo "[param] ===== 3. valores invalidos sao recusados ====="
passo ros2 param set $NO confianca 1.5           # fora da faixa 0.05-0.95
passo ros2 param set $NO classes "['dragao']"    # classe que o modelo nao conhece
passo ros2 param set $NO modelo inexistente.pt   # arquivo que nao existe

echo; echo "[param] ===== 4. ros2 param load: perfil alternativo e volta ao YAML principal ====="
passo ros2 param load $NO "$CFG/params_economia.yaml"
sleep 2; taxa
passo ros2 param load $NO "$CFG/params.yaml"
sleep 5; taxa      # volta a entrada para 640: espera acomodar

echo; echo "[param] ===== 5. estado final (ros2 param dump) ====="
passo ros2 param dump $NO
