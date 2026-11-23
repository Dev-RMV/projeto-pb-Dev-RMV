"""Utilidades de desenho das imagens de saida: rotulos legiveis e mosaicos."""

import cv2
import numpy as np


def rotulo(imagem, texto, origem=(6, 6), escala=0.55, cor=(255, 255, 255), fundo=(0, 0, 0)):
    """Escreve `texto` com um retangulo de fundo; `origem` e o canto superior esquerdo."""
    fonte = cv2.FONT_HERSHEY_SIMPLEX
    (larg, alt), base = cv2.getTextSize(texto, fonte, escala, 1)
    x, y = int(origem[0]), int(origem[1])
    cv2.rectangle(imagem, (x, y), (x + larg + 8, y + alt + base + 8), fundo, -1)
    cv2.putText(imagem, texto, (x + 4, y + alt + 4), fonte, escala, cor, 1, cv2.LINE_AA)
    return imagem


def redimensionar(imagem, largura):
    """Redimensiona para `largura` mantendo a proporcao."""
    alt, larg = imagem.shape[:2]
    if larg == largura:
        return imagem.copy()
    interp = cv2.INTER_AREA if largura < larg else cv2.INTER_NEAREST
    return cv2.resize(imagem, (int(largura), int(round(alt * largura / larg))),
                      interpolation=interp)


def em_bgr(imagem):
    """Devolve a imagem com 3 canais (uma imagem em tons de cinza e repetida nos tres)."""
    if imagem.ndim == 2:
        return cv2.cvtColor(imagem, cv2.COLOR_GRAY2BGR)
    return imagem


def tira(paineis, largura=480):
    """Junta lado a lado os paineis [(titulo, imagem), ...], todos com a mesma largura."""
    prontos = []
    for titulo, imagem in paineis:
        imagem = redimensionar(em_bgr(imagem), largura)
        if titulo:
            rotulo(imagem, titulo, cor=(0, 255, 255))
        prontos.append(imagem)
    altura = min(p.shape[0] for p in prontos)
    return np.hstack([p[:altura] for p in prontos])


def grade(imagens, colunas, largura=200, fundo=0):
    """Monta uma grade com `colunas` imagens por linha, todas reduzidas a `largura`."""
    celulas = [redimensionar(em_bgr(imagem), largura) for imagem in imagens]
    if not celulas:
        return np.zeros((largura, largura, 3), np.uint8)
    altura = min(c.shape[0] for c in celulas)
    celulas = [c[:altura] for c in celulas]
    while len(celulas) % colunas:
        celulas.append(np.full_like(celulas[0], fundo))
    linhas = [np.hstack(celulas[i:i + colunas]) for i in range(0, len(celulas), colunas)]
    return np.vstack(linhas)
