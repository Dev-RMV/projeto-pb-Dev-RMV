"""
Placas de transito na cena do MetaDrive, para a deteccao de formas.

O MetaDrive nao traz placas: aqui elas sao criadas como poligonos texturizados do panda3d (um
circulo, um octogono, um triangulo e um losango, com a arte desenhada pelo OpenCV), fincadas na
calcada a direita da pista e viradas para quem vem pela via. Elas passam pela mesma camera
sintetica do carro, entao aparecem com perspectiva, escala e recorte reais.

  placas = instalar_placas(env)            # depois de cada env.reset()
  visiveis = placas_na_imagem(env, placas)  # gabarito: forma, centro (u, v) e raio em px
  remover_placas(placas)                    # antes do proximo reset

Tipos (sinalizacao brasileira): `limite` (regulamentacao, circular), `pare` (octogonal),
`preferencia` (triangulo invertido) e `advertencia` (losango amarelo).
"""

import math

import cv2
import numpy as np

from tp4_extras.simulador import BLOCOS_CRUZAMENTO, bloco_do_no, projetar

# forma geometrica e numero de cantos de cada tipo
FORMAS = {'limite': 'circular', 'pare': 'octogonal', 'preferencia': 'triangular',
          'advertencia': 'losango'}
CANTOS = {'circular': 0, 'octogonal': 8, 'triangular': 3, 'losango': 4}
ORDEM = ('limite', 'preferencia', 'pare', 'advertencia')


def contorno(tipo):
    """Devolve os vertices (x, z) da placa num quadrado de lado 1 centrado na origem."""
    if tipo == 'limite':
        angulos = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
        return [(0.5 * math.cos(a), 0.5 * math.sin(a)) for a in angulos]
    if tipo == 'pare':
        raio = 0.5 / math.cos(math.pi / 8.0)        # lados encostados no quadrado
        return [(raio * math.cos(math.pi / 8.0 + k * math.pi / 4.0),
                 raio * math.sin(math.pi / 8.0 + k * math.pi / 4.0)) for k in range(8)]
    if tipo == 'preferencia':
        meia = math.sqrt(3.0) / 4.0                 # triangulo equilatero de lado 1, ponta embaixo
        return [(-0.5, meia), (0.5, meia), (0.0, -meia)]
    if tipo == 'advertencia':
        return [(0.0, 0.5), (0.5, 0.0), (0.0, -0.5), (-0.5, 0.0)]
    raise ValueError('tipo de placa desconhecido: %s' % tipo)


def arte(tipo, lado=256):
    """Desenha a arte da placa (BGR, lado x lado); fora do contorno fica preto."""
    imagem = np.zeros((lado, lado, 3), np.uint8)

    def px(pontos, fator=1.0):
        return np.array([[(0.5 + x * fator) * (lado - 1), (0.5 - z * fator) * (lado - 1)]
                         for x, z in pontos], np.int32)

    borda = contorno(tipo)
    vermelho, branco, amarelo, preto = (0, 0, 255), (255, 255, 255), (0, 215, 255), (0, 0, 0)
    fonte = cv2.FONT_HERSHEY_DUPLEX
    if tipo == 'limite':
        cv2.fillPoly(imagem, [px(borda)], vermelho, cv2.LINE_AA)
        cv2.fillPoly(imagem, [px(borda, 0.70)], branco, cv2.LINE_AA)
        cv2.putText(imagem, '60', (int(lado * 0.26), int(lado * 0.62)), fonte, lado / 110.0,
                    preto, max(2, lado // 40), cv2.LINE_AA)
    elif tipo == 'pare':
        cv2.fillPoly(imagem, [px(borda)], vermelho, cv2.LINE_AA)
        cv2.polylines(imagem, [px(borda, 0.88)], True, branco, max(2, lado // 64), cv2.LINE_AA)
        cv2.putText(imagem, 'PARE', (int(lado * 0.17), int(lado * 0.58)), fonte, lado / 170.0,
                    branco, max(2, lado // 64), cv2.LINE_AA)
    elif tipo == 'preferencia':
        cv2.fillPoly(imagem, [px(borda)], vermelho, cv2.LINE_AA)
        cv2.fillPoly(imagem, [px(borda, 0.55)], branco, cv2.LINE_AA)
    else:
        cv2.fillPoly(imagem, [px(borda)], amarelo, cv2.LINE_AA)
        cv2.polylines(imagem, [px(borda, 0.84)], True, preto, max(2, lado // 50), cv2.LINE_AA)
        cv2.putText(imagem, '!', (int(lado * 0.43), int(lado * 0.66)), fonte, lado / 90.0, preto,
                    max(3, lado // 28), cv2.LINE_AA)
    return imagem


def _textura(imagem):
    from panda3d.core import Texture

    alt, larg = imagem.shape[:2]
    bgra = np.dstack([imagem, np.full((alt, larg), 255, np.uint8)])
    textura = Texture('placa')
    textura.setup2dTexture(larg, alt, Texture.T_unsigned_byte, Texture.F_rgba8)
    textura.setRamImage(np.ascontiguousarray(bgra[::-1]).tobytes())   # panda3d: BGRA, de baixo
    return textura


def _malha(pontos):
    """Cria um GeomNode com o poligono (leque de triangulos) no plano xz, com normais e uv."""
    from panda3d.core import (Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat,
                              GeomVertexWriter)

    dados = GeomVertexData('placa', GeomVertexFormat.getV3n3t2(), Geom.UHStatic)
    vertice = GeomVertexWriter(dados, 'vertex')
    normal = GeomVertexWriter(dados, 'normal')
    uv = GeomVertexWriter(dados, 'texcoord')
    for x, z in [(0.0, 0.0)] + list(pontos):
        vertice.addData3(x, 0.0, z)
        normal.addData3(0.0, -1.0, 0.0)
        uv.addData2(x + 0.5, z + 0.5)
    triangulos = GeomTriangles(Geom.UHStatic)
    n = len(pontos)
    for i in range(n):
        triangulos.addVertices(0, 1 + i, 1 + (i + 1) % n)
    geometria = Geom(dados)
    geometria.addPrimitive(triangulos)
    no = GeomNode('placa')
    no.addGeom(geometria)
    return no


def criar_placa(env, tipo, x, y, rumo, tamanho=1.5, altura=2.4):
    """
    Finca uma placa em (x, y), virada para quem trafega no sentido `rumo` (rad).

    tamanho: lado do quadrado que contem a placa (m); altura: centro da placa acima do chao (m).
    Devolve um dict com tipo, forma, posicao do centro, tamanho e os nos do panda3d.
    """
    from panda3d.core import CardMaker

    raiz = env.engine.origin.attachNewNode('placa_' + tipo)
    raiz.setPos(float(x), float(y), 0.0)
    raiz.setH(math.degrees(rumo) - 90.0)          # o y local aponta no sentido da via
    face = raiz.attachNewNode(_malha(contorno(tipo)))
    face.setTexture(_textura(arte(tipo)))
    face.setScale(tamanho)
    face.setZ(altura)
    poste = CardMaker('poste')
    poste.setFrame(-0.05, 0.05, 0.0, altura)
    haste = raiz.attachNewNode(poste.generate())
    haste.setY(0.02)
    haste.setColor(0.35, 0.35, 0.35, 1.0)
    # fora do shader PBR da cena (sem material ele pinta a placa de preto) e sem iluminacao:
    # a placa sai com as cores da textura
    raiz.setShaderOff(1)
    raiz.setLightOff(1)
    raiz.setTwoSided(True)
    return {'tipo': tipo, 'forma': FORMAS[tipo], 'posicao': (float(x), float(y), float(altura)),
            'tamanho': float(tamanho), 'rumo': float(rumo), 'no': raiz}


def instalar_placas(env, espacamento=30.0, primeira=35.0, afastamento=1.2, tamanho=1.5,
                    altura=2.4, tipos=ORDEM):
    """
    Distribui placas ao longo da rota do carro, a direita da pista, alternando os tipos.

    Uma placa a cada `espacamento` metros de via, a `afastamento` metros da borda direita. As
    vias de dentro dos cruzamentos sao puladas. Devolve a lista de placas criadas.
    """
    mapa = env.current_map
    nos = env.agent.navigation.checkpoints
    placas = []
    percorrido, proxima = 0.0, float(primeira)
    for inicio, fim in zip(nos[:-1], nos[1:]):
        faixas = mapa.road_network.graph[inicio][fim]
        direita = faixas[-1]
        dentro = bloco_do_no(inicio) == bloco_do_no(fim) and \
            bloco_do_no(fim)[1] in BLOCOS_CRUZAMENTO
        while not dentro and proxima <= percorrido + direita.length:
            s = proxima - percorrido
            x, y = direita.position(s, direita.width_at(s) / 2.0 + afastamento)
            tipo = tipos[len(placas) % len(tipos)]
            placas.append(criar_placa(env, tipo, x, y, direita.heading_theta_at(s), tamanho,
                                      altura))
            proxima += espacamento
        if dentro:
            proxima = max(proxima, percorrido + direita.length + 10.0)
        percorrido += direita.length
    return placas


def remover_placas(placas):
    """Tira as placas da cena (chamar antes do env.reset() seguinte)."""
    for placa in placas:
        placa['no'].removeNode()
    del placas[:]


def placas_na_imagem(env, placas):
    """
    Devolve o gabarito das placas a frente da camera: forma, centro (u, v), raio e distancia.

    Entram as placas viradas para a camera cujo centro cai na imagem. `raio` e o raio aparente em
    px (metade do lado da placa); `inteira` diz se a placa cabe toda na imagem; `distancia` em m.
    """
    from tp4_extras.simulador import camera

    cam = camera(env)
    larg, alt = cam.BUFFER_W, cam.BUFFER_H
    veiculo = np.asarray(env.agent.position, dtype=float)
    visiveis = []
    for placa in placas:
        x, y, z = placa['posicao']
        frente = (x - veiculo[0]) * math.cos(placa['rumo']) + \
            (y - veiculo[1]) * math.sin(placa['rumo'])
        centro = projetar(env, x, y, z)
        if frente <= 0.0 or centro is None:
            continue
        u, v = centro[0], centro[1]
        # raio pela distancia ao longo do eixo da camera e a distancia focal em px
        foco = larg / (2.0 * math.tan(math.radians(cam.lens.getFov()[0]) / 2.0))
        raio = foco * (placa['tamanho'] / 2.0) / centro[2]
        inteira = u - raio >= 0 and u + raio < larg and v - raio >= 0 and v + raio < alt
        visiveis.append({'tipo': placa['tipo'], 'forma': placa['forma'], 'u': float(u),
                         'v': float(v), 'raio': float(raio), 'distancia': float(centro[2]),
                         'inteira': bool(inteira)})
    return visiveis
