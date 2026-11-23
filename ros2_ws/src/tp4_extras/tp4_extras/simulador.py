"""
MetaDrive com camera sintetica RGB, sem janela, e a geometria usada pelas outras etapas.

  env = criar_ambiente(largura=640, altura=360, mapa='SCS', semente=0)
  obs, info = env.reset(seed=0)
  frame = frame_bgr(obs)                     # (altura, largura, 3) uint8 em BGR, para o OpenCV

O carro e dirigido pelo piloto IDM do proprio simulador (segue a rota, freia e troca de faixa),
entao a acao passada ao env.step() e ignorada; cada passo vale 0,1 s de simulacao. A camera fica
0,8 m a frente e 1,5 m acima do centro do carro (cerca de 2,1 m do chao), com 60 graus de campo
horizontal: o chao so aparece a partir de uns 7 m a frente. Coordenadas do MetaDrive 0.4: x para
a frente, y para a esquerda, em metros; na faixa, a coordenada lateral cresce para a DIREITA.

Sem DISPLAY o panda3d nao abre o contexto OpenGL: use o WSLg (DISPLAY=:0) ou `xvfb-run -a`.
"""

import logging
import os

import numpy as np

SENSOR = 'rgb_camera'
BLOCOS_CRUZAMENTO = ('X', 'T')     # StdInterSection e StdTInterSection


def criar_ambiente(largura=640, altura=360, mapa='SCS', semente=0, cenarios=1, trafego=0.1,
                   acidentes=0.0, faixas=3, faixa_inicial=None, distribuicao=None, extra=None):
    """
    Cria o MetaDriveEnv com a camera RGB como observacao.

    mapa: sequencia de blocos ('S' reta, 'C' curva, 'X' cruzamento, 'T' cruzamento em T, 'O'
    rotatoria, 'r'/'R' rampas) ou um inteiro (quantidade de blocos sorteados pela semente).
    faixa_inicial: indice da faixa de partida (0 = a mais a esquerda); None sorteia.
    distribuicao: dict {nome do bloco: probabilidade} para os mapas sorteados.
    """
    from metadrive import MetaDriveEnv
    from metadrive.component.algorithm.blocks_prob_dist import PGBlockDistConfig
    from metadrive.component.map.base_map import BaseMap
    from metadrive.component.map.pg_map import MapGenerateMethod
    from metadrive.component.pgblock.first_block import FirstPGBlock
    from metadrive.component.sensors.rgb_camera import RGBCamera
    from metadrive.constants import DEFAULT_AGENT
    from metadrive.policy.idm_policy import IDMPolicy
    from panda3d.core import loadPrcFileData

    # sem teclado/joystick: cala o aviso do panda3d sobre /dev/input (WSL e containers)
    loadPrcFileData('', 'notify-level-device fatal')
    os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

    config = {
        'use_render': False,
        'image_observation': True,
        'sensors': {SENSOR: (RGBCamera, int(largura), int(altura))},
        'vehicle_config': {'image_source': SENSOR, 'show_navi_mark': False},
        'stack_size': 1,
        'norm_pixel': False,
        'map_config': {
            BaseMap.GENERATE_TYPE: (MapGenerateMethod.BIG_BLOCK_SEQUENCE if isinstance(mapa, str)
                                    else MapGenerateMethod.BIG_BLOCK_NUM),
            BaseMap.GENERATE_CONFIG: mapa,
            BaseMap.LANE_NUM: int(faixas),
        },
        'start_seed': int(semente),
        'num_scenarios': int(cenarios),
        'traffic_density': float(trafego),
        'accident_prob': float(acidentes),
        'agent_policy': IDMPolicy,
        'random_spawn_lane_index': faixa_inicial is None,
        'log_level': logging.ERROR,
    }
    if faixa_inicial is not None:
        config['agent_configs'] = {DEFAULT_AGENT: {
            'spawn_lane_index': (FirstPGBlock.NODE_1, FirstPGBlock.NODE_2, int(faixa_inicial))}}
    if distribuicao:
        tabela = dict.fromkeys(PGBlockDistConfig.BLOCK_TYPE_DISTRIBUTION_V2, 0.0)
        tabela.update(distribuicao)

        class Distribuicao(PGBlockDistConfig):
            BLOCK_TYPE_DISTRIBUTION_V2 = tabela

        config['block_dist_config'] = Distribuicao
    if extra:
        config.update(extra)
    return MetaDriveEnv(config)


def frame_bgr(obs):
    """Devolve o frame mais recente da camera como imagem BGR uint8 (copia contigua)."""
    return np.ascontiguousarray(obs['image'][..., -1])


def camera(env):
    """Devolve o sensor da camera RGB (panda3d: .cam e o NodePath, .lens a lente)."""
    return env.engine.get_sensor(SENSOR)


def capturar(env):
    """
    Renderiza a cena AGORA e devolve o frame BGR da camera do carro, sem avancar a simulacao.

    Serve para fotografar a cena depois de mexer nela (por exemplo, apos por um objeto na pista);
    o frame de env.step() so mostra o que existia quando o passo foi dado.
    """
    env.engine.taskMgr.step()            # desenha a cena atual; a fisica nao anda
    return np.ascontiguousarray(camera(env).perceive(to_float=False)[..., :3])


def projetar(env, x, y, z=0.0):
    """
    Projeta um ponto do mundo (m) na imagem; devolve (u, v, profundidade) ou None.

    None quando o ponto esta atras da camera ou fora do campo de visao. u, v em pixels
    (origem no canto superior esquerdo); profundidade em metros ao longo do eixo da camera.
    """
    from panda3d.core import Point2, Point3

    cam = camera(env)
    p = cam.cam.getRelativePoint(env.engine.origin, Point3(float(x), float(y), float(z)))
    uv = Point2()
    if p.y <= 0.1 or not cam.lens.project(p, uv):
        return None
    return ((uv.x + 1.0) / 2.0 * cam.BUFFER_W, (1.0 - uv.y) / 2.0 * cam.BUFFER_H, float(p.y))


def bloco_do_no(no):
    """Devolve (indice, letra) do bloco de um no: '2X0_1_' -> (2, 'X'); '>>' -> (0, 'I')."""
    i = 0
    while i < len(no) and no[i].isdigit():
        i += 1
    if i == 0 or i >= len(no):
        return 0, 'I'
    return int(no[:i]), no[i]


def situacao_cruzamento(env):
    """
    Diz onde o carro esta em relacao a um cruzamento (blocos X e T).

    Devolve (dentro, entrada, saida): dentro = a faixa atual liga dois nos do mesmo bloco de
    cruzamento (a area sem faixas pintadas); entrada = metros ate o cruzamento quando a via atual
    termina nele (0.0 se ja esta dentro, None se nao termina); saida = metros ja percorridos
    desde a saida de um cruzamento, se a via atual comeca nele (senao None).
    """
    veiculo = env.agent
    inicio, fim = veiculo.lane_index[0], veiculo.lane_index[1]
    bloco_i, bloco_f = bloco_do_no(inicio), bloco_do_no(fim)
    s, _ = veiculo.lane.local_coordinates(veiculo.position)
    if bloco_f[1] in BLOCOS_CRUZAMENTO:
        if bloco_i == bloco_f:
            return True, 0.0, None
        return False, max(0.0, float(veiculo.lane.length - s)), None
    if bloco_i[1] in BLOCOS_CRUZAMENTO:
        return False, None, max(0.0, float(s))
    return False, None, None


def faixa_seguinte(env):
    """Devolve a faixa da proxima via da rota com o mesmo indice da atual (ou a mais proxima)."""
    veiculo = env.agent
    proximas = getattr(veiculo.navigation, 'next_ref_lanes', None)
    if not proximas:
        return None
    return proximas[min(veiculo.lane_index[2], len(proximas) - 1)]


def distancia_obstaculo(env, alcance=60.0, meia_largura=1.5):
    """
    Mede a distancia (m) ate o objeto mais proximo a frente NA FAIXA do carro, ou None.

    Conta veiculos, cones e barreiras cujo centro esta a menos de meia_largura do eixo da faixa
    atual (ou da faixa seguinte da rota, quando o objeto ja passou do fim da atual). A medida e ao
    longo da faixa, entao vale tambem nas curvas.
    """
    veiculo = env.agent
    faixa = veiculo.lane
    s_ego, _ = faixa.local_coordinates(veiculo.position)
    resto = faixa.length - s_ego
    seguinte = faixa_seguinte(env)
    posicao = np.asarray(veiculo.position, dtype=float)
    menor = None
    for objeto in env.engine.get_objects().values():
        if objeto is veiculo or not hasattr(objeto, 'position'):
            continue
        p = np.asarray(objeto.position, dtype=float)
        if np.linalg.norm(p - posicao) > alcance + 10.0:
            continue
        s, lateral = faixa.local_coordinates(p)
        if -1.0 <= s <= faixa.length + 0.5:
            frente = s - s_ego
        elif seguinte is not None and s > faixa.length:
            s, lateral = seguinte.local_coordinates(p)
            if not -1.0 <= s <= seguinte.length + 0.5:
                continue
            frente = resto + s
        else:
            continue
        if abs(lateral) > meia_largura or not 2.0 < frente <= alcance:
            continue
        if menor is None or frente < menor:
            menor = float(frente)
    return menor


def bordas_da_faixa(env, distancias=(8.0, 12.0, 18.0)):
    """
    Projeta na imagem as bordas esquerda e direita da faixa do carro (gabarito do simulador).

    Devolve uma lista de ((u_esq, v_esq), (u_dir, v_dir)), um par por distancia a frente, ou None
    se a faixa atual acaba antes da maior distancia ou se algum ponto cai fora da imagem.
    """
    veiculo = env.agent
    faixa = veiculo.lane
    s, _ = faixa.local_coordinates(veiculo.position)
    if s + max(distancias) > faixa.length:
        return None
    pares = []
    for d in distancias:
        meia = faixa.width_at(s + d) / 2.0
        esq = projetar(env, *faixa.position(s + d, -meia))
        dire = projetar(env, *faixa.position(s + d, meia))
        if esq is None or dire is None:
            return None
        pares.append((esq[:2], dire[:2]))
    return pares
