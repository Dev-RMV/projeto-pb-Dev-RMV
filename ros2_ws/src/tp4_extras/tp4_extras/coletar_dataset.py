"""
Coleta de frames rotulados do MetaDrive em 3 classes: via livre, obstaculo e cruzamento.

O carro (piloto IDM do simulador) percorre mapas sorteados por semente, com trafego e cruzamentos
em X e em T. A cada passo o rotulo sai do ESTADO DO SIMULADOR, nao da imagem:
  via_livre   nada na faixa do carro em 40 m e a via atual nao termina num cruzamento;
  obstaculo   ha um veiculo, cone ou barreira na faixa do carro a ate 20 m (medidos ao longo da
              faixa), longe de cruzamento;
  cruzamento  o carro esta a ate 18 m da entrada de um cruzamento ou dentro dele (com pelo menos
              10 m de cruzamento pela frente).
Os casos de fronteira (obstaculo entre 20 e 40 m, cruzamento a mais de 18 m, obstaculo dentro do
cruzamento, os primeiros 15 m depois de um cruzamento, carro trocando de faixa) sao descartados,
para as classes nao se misturarem.

Obstaculo perto e raro com o piloto IDM, que freia ou troca de faixa bem antes. Por isso, a cada
8 m de via livre o coletor sorteia: ou guarda o frame como via livre, ou poe um obstaculo na faixa
do carro (cones, barreira, carro parado ou triangulo, entre 7 e 19 m a frente), fotografa a cena
e tira o obstaculo antes do passo seguinte. As duas classes saem assim das mesmas pistas e do
mesmo transito: o que as separa e o obstaculo. Os obstaculos que o proprio transito cria (carro
lento ou parado na faixa, acidente do MetaDrive) tambem entram, com origem `cena` no manifesto.

Sao 500 frames, as 3 classes com o mesmo numero, em treino (70%), validacao (15%) e teste (15%).
Cada particao usa sementes proprias (mapas diferentes): a validacao e o teste medem a
generalizacao para pistas que a rede nao viu. Cada episodio cede no maximo 10 frames de via livre,
12 de obstaculo e 12 de cruzamento (um a cada 2,5 m percorridos).

  ros2 run tp4_extras coletar_dataset              # 500 frames -> <ws>/log/tp4_extras/dataset
  ros2 run tp4_extras coletar_dataset --total 900 --semente 5000

Saidas: dataset/<particao>/<classe>/*.jpg (320x180), dataset/manifesto.csv (arquivo, particao,
classe, semente, passo e as medidas que geraram o rotulo), dataset_resumo.json e
dataset_amostras.jpg (amostras de cada classe).
"""

import argparse
import collections
import csv
import json
import os
import shutil
import time

import cv2
import numpy as np

from tp4_extras.ambiente import pasta_saida
from tp4_extras.desenho import grade, rotulo

CLASSES = ('via_livre', 'obstaculo', 'cruzamento')
PARTICOES = (('treino', 0.70, 0), ('validacao', 0.15, 2000), ('teste', 0.15, 4000))
LIMIARES = {
    'obstaculo_max_m': 20.0,       # obstaculo na faixa ate aqui -> classe obstaculo
    'livre_min_m': 40.0,           # nada na faixa ate aqui -> pode ser via livre
    'entrada_cruzamento_m': 18.0,  # distancia ate o cruzamento para ja rotular cruzamento
    'resto_cruzamento_m': 10.0,    # dentro do cruzamento, minimo de cruzamento ainda pela frente
    'saida_cruzamento_m': 15.0,    # primeiros metros depois do cruzamento: nao rotula
    'lateral_max_m': 0.9,          # carro mais longe que isso do eixo da faixa: trocando de faixa
}
BLOCOS = {'Curve': 0.2, 'Straight': 0.2, 'StdInterSection': 0.3, 'StdTInterSection': 0.3}
# amostragem por episodio: (maximo de frames, metros percorridos entre dois frames da classe);
# o espaco de via_livre vale tambem para os obstaculos postos na faixa
AMOSTRAGEM = {'via_livre': (10, 8.0), 'obstaculo': (12, 4.0), 'cruzamento': (12, 2.5)}
# obstaculos postos na faixa: tipo -> (distancia minima, distancia maxima) em metros
OBSTACULOS = {'cones': (7.0, 19.0), 'barreira': (7.0, 19.0), 'veiculo': (8.0, 19.0),
              'triangulo': (7.0, 12.0)}
CAMPOS = ('arquivo', 'particao', 'classe', 'rotulo', 'semente', 'passo', 'origem', 'obstaculo',
          'dist_obstaculo_m', 'dist_cruzamento_m', 'dentro_do_cruzamento', 'bloco',
          'velocidade_kmh')


def cotas(total):
    """Divide `total` frames em particoes e classes: {particao: {classe: n}}."""
    por_particao = [int(round(total * fracao)) for _, fracao, _ in PARTICOES]
    por_particao[0] += total - sum(por_particao)
    tabela = {}
    for (nome, _, _), n in zip(PARTICOES, por_particao):
        base, resto = divmod(n, len(CLASSES))
        tabela[nome] = {classe: base + (1 if i < resto else 0) for i, classe in enumerate(CLASSES)}
    return tabela


def rotular(env, limiares=LIMIARES):
    """
    Devolve (classe ou None, medidas) para o estado atual do simulador.

    None = caso de fronteira, que nao entra no dataset. medidas: distancia ao obstaculo na faixa
    (m), distancia ao cruzamento (m), dentro do cruzamento, bloco e velocidade (km/h).
    """
    from tp4_extras.simulador import bloco_do_no, distancia_obstaculo, situacao_cruzamento

    veiculo = env.agent
    dentro, entrada, saida = situacao_cruzamento(env)
    dist_obst = distancia_obstaculo(env)
    s, lateral = veiculo.lane.local_coordinates(veiculo.position)
    medidas = {'dist_obstaculo_m': dist_obst, 'dist_cruzamento_m': entrada,
               'dentro_do_cruzamento': dentro, 'bloco': bloco_do_no(veiculo.lane_index[1])[1],
               'velocidade_kmh': float(veiculo.speed_km_h), 'origem': 'cena', 'obstaculo': ''}
    perto = dist_obst is not None and dist_obst <= limiares['obstaculo_max_m']
    if dentro:
        if perto or veiculo.lane.length - s < limiares['resto_cruzamento_m']:
            return None, medidas
        return 'cruzamento', medidas
    if abs(lateral) > limiares['lateral_max_m']:
        return None, medidas
    if entrada is not None:
        if entrada <= limiares['entrada_cruzamento_m'] and not perto:
            return 'cruzamento', medidas
        return None, medidas
    if saida is not None and saida < limiares['saida_cruzamento_m']:
        return None, medidas
    if perto:
        return 'obstaculo', medidas
    if dist_obst is None or dist_obst >= limiares['livre_min_m']:
        return 'via_livre', medidas
    return None, medidas


def fotografar_obstaculo(env, sorteio):
    """
    Poe um obstaculo na faixa do carro, fotografa a cena e tira o obstaculo.

    O obstaculo so existe para a camera: e removido antes do passo seguinte da simulacao, entao
    nem o carro nem o transito reagem a ele. Devolve (frame BGR, tipo, distancia em m) ou None
    se a faixa atual acaba antes do ponto sorteado.
    """
    from metadrive.component.static_object import traffic_object

    from tp4_extras.simulador import capturar

    veiculo = env.agent
    faixa = veiculo.lane
    s, _ = faixa.local_coordinates(veiculo.position)
    tipo = sorted(OBSTACULOS)[int(sorteio.integers(len(OBSTACULOS)))]
    distancia = float(sorteio.uniform(*OBSTACULOS[tipo]))
    onde = float(s + distancia)
    if onde + 3.0 > faixa.length:
        return None
    motor = env.engine
    rumo = faixa.heading_theta_at(onde)
    objetos = []
    try:
        if tipo == 'veiculo':
            carro = motor.spawn_object(motor.traffic_manager.random_vehicle_type(),
                                       vehicle_config={'spawn_lane_index': faixa.index,
                                                       'spawn_longitude': onde})
            carro.set_break_down()
            objetos.append(carro)
        else:
            classe = {'cones': traffic_object.TrafficCone,
                      'barreira': traffic_object.TrafficBarrier,
                      'triangulo': traffic_object.TrafficWarning}[tipo]
            for lateral in ((-1.1, 0.0, 1.1) if tipo == 'cones' else (0.0,)):
                objetos.append(motor.spawn_object(classe, lane=faixa, static=True,
                                                  position=faixa.position(onde, lateral),
                                                  heading_theta=rumo))
        frame = capturar(env)
    finally:
        if objetos:
            motor.clear_objects([objeto.id for objeto in objetos])
    return frame, tipo, distancia


def linha_do_manifesto(arquivo, particao, classe, semente, passo, medidas):
    """Monta a linha do manifesto de um frame salvo."""
    def metros(valor):
        return '' if valor is None else '%.1f' % valor

    return {'arquivo': arquivo.replace(os.sep, '/'), 'particao': particao, 'classe': classe,
            'rotulo': CLASSES.index(classe), 'semente': semente, 'passo': passo,
            'origem': medidas['origem'], 'obstaculo': medidas['obstaculo'],
            'dist_obstaculo_m': metros(medidas['dist_obstaculo_m']),
            'dist_cruzamento_m': metros(medidas['dist_cruzamento_m']),
            'dentro_do_cruzamento': int(medidas['dentro_do_cruzamento']),
            'bloco': medidas['bloco'], 'velocidade_kmh': '%.1f' % medidas['velocidade_kmh']}


def coletar_particao(env, nome, cota, pasta, semente, cenarios, passos_max, escritor):
    """Roda episodios ate encher a cota da particao; devolve (contagem, sementes usadas)."""
    from tp4_extras.simulador import frame_bgr

    sorteio = np.random.default_rng(semente)
    contagem = dict.fromkeys(CLASSES, 0)
    usadas = []
    for episodio in range(cenarios):
        if all(contagem[c] >= cota[c] for c in CLASSES):
            break
        env.reset(seed=semente + episodio)
        no_episodio = dict.fromkeys(CLASSES, 0)
        ultimo = dict.fromkeys(CLASSES, -1e9)
        anterior = np.asarray(env.agent.position, dtype=float)
        percorrido, parado, salvou = 0.0, 0, False

        def falta(classe):
            return contagem[classe] < cota[classe] and \
                no_episodio[classe] < AMOSTRAGEM[classe][0]

        for passo in range(passos_max):
            obs, _, terminou, truncou, _ = env.step([0.0, 0.0])
            posicao = np.asarray(env.agent.position, dtype=float)
            percorrido += float(np.linalg.norm(posicao - anterior))
            anterior = posicao
            parado = parado + 1 if env.agent.speed_km_h < 1.0 else 0
            classe, medidas = rotular(env)
            frame = None
            if classe is not None and passo >= 10 and \
                    percorrido - ultimo[classe] >= AMOSTRAGEM[classe][1]:
                if classe != 'via_livre':
                    if falta(classe):
                        frame = frame_bgr(obs)
                elif falta('obstaculo') and (not falta('via_livre') or sorteio.random() < 0.5):
                    foto = fotografar_obstaculo(env, sorteio)
                    if foto is not None:
                        frame, tipo, distancia = foto
                        ultimo['via_livre'] = percorrido
                        classe = 'obstaculo'
                        medidas.update(origem='colocado', obstaculo=tipo,
                                       dist_obstaculo_m=distancia)
                elif falta('via_livre'):
                    frame = frame_bgr(obs)
            if frame is not None:
                contagem[classe] += 1
                no_episodio[classe] += 1
                ultimo[classe] = percorrido
                salvou = True
                arquivo = os.path.join(nome, classe,
                                       '%s_%s_%04d.jpg' % (nome, classe, contagem[classe]))
                cv2.imwrite(os.path.join(pasta, arquivo), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                escritor.writerow(linha_do_manifesto(arquivo, nome, classe, semente + episodio,
                                                     passo, medidas))
            # fim do episodio, carro preso num bloqueio ou nada mais a colher neste mapa
            if terminou or truncou or parado >= 40 or not any(falta(c) for c in CLASSES):
                break
        if salvou:
            usadas.append(semente + episodio)
        print('[dataset] %-9s semente %d: %s' % (nome, semente + episodio, ', '.join(
            '%s %d/%d' % (c, contagem[c], cota[c]) for c in CLASSES)), flush=True)
    return contagem, usadas


def mosaico_de_amostras(pasta, linhas, por_classe=8, largura=200, semente=0):
    """Monta uma imagem com `por_classe` amostras de treino de cada classe, uma por linha."""
    sorteio = np.random.default_rng(semente)
    imagens = []
    for classe in CLASSES:
        arquivos = [linha['arquivo'] for linha in linhas
                    if linha['classe'] == classe and linha['particao'] == 'treino']
        escolhidos = sorteio.choice(len(arquivos), size=min(por_classe, len(arquivos)),
                                    replace=False) if arquivos else []
        for n, i in enumerate(sorted(escolhidos)):
            imagem = cv2.imread(os.path.join(pasta, arquivos[int(i)]))
            if n == 0:
                rotulo(imagem, classe, escala=0.6, cor=(0, 255, 255))
            imagens.append(imagem)
        vazio = np.zeros((90, 160, 3), np.uint8) if not imagens else np.zeros_like(imagens[-1])
        imagens += [vazio] * (por_classe - len(escolhidos))
    return grade(imagens, por_classe, largura)


def main(argv=None):
    from tp4_extras.simulador import criar_ambiente

    ap = argparse.ArgumentParser(description='Coleta de frames rotulados (3 classes) no MetaDrive')
    ap.add_argument('--total', type=int, default=500, help='frames no dataset')
    ap.add_argument('--semente', type=int, default=1000,
                    help='primeira semente do treino (validacao +2000, teste +4000)')
    ap.add_argument('--cenarios', type=int, default=300, help='maximo de episodios por particao')
    ap.add_argument('--blocos', type=int, default=4, help='blocos sorteados por mapa')
    ap.add_argument('--trafego', type=float, default=0.15, help='densidade de trafego (0-1)')
    ap.add_argument('--acidentes', type=float, default=0.3,
                    help='probabilidade de acidente do MetaDrive por bloco (cones, carro parado)')
    ap.add_argument('--passos_max', type=int, default=1200, help='passos maximos por episodio')
    ap.add_argument('--largura', type=int, default=320)
    ap.add_argument('--altura', type=int, default=180)
    ap.add_argument('--saida', default=None, help='pasta base (padrao <ws>/log/tp4_extras)')
    args = ap.parse_args(argv)

    base = pasta_saida('', args.saida)
    pasta = os.path.join(base, 'dataset')
    if os.path.isdir(pasta):
        if os.listdir(pasta) and not os.path.isfile(os.path.join(pasta, 'manifesto.csv')):
            raise SystemExit('[dataset] %s existe e nao e um dataset deste script; apague-a ou '
                             'use --saida' % pasta)
        shutil.rmtree(pasta)
    tabela = cotas(args.total)
    for nome in tabela:
        for classe in CLASSES:
            os.makedirs(os.path.join(pasta, nome, classe))
    t0 = time.time()
    contagens, sementes = {}, {}
    with open(os.path.join(pasta, 'manifesto.csv'), 'w', newline='', encoding='utf-8') as arq:
        escritor = csv.DictWriter(arq, fieldnames=CAMPOS)
        escritor.writeheader()
        for nome, _, deslocamento in PARTICOES:
            inicio = args.semente + deslocamento
            env = criar_ambiente(args.largura, args.altura, mapa=args.blocos, semente=inicio,
                                 cenarios=args.cenarios, trafego=args.trafego,
                                 acidentes=args.acidentes, distribuicao=BLOCOS)
            try:
                contagens[nome], sementes[nome] = coletar_particao(
                    env, nome, tabela[nome], pasta, inicio, args.cenarios, args.passos_max,
                    escritor)
            finally:
                env.close()
    with open(os.path.join(pasta, 'manifesto.csv'), newline='', encoding='utf-8') as arq:
        linhas = list(csv.DictReader(arq))
    cv2.imwrite(os.path.join(base, 'dataset_amostras.jpg'), mosaico_de_amostras(pasta, linhas),
                [cv2.IMWRITE_JPEG_QUALITY, 90])
    completo = all(contagens[n][c] == tabela[n][c] for n in tabela for c in CLASSES)

    def contar(campo, classe):
        return dict(sorted(collections.Counter(
            linha[campo] for linha in linhas if linha['classe'] == classe).items()))

    resumo = {
        'total': len(linhas), 'pedido': args.total, 'completo': completo,
        'classes': list(CLASSES), 'resolucao': [args.largura, args.altura],
        'frames_por_particao_e_classe': contagens,
        'frames_por_classe': {c: sum(contagens[n][c] for n in contagens) for c in CLASSES},
        'sementes_usadas': {n: [min(s), max(s), len(s)] if s else [] for n, s in sementes.items()},
        'obstaculos': {'origem': contar('origem', 'obstaculo'),
                       'tipo_dos_colocados': {tipo: n for tipo, n in
                                              contar('obstaculo', 'obstaculo').items() if tipo}},
        'blocos_dos_frames': {c: contar('bloco', c) for c in CLASSES},
        'limiares': LIMIARES, 'blocos_do_mapa': BLOCOS, 'blocos_por_mapa': args.blocos,
        'trafego': args.trafego, 'acidentes': args.acidentes,
        'amostragem_por_episodio': {c: {'maximo': m, 'metros_entre_frames': e}
                                    for c, (m, e) in AMOSTRAGEM.items()},
        'obstaculos_colocados_m': OBSTACULOS,
        'tempo_s': round(time.time() - t0, 1),
    }
    with open(os.path.join(base, 'dataset_resumo.json'), 'w', encoding='utf-8') as arq:
        json.dump(resumo, arq, indent=2, ensure_ascii=False)
    print('[dataset] %d frames em %.0f s: %s' % (len(linhas), resumo['tempo_s'], ', '.join(
        '%s %d' % (c, n) for c, n in resumo['frames_por_classe'].items())))
    for nome in contagens:
        print('[dataset]   %-9s %s (%d episodios com frames)'
              % (nome, contagens[nome], len(sementes[nome])))
    print('[dataset] obstaculos: %s; colocados por tipo: %s'
          % (resumo['obstaculos']['origem'], resumo['obstaculos']['tipo_dos_colocados']))
    print('[dataset] saidas em %s (manifesto.csv), %s e dataset_resumo.json'
          % (pasta, os.path.join(base, 'dataset_amostras.jpg')))
    if not completo:
        print('[dataset] AVISO: faltaram frames de alguma classe; aumente --cenarios')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
