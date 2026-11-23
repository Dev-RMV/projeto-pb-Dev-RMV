"""
Deteccao de formas geometricas (placas de transito) com Hough Circle e Harris Corner no MetaDrive.

As placas (circular, octogonal, triangular e losango) sao postas na cena pelo modulo `placas` e
vistas pela camera sintetica do carro. Por frame (BGR):
  1. regioes candidatas pela cor da placa em HSV (vermelho e amarelo saturados), com filtro de
     tamanho, proporcao e solidez; a silhueta de cada regiao (envoltoria convexa preenchida) e
     levada a um quadrado de lado fixo, para os limiares nao dependerem da distancia;
  2. Hough Circle (cv2.HoughCircles) na silhueta: a placa circular fecha um circulo;
  3. Harris Corner (cv2.cornerHarris) na silhueta: os picos da resposta sao os vertices do
     poligono; 3 cantos = triangular, 4 = losango, 5 a 9 = octogonal, nenhum = circular.
A cor so escolhe ONDE procurar: sozinhos, o Hough Circle e o Harris disparam tambem em rodas,
nuvens e marcas da pista (os paineis "bruto" do mosaico mostram isso no frame inteiro).

Conferencia: a posicao de cada placa no simulador, projetada na imagem, diz quais estavam visiveis
e de que forma eram; a saida traz acertos por forma e por tamanho aparente (so entram as placas
inteiras na imagem e com raio aparente de pelo menos --raio_min px) e os falsos positivos
(deteccoes fora de qualquer placa).

  ros2 run tp4_extras detectar_formas                    # 500 passos -> <ws>/log/tp4_extras/formas
  ros2 run tp4_extras detectar_formas --video --mostrar
  ros2 run tp4_extras detectar_formas --mapa SCSS --faixa 1

Saidas: formas_etapas.jpg (camera | Hough Circle bruto | Harris bruto | formas classificadas),
formas_NN.jpg (amostras do resultado) e formas_resumo.json (acertos por forma, falsos positivos).
"""

import argparse
import json
import os
import time

import cv2
import numpy as np

from tp4_extras.ambiente import pasta_saida
from tp4_extras.desenho import rotulo, tira

PARAMETROS = {
    # faixas HSV (OpenCV: H 0-179); o vermelho da a volta pelo zero
    'cores': {
        'vermelho': (((0, 120, 90), (8, 255, 255)), ((170, 120, 90), (179, 255, 255))),
        'amarelo': (((18, 170, 160), (34, 255, 255)),),
    },
    'lado_min': 14,             # menor lado da caixa da regiao (px em 640 de largura)
    'proporcao': (0.55, 1.8),   # largura / altura da caixa
    'solidez_min': 0.85,        # area do contorno / area da envoltoria convexa
    'lado_silhueta': 128,       # silhueta normalizada: quadrado deste lado...
    'margem_silhueta': 16,      # ...com esta margem preta em volta
    'suavizacao': 2.0,          # desvio (px) do borrao gaussiano da silhueta
    # Hough Circle (metodo do gradiente) na silhueta normalizada
    'circulo': {'dp': 1.0, 'param1': 100.0, 'param2': 22.0, 'raio_min': 40, 'raio_max': 56},
    # Harris na silhueta normalizada; um canto e um maximo local acima de `fracao` do maior
    # pico e acima do piso absoluto `resposta_min`. Respostas medidas nas placas da cena:
    # circulo 0,00; octogono (135 graus) 0,03; losango (90 graus) 0,16; triangulo (60 graus) 0,23
    'harris': {'bloco': 9, 'abertura': 5, 'k': 0.04, 'fracao': 0.28, 'resposta_min': 0.008,
               'vizinhanca': 17},
    'canto_agudo': 0.19,        # resposta acima disto = canto mais fechado que 90 graus
}

CORES_DESENHO = {'circular': (0, 255, 0), 'triangular': (255, 0, 255), 'losango': (255, 128, 0),
                 'octogonal': (0, 128, 255), 'indefinida': (160, 160, 160)}


def regioes_por_cor(frame, par=PARAMETROS):
    """Passo 1: regioes candidatas (cor de placa, tamanho, proporcao e solidez)."""
    escala = frame.shape[1] / 640.0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    regioes = []
    mascara_total = np.zeros(frame.shape[:2], np.uint8)
    for cor, faixas in par['cores'].items():
        mascara = np.zeros(frame.shape[:2], np.uint8)
        for baixo, alto in faixas:
            mascara |= cv2.inRange(hsv, np.array(baixo, np.uint8), np.array(alto, np.uint8))
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        mascara_total |= mascara
        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contorno in contornos:
            x, y, larg, alt = cv2.boundingRect(contorno)
            if min(larg, alt) < par['lado_min'] * escala:
                continue
            if not par['proporcao'][0] <= larg / float(alt) <= par['proporcao'][1]:
                continue
            envoltoria = cv2.convexHull(contorno)
            area = cv2.contourArea(envoltoria)
            if area <= 0 or cv2.contourArea(contorno) / area < par['solidez_min']:
                continue
            regioes.append({'cor': cor, 'caixa': (x, y, larg, alt), 'envoltoria': envoltoria})
    return regioes, mascara_total


def silhueta(regiao, par=PARAMETROS):
    """Devolve a silhueta da regiao (envoltoria preenchida) num quadrado de lado fixo."""
    x, y, larg, alt = regiao['caixa']
    lado, margem = par['lado_silhueta'], par['margem_silhueta']
    miolo = lado - 2 * margem
    # a envoltoria e redesenhada ja no tamanho final (e nao ampliada a partir dos pixels), para
    # as bordas sairem retas; o centro de cada pixel da caixa vai para dentro do miolo
    pontos = regiao['envoltoria'].reshape(-1, 2).astype(np.float64)
    pontos[:, 0] = margem + (pontos[:, 0] - x + 0.5) * miolo / float(larg)
    pontos[:, 1] = margem + (pontos[:, 1] - y + 0.5) * miolo / float(alt)
    normal = np.zeros((lado, lado), np.uint8)
    cv2.fillPoly(normal, [np.round(pontos * 16.0).astype(np.int32)], 255, cv2.LINE_AA, 4)
    return cv2.GaussianBlur(normal, (0, 0), par['suavizacao'])


def circulo_hough(sil, par=PARAMETROS):
    """Passo 2: Hough Circle na silhueta. Devolve (cx, cy, raio) ou None."""
    c = par['circulo']
    circulos = cv2.HoughCircles(sil, cv2.HOUGH_GRADIENT, c['dp'], sil.shape[0],
                                param1=c['param1'], param2=c['param2'],
                                minRadius=c['raio_min'], maxRadius=c['raio_max'])
    if circulos is None:
        return None
    return tuple(float(v) for v in circulos[0][0])


def resposta_harris(imagem, par=PARAMETROS):
    """Devolve o mapa de resposta do Harris (float32) de uma imagem em tons de cinza."""
    h = par['harris']
    return cv2.cornerHarris(np.float32(imagem) / 255.0, h['bloco'], h['abertura'], h['k'])


def cantos_harris(sil, par=PARAMETROS):
    """Passo 3: cantos da silhueta (maximos locais da resposta do Harris). Lista de (x, y)."""
    h = par['harris']
    resposta = resposta_harris(sil, par)
    maior = float(resposta.max())
    if maior <= h['resposta_min']:
        return [], maior
    vizinhos = cv2.dilate(resposta, np.ones((h['vizinhanca'], h['vizinhanca']), np.uint8))
    limiar = max(h['fracao'] * maior, h['resposta_min'])
    picos = np.argwhere((resposta >= vizinhos) & (resposta > limiar))
    return [(float(x), float(y)) for y, x in picos], maior


def classificar(n_cantos, pico, tem_circulo, par=PARAMETROS):
    """
    Decide a forma a partir dos cantos do Harris e do circulo do Hough.

    3 cantos agudos = triangular; 4 cantos (ou 3 de 90 graus, com o quarto fraco) = losango;
    5 a 9 cantos = octogonal; sem cantos e com circulo = circular.
    """
    if n_cantos == 3:
        return 'triangular' if pico >= par['canto_agudo'] else 'losango'
    if n_cantos == 4:
        return 'losango'
    if 5 <= n_cantos <= 9:
        return 'octogonal'
    if tem_circulo:
        return 'circular'
    return 'indefinida'


def detectar_formas(frame, par=PARAMETROS):
    """Roda os tres passos num frame BGR. Devolve (lista de deteccoes, mascara de cor)."""
    regioes, mascara = regioes_por_cor(frame, par)
    lado, margem = par['lado_silhueta'], par['margem_silhueta']
    miolo = float(lado - 2 * margem)
    deteccoes = []
    for regiao in regioes:
        x, y, larg, alt = regiao['caixa']
        sil = silhueta(regiao, par)
        circulo = circulo_hough(sil, par)
        cantos, pico = cantos_harris(sil, par)
        forma = classificar(len(cantos), pico, circulo is not None, par)

        def para_imagem(px, py):
            return x + (px - margem) * larg / miolo, y + (py - margem) * alt / miolo

        deteccao = {'forma': forma, 'cor': regiao['cor'], 'caixa': regiao['caixa'],
                    'u': x + larg / 2.0, 'v': y + alt / 2.0, 'raio': max(larg, alt) / 2.0,
                    'n_cantos': len(cantos), 'pico_harris': pico,
                    'cantos': [para_imagem(px, py) for px, py in cantos], 'circulo': None}
        if circulo is not None:
            cu, cv_ = para_imagem(circulo[0], circulo[1])
            deteccao['circulo'] = (cu, cv_, circulo[2] * (larg + alt) / (2.0 * miolo))
        deteccoes.append(deteccao)
    return deteccoes, mascara


def desenhar(frame, deteccoes, texto=True):
    """Desenha as formas classificadas: circulo do Hough, cantos do Harris, caixa e rotulo."""
    saida = frame.copy()
    for d in deteccoes:
        cor = CORES_DESENHO[d['forma']]
        x, y, larg, alt = d['caixa']
        cv2.rectangle(saida, (x - 2, y - 2), (x + larg + 2, y + alt + 2), cor, 1)
        if d['forma'] == 'circular' and d['circulo']:
            cu, cv_, raio = d['circulo']
            cv2.circle(saida, (int(round(cu)), int(round(cv_))), int(round(raio)), cor, 2,
                       cv2.LINE_AA)
        else:
            for px, py in d['cantos']:
                cv2.circle(saida, (int(round(px)), int(round(py))), 3, (0, 0, 255), -1,
                           cv2.LINE_AA)
        if texto:
            detalhe = 'Hough' if d['forma'] == 'circular' else '%d cantos' % d['n_cantos']
            rotulo(saida, '%s (%s)' % (d['forma'], detalhe), (x - 2, max(0, y - 24)), 0.42, cor)
    return saida


def hough_bruto(frame):
    """Roda o Hough Circle no frame inteiro em cinza, sem filtro de cor (para o mosaico)."""
    escala = frame.shape[1] / 640.0
    cinza = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    circulos = cv2.HoughCircles(cinza, cv2.HOUGH_GRADIENT, 1.2, 20 * escala, param1=120,
                                param2=30, minRadius=int(6 * escala), maxRadius=int(90 * escala))
    saida = frame.copy()
    n = 0
    if circulos is not None:
        for cu, cv_, raio in circulos[0]:
            cv2.circle(saida, (int(cu), int(cv_)), int(raio), (0, 255, 0), 1, cv2.LINE_AA)
            n += 1
    return saida, n


def harris_bruto(frame, par=PARAMETROS):
    """Roda o Harris no frame inteiro em cinza e marca os cantos fortes (para o mosaico)."""
    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    resposta = cv2.cornerHarris(np.float32(cinza), 3, 3, 0.04)
    saida = frame.copy()
    fortes = resposta > 0.02 * resposta.max()
    saida[cv2.dilate(fortes.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0] = (0, 0, 255)
    return saida, int(fortes.sum())


def paineis_das_etapas(frame, deteccoes, largura=480):
    """Monta a tira camera | Hough Circle bruto | Harris bruto | formas classificadas."""
    circulos, n_circulos = hough_bruto(frame)
    cantos, _ = harris_bruto(frame)
    return tira([('camera sintetica', frame),
                 ('Hough Circle no frame inteiro: %d' % n_circulos, circulos),
                 ('Harris no frame inteiro', cantos),
                 ('cor + Hough Circle + Harris', desenhar(frame, deteccoes))], largura)


def conferir(deteccoes, gabarito):
    """
    Casa as deteccoes com as placas visiveis (gabarito do simulador).

    Devolve (pares, falsos): pares = [(placa, deteccao ou None)], uma deteccao por placa (a de
    centro mais proximo, a menos de um raio); falsos = deteccoes que nao casaram com placa alguma.
    """
    livres = list(deteccoes)
    pares = []
    for placa in sorted(gabarito, key=lambda p: -p['raio']):
        melhor, menor = None, None
        for d in livres:
            dist = float(np.hypot(d['u'] - placa['u'], d['v'] - placa['v']))
            if dist <= max(placa['raio'], 6.0) and (menor is None or dist < menor):
                melhor, menor = d, dist
        if melhor is not None:
            livres.remove(melhor)
        pares.append((placa, melhor))
    return pares, livres


def main(argv=None):
    from tp4_extras.placas import FORMAS, instalar_placas, placas_na_imagem, remover_placas
    from tp4_extras.simulador import criar_ambiente, frame_bgr

    ap = argparse.ArgumentParser(
        description='Formas em placas (Hough Circle, Harris Corner) no MetaDrive')
    ap.add_argument('--passos', type=int, default=500, help='passos de simulacao (0,1 s cada)')
    ap.add_argument('--cada', type=int, default=2, help='processa um frame a cada N passos')
    ap.add_argument('--mapa', default='SCSS', help='blocos do mapa (S reta, C curva, X, T, O)')
    ap.add_argument('--semente', type=int, default=0)
    ap.add_argument('--cenarios', type=int, default=3, help='cenarios (sementes) disponiveis')
    ap.add_argument('--trafego', type=float, default=0.05, help='densidade de trafego (0-1)')
    ap.add_argument('--faixa', type=int, default=2, help='faixa de partida (0 = esquerda)')
    ap.add_argument('--raio_min', type=float, default=10.0,
                    help='raio aparente minimo (px) para a placa contar na conferencia')
    ap.add_argument('--largura', type=int, default=640)
    ap.add_argument('--altura', type=int, default=360)
    ap.add_argument('--amostras', type=int, default=8, help='frames com o resultado a salvar')
    ap.add_argument('--saida', default=None, help='pasta base (padrao <ws>/log/tp4_extras)')
    ap.add_argument('--video', action='store_true', help='grava formas.mp4 com o resultado')
    ap.add_argument('--mostrar', action='store_true', help='janela ao vivo (precisa de DISPLAY)')
    args = ap.parse_args(argv)

    pasta = pasta_saida('formas', args.saida)
    env = criar_ambiente(args.largura, args.altura, mapa=args.mapa, semente=args.semente,
                         cenarios=args.cenarios, trafego=args.trafego, faixa_inicial=args.faixa)
    semente = args.semente
    obs, _ = env.reset(seed=semente)
    placas = instalar_placas(env)
    raio_min = args.raio_min * args.largura / 640.0
    formas = sorted(set(FORMAS.values()))
    tabela = {f: {'visiveis': 0, 'detectadas': 0, 'forma_certa': 0, 'achado': {}} for f in formas}
    faixas_raio = ((10, 15), (15, 25), (25, 45), (45, 1000))
    por_raio = {'%d-%d px' % f if f[1] < 1000 else '%d+ px' % f[0]:
                {'visiveis': 0, 'forma_certa': 0} for f in faixas_raio}
    total = max(1, args.passos // max(1, args.cada))
    salvar_a_cada = max(1, total // max(1, args.amostras))
    frames = falsos = salvos = n_placas_cena = 0
    tempos, candidatos = [], []
    video = None
    try:
        for passo in range(args.passos):
            obs, _, terminou, truncou, _ = env.step([0.0, 0.0])
            if passo % max(1, args.cada) == 0:
                frame = frame_bgr(obs)
                t0 = time.perf_counter()
                deteccoes, _ = detectar_formas(frame)
                tempos.append((time.perf_counter() - t0) * 1000.0)
                frames += 1
                todas = placas_na_imagem(env, placas)
                gabarito = [p for p in todas if p['inteira'] and p['raio'] >= raio_min]
                pares, sobras = conferir(deteccoes, gabarito)
                # deteccao sobre uma placa pequena demais ou cortada pela borda nao e falso
                # positivo: so conta o que nao esta em cima de placa alguma
                falsos += sum(1 for d in sobras if not any(
                    np.hypot(d['u'] - p['u'], d['v'] - p['v']) <= max(1.5 * p['raio'], 8.0)
                    for p in todas))
                certas = 0
                for placa, deteccao in pares:
                    linha = tabela[placa['forma']]
                    linha['visiveis'] += 1
                    escala_raio = placa['raio'] * 640.0 / args.largura
                    chave = [k for k, f in zip(por_raio, faixas_raio)
                             if f[0] <= escala_raio < f[1]]
                    if chave:
                        por_raio[chave[0]]['visiveis'] += 1
                    if deteccao is None:
                        continue
                    linha['detectadas'] += 1
                    linha['achado'][deteccao['forma']] = \
                        linha['achado'].get(deteccao['forma'], 0) + 1
                    if deteccao['forma'] == placa['forma']:
                        linha['forma_certa'] += 1
                        certas += 1
                        if chave:
                            por_raio[chave[0]]['forma_certa'] += 1
                imagem = desenhar(frame, deteccoes)
                if certas >= 2 and len(candidatos) < 40:
                    candidatos.append((certas, frame, deteccoes))
                if frames % salvar_a_cada == 0 and salvos < args.amostras:
                    salvos += 1
                    cv2.imwrite(os.path.join(pasta, 'formas_%02d.jpg' % salvos), imagem,
                                [cv2.IMWRITE_JPEG_QUALITY, 92])
                if args.video:
                    if video is None:
                        video = cv2.VideoWriter(os.path.join(pasta, 'formas.mp4'),
                                                cv2.VideoWriter_fourcc(*'mp4v'),
                                                10.0 / max(1, args.cada),
                                                (imagem.shape[1], imagem.shape[0]))
                    video.write(imagem)
                if args.mostrar:
                    cv2.imshow('formas (q sai)', imagem)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            if terminou or truncou:
                n_placas_cena += len(placas)
                remover_placas(placas)
                semente = args.semente + (semente - args.semente + 1) % max(1, args.cenarios)
                obs, _ = env.reset(seed=semente)
                placas = instalar_placas(env)
    finally:
        n_placas_cena += len(placas)
        remover_placas(placas)
        env.close()
        if video is not None:
            video.release()
        if args.mostrar:
            cv2.destroyAllWindows()

    if candidatos:
        candidatos.sort(key=lambda c: -c[0])
        escolhidos = candidatos[:12]
        escolhidos = [escolhidos[int(i)] for i in
                      np.linspace(0, len(escolhidos) - 1, min(3, len(escolhidos)))]
        cv2.imwrite(os.path.join(pasta, 'formas_etapas.jpg'),
                    np.vstack([paineis_das_etapas(f, d) for _, f, d in escolhidos]),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
    visiveis = sum(linha['visiveis'] for linha in tabela.values())
    certas = sum(linha['forma_certa'] for linha in tabela.values())
    for linha in tabela.values():
        linha['taxa_forma_certa'] = round(linha['forma_certa'] / max(1, linha['visiveis']), 4)
    for linha in por_raio.values():
        linha['taxa_forma_certa'] = round(linha['forma_certa'] / max(1, linha['visiveis']), 4)
    resumo = {
        'mapa': args.mapa, 'semente': args.semente, 'faixa_inicial': args.faixa,
        'resolucao': [args.largura, args.altura], 'passos': args.passos, 'cada': args.cada,
        'frames_processados': frames, 'placas_na_cena': n_placas_cena,
        'raio_min_px': args.raio_min,
        'placas_visiveis': visiveis, 'forma_certa': certas,
        'taxa_forma_certa': round(certas / max(1, visiveis), 4),
        'falsos_positivos': falsos,
        'falsos_positivos_por_frame': round(falsos / max(1, frames), 4),
        'por_forma': tabela, 'por_raio_aparente': por_raio,
        'ms_por_frame': round(float(np.mean(tempos)), 2) if tempos else None,
        'parametros': PARAMETROS,
    }
    with open(os.path.join(pasta, 'formas_resumo.json'), 'w', encoding='utf-8') as arquivo:
        json.dump(resumo, arquivo, indent=2, ensure_ascii=False)
    print('[formas] %d frames, %d placas na cena; %d placas visiveis (raio >= %.0f px): forma '
          'certa em %d (%.1f%%); %d falsos positivos'
          % (frames, n_placas_cena, visiveis, args.raio_min, certas,
             100.0 * resumo['taxa_forma_certa'], falsos))
    for forma in formas:
        linha = tabela[forma]
        print('[formas]   %-10s visiveis %4d | detectadas %4d | forma certa %4d (%.1f%%) | %s'
              % (forma, linha['visiveis'], linha['detectadas'], linha['forma_certa'],
                 100.0 * linha['taxa_forma_certa'], linha['achado']))
    print('[formas] por raio aparente: '
          + ', '.join('%s %d/%d' % (k, v['forma_certa'], v['visiveis'])
                      for k, v in por_raio.items()))
    print('[formas] %.1f ms por frame; saidas em %s' % (resumo['ms_por_frame'] or 0.0, pasta))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
