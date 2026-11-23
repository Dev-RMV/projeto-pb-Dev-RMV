"""
Deteccao de faixas na camera sintetica do MetaDrive: mascara de cor em HSV, Canny e Hough.

Etapas, por frame (BGR):
  (a) conversao para HSV e mascara de cor: branco (linhas tracejadas e bordas) e amarelo (linha
      central), so dentro da regiao de interesse (trapezio da pista, abaixo do horizonte);
  (b) Canny sobre o frame em tons de cinza limitado a mascara: restam as bordas das linhas;
  (c) transformada de Hough probabilistica (cv2.HoughLinesP): segmentos de reta, agrupados em
      linhas de faixa (um ajuste x = a*y + b por grupo). As duas linhas mais proximas do centro
      da imagem, uma de cada lado, delimitam a faixa do veiculo.
O resultado e sobreposto ao frame: segmentos de Hough (vermelho), linhas ajustadas (ciano), as duas
linhas da faixa do veiculo (verde) e a area entre elas preenchida.

Conferencia: a geometria da pista no simulador da a posicao real das bordas da faixa do veiculo;
projetadas na imagem (8, 12 e 18 m a frente), elas medem o erro das duas linhas detectadas, em px.

  ros2 run tp4_extras detectar_faixas                    # 600 passos -> <ws>/log/tp4_extras/faixas
  ros2 run tp4_extras detectar_faixas --video            # grava tambem faixas.mp4
  ros2 run tp4_extras detectar_faixas --mostrar          # janela com o resultado ao vivo (q sai)
  ros2 run tp4_extras detectar_faixas --mapa SXS --trafego 0.2 --semente 7

Saidas: faixas_etapas.jpg (original | mascara HSV | Canny | Hough sobreposto), faixas_NN.jpg
(amostras do resultado) e faixas_resumo.json (taxas de deteccao, erro em px, tempo por frame).
"""

import argparse
import json
import os
import time

import cv2
import numpy as np

from tp4_extras.ambiente import pasta_saida
from tp4_extras.desenho import rotulo, tira

# Limiares HSV no espaco do OpenCV (H 0-179, S 0-255, V 0-255), medidos nos frames do MetaDrive:
# linha branca V ~ 200 e S < 15; asfalto V 145-170; linha amarela H ~ 28, S ~ 255, V ~ 200; a grama
# tem o mesmo matiz do amarelo, mas V < 150.
PARAMETROS = {
    'branco': ((0, 0, 185), (179, 60, 255)),
    'amarelo': ((18, 150, 150), (36, 255, 255)),
    # trapezio da pista em fracoes de (largura, altura), no sentido horario a partir de baixo
    'roi': ((0.0, 1.0), (0.0, 0.64), (0.38, 0.47), (0.62, 0.47), (1.0, 0.64), (1.0, 1.0)),
    'canny': (50, 150),
    # Hough probabilistica; limiar, comprimento e lacuna em px para 640 de largura (escalados)
    'hough': {'rho': 1.0, 'theta_graus': 1.0, 'limiar': 15, 'comprimento_min': 12,
              'lacuna_max': 20},
    'inclinacao_min': 0.2,      # |dy/dx| minimo: descarta segmentos quase horizontais
    'zona_proxima': 0.58,       # so entram nas linhas os segmentos que descem abaixo desta altura
    'linha_ref': 0.80,          # altura (fracao) em que os segmentos sao comparados
    'tolerancia': 0.05,         # distancia maxima (fracao da largura) dentro de um grupo
    'comprimento_grupo': 30,    # soma minima dos comprimentos de um grupo (px em 640)
}


def regiao_de_interesse(forma, par=PARAMETROS):
    """Devolve a mascara (uint8) do trapezio da pista para uma imagem de formato `forma`."""
    alt, larg = forma[:2]
    pontos = np.array([[int(x * (larg - 1)), int(y * (alt - 1))] for x, y in par['roi']], np.int32)
    roi = np.zeros((alt, larg), np.uint8)
    cv2.fillPoly(roi, [pontos], 255)
    return roi


def mascara_de_cor(frame, par=PARAMETROS):
    """Etapa (a): HSV + mascara. Devolve (mascara, branco, amarelo), ja restritas a regiao."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = regiao_de_interesse(frame.shape, par)
    branco = cv2.bitwise_and(cv2.inRange(hsv, np.array(par['branco'][0], np.uint8),
                                         np.array(par['branco'][1], np.uint8)), roi)
    amarelo = cv2.bitwise_and(cv2.inRange(hsv, np.array(par['amarelo'][0], np.uint8),
                                          np.array(par['amarelo'][1], np.uint8)), roi)
    # a linha amarela tem 1-2 px de largura: engrossa antes de procurar bordas
    amarelo = cv2.dilate(amarelo, np.ones((3, 3), np.uint8))
    return cv2.bitwise_or(branco, amarelo), branco, amarelo


def bordas_canny(frame, mascara, par=PARAMETROS):
    """Etapa (b): Canny no frame em cinza, mantendo so o que a mascara de cor selecionou."""
    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cinza = cv2.bitwise_and(cinza, mascara)
    cinza = cv2.GaussianBlur(cinza, (3, 3), 0)
    return cv2.Canny(cinza, par['canny'][0], par['canny'][1])


def segmentos_hough(bordas, par=PARAMETROS):
    """Etapa (c): cv2.HoughLinesP. Devolve um array (n, 4) de segmentos x1, y1, x2, y2."""
    escala = bordas.shape[1] / 640.0
    h = par['hough']
    linhas = cv2.HoughLinesP(bordas, h['rho'], np.radians(h['theta_graus']),
                             max(5, int(round(h['limiar'] * escala))),
                             minLineLength=max(4, int(round(h['comprimento_min'] * escala))),
                             maxLineGap=max(4, int(round(h['lacuna_max'] * escala))))
    if linhas is None:
        return np.zeros((0, 4), np.int32)
    return linhas.reshape(-1, 4).astype(np.int32)


def agrupar_em_linhas(segmentos, forma, par=PARAMETROS):
    """
    Agrupa os segmentos em linhas de faixa e ajusta uma reta x = a*y + b por grupo.

    Dois segmentos sao da mesma linha quando cruzam a altura de referencia (linha_ref) a menos de
    `tolerancia` um do outro. Devolve uma lista de dicts {a, b, x_base, y_topo, comprimento, n},
    ordenada da esquerda para a direita (x_base = x na ultima linha da imagem).
    """
    alt, larg = forma[:2]
    escala = larg / 640.0
    y_ref = par['linha_ref'] * alt
    itens = []
    for x1, y1, x2, y2 in segmentos.astype(float):
        dx, dy = x2 - x1, y2 - y1
        if abs(dy) < 1.0 or abs(dy) < par['inclinacao_min'] * abs(dx):
            continue
        if max(y1, y2) < par['zona_proxima'] * alt:
            continue                     # longe demais: nas curvas ja nao segue a reta de perto
        a = dx / dy
        x_ref = x1 + a * (y_ref - y1)
        if abs(x_ref - larg / 2.0) > 2.5 * larg:
            continue
        itens.append((x_ref, (x1, y1, x2, y2), float(np.hypot(dx, dy))))
    itens.sort(key=lambda item: item[0])
    grupos = []
    for item in itens:
        if grupos and item[0] - grupos[-1][-1][0] <= par['tolerancia'] * larg:
            grupos[-1].append(item)
        else:
            grupos.append([item])
    linhas = []
    for grupo in grupos:
        comprimento = sum(item[2] for item in grupo)
        if comprimento < par['comprimento_grupo'] * escala:
            continue
        xs, ys, pesos = [], [], []
        for _, (x1, y1, x2, y2), tamanho in grupo:
            xs += [x1, x2]
            ys += [y1, y2]
            pesos += [tamanho, tamanho]
        if max(ys) - min(ys) < 3.0:
            continue
        a, b = np.polyfit(ys, xs, 1, w=pesos)
        linhas.append({'a': float(a), 'b': float(b), 'x_base': float(a * (alt - 1) + b),
                       'y_topo': float(min(ys)), 'comprimento': float(comprimento),
                       'n': len(grupo)})
    linhas.sort(key=lambda linha: linha['x_base'])
    return linhas


def faixa_do_veiculo(linhas, forma):
    """Escolhe as linhas esquerda e direita da faixa do veiculo (as mais proximas do centro)."""
    alt, larg = forma[:2]
    centro = larg / 2.0
    esquerdas = [linha for linha in linhas if linha['x_base'] < centro]
    direitas = [linha for linha in linhas if linha['x_base'] >= centro]
    esq = max(esquerdas, key=lambda linha: linha['x_base']) if esquerdas else None
    dire = min(direitas, key=lambda linha: linha['x_base']) if direitas else None
    if esq and dire:
        # as duas tem de continuar na ordem esquerda/direita ate o topo da regiao
        y = max(esq['y_topo'], dire['y_topo'])
        if esq['a'] * y + esq['b'] >= dire['a'] * y + dire['b']:
            if abs(esq['x_base'] - centro) < abs(dire['x_base'] - centro):
                dire = None
            else:
                esq = None
    return esq, dire


def detectar_faixas(frame, par=PARAMETROS):
    """Roda as tres etapas num frame BGR e devolve um dict com as imagens e as linhas."""
    mascara, branco, amarelo = mascara_de_cor(frame, par)
    bordas = bordas_canny(frame, mascara, par)
    segmentos = segmentos_hough(bordas, par)
    linhas = agrupar_em_linhas(segmentos, frame.shape, par)
    esq, dire = faixa_do_veiculo(linhas, frame.shape)
    return {'mascara': mascara, 'branco': branco, 'amarelo': amarelo, 'bordas': bordas,
            'segmentos': segmentos, 'linhas': linhas, 'esquerda': esq, 'direita': dire}


def _ponto(linha, y):
    return int(round(linha['a'] * y + linha['b'])), int(round(y))


def sobrepor(frame, resultado, texto=True):
    """Desenha sobre o frame os segmentos de Hough, as linhas ajustadas e a faixa do veiculo."""
    alt, larg = frame.shape[:2]
    saida = frame.copy()
    esq, dire = resultado['esquerda'], resultado['direita']
    if esq and dire:
        topo = max(esq['y_topo'], dire['y_topo'])
        poligono = np.array([_ponto(esq, alt - 1), _ponto(esq, topo), _ponto(dire, topo),
                             _ponto(dire, alt - 1)], np.int32)
        camada = saida.copy()
        cv2.fillPoly(camada, [poligono], (0, 200, 0))
        saida = cv2.addWeighted(camada, 0.3, saida, 0.7, 0)
    espessura = max(1, int(round(larg / 320.0)))
    for x1, y1, x2, y2 in resultado['segmentos']:
        cv2.line(saida, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1, cv2.LINE_AA)
    for linha in resultado['linhas']:
        propria = linha is esq or linha is dire
        cor = (0, 255, 0) if propria else (255, 255, 0)
        ok, p1, p2 = cv2.clipLine((0, 0, larg, alt), _ponto(linha, alt - 1),
                                  _ponto(linha, linha['y_topo']))
        if ok:
            cv2.line(saida, p1, p2, cor, espessura + (1 if propria else 0), cv2.LINE_AA)
    if texto:
        estado = 'esq+dir' if esq and dire else 'esq' if esq else 'dir' if dire else 'nenhuma'
        rotulo(saida, 'linhas: %d | faixa do veiculo: %s' % (len(resultado['linhas']), estado),
               escala=larg / 1200.0)
    return saida


def paineis_das_etapas(frame, resultado, largura=480):
    """Monta a tira original | mascara HSV | Canny | Hough sobreposto de um frame."""
    mascara = np.zeros_like(frame)
    mascara[resultado['branco'] > 0] = (255, 255, 255)
    mascara[resultado['amarelo'] > 0] = (0, 215, 255)
    roi = regiao_de_interesse(frame.shape)
    contorno, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(mascara, contorno, -1, (90, 90, 90), 1)
    return tira([('camera sintetica', frame), ('(a) HSV + mascara de cor', mascara),
                 ('(b) Canny', resultado['bordas']),
                 ('(c) Hough + faixas sobrepostas', sobrepor(frame, resultado, False))], largura)


def erro_das_bordas(resultado, gabarito):
    """Devolve os erros (px) das linhas esquerda e direita contra o gabarito do simulador."""
    erros = {'esquerda': [], 'direita': []}
    for (u_esq, v_esq), (u_dir, v_dir) in gabarito:
        if resultado['esquerda']:
            linha = resultado['esquerda']
            erros['esquerda'].append(abs(linha['a'] * v_esq + linha['b'] - u_esq))
        if resultado['direita']:
            linha = resultado['direita']
            erros['direita'].append(abs(linha['a'] * v_dir + linha['b'] - u_dir))
    return erros


def _estatistica(valores):
    if not valores:
        return None
    v = np.asarray(valores, dtype=float)
    return {'n': int(v.size), 'mediana': round(float(np.median(v)), 2),
            'media': round(float(v.mean()), 2), 'p90': round(float(np.percentile(v, 90)), 2)}


def main(argv=None):
    from tp4_extras.simulador import (bloco_do_no, bordas_da_faixa, criar_ambiente, frame_bgr,
                                      situacao_cruzamento)

    ap = argparse.ArgumentParser(description='Deteccao de faixas (HSV, Canny, Hough) no MetaDrive')
    ap.add_argument('--passos', type=int, default=600, help='passos de simulacao (0,1 s cada)')
    ap.add_argument('--cada', type=int, default=2, help='processa um frame a cada N passos')
    ap.add_argument('--mapa', default='SCSCS', help='blocos do mapa (S reta, C curva, X, T, O)')
    ap.add_argument('--semente', type=int, default=0)
    ap.add_argument('--cenarios', type=int, default=5, help='cenarios (sementes) disponiveis')
    ap.add_argument('--trafego', type=float, default=0.08, help='densidade de trafego (0-1)')
    ap.add_argument('--largura', type=int, default=640)
    ap.add_argument('--altura', type=int, default=360)
    ap.add_argument('--amostras', type=int, default=8, help='frames com o resultado a salvar')
    ap.add_argument('--saida', default=None, help='pasta base (padrao <ws>/log/tp4_extras)')
    ap.add_argument('--video', action='store_true', help='grava faixas.mp4 com o resultado')
    ap.add_argument('--mostrar', action='store_true', help='janela ao vivo (precisa de DISPLAY)')
    args = ap.parse_args(argv)

    pasta = pasta_saida('faixas', args.saida)
    env = criar_ambiente(args.largura, args.altura, mapa=args.mapa, semente=args.semente,
                         cenarios=args.cenarios, trafego=args.trafego)
    semente = args.semente
    obs, _ = env.reset(seed=semente)
    total = max(1, args.passos // max(1, args.cada))
    salvar_a_cada = max(1, total // max(1, args.amostras))
    guardar_a_cada = max(1, total // 30)        # candidatos ao mosaico das etapas
    video = None
    contagem = {'frames': 0, 'na_via': 0, 'com_linha': 0, 'com_faixa': 0, 'no_cruzamento': 0}
    n_linhas, n_segmentos, tempos = [], [], []
    por_bloco = {}
    erros = {'esquerda': [], 'direita': []}
    candidatos, salvos = [], 0
    try:
        for passo in range(args.passos):
            obs, _, terminou, truncou, _ = env.step([0.0, 0.0])
            if passo % max(1, args.cada) == 0:
                frame = frame_bgr(obs)
                t0 = time.perf_counter()
                resultado = detectar_faixas(frame)
                tempos.append((time.perf_counter() - t0) * 1000.0)
                dentro = situacao_cruzamento(env)[0]
                contagem['frames'] += 1
                n_linhas.append(len(resultado['linhas']))
                n_segmentos.append(len(resultado['segmentos']))
                completa = bool(resultado['esquerda'] and resultado['direita'])
                if dentro:
                    contagem['no_cruzamento'] += 1
                else:
                    contagem['na_via'] += 1
                    contagem['com_linha'] += bool(resultado['linhas'])
                    contagem['com_faixa'] += completa
                    bloco = por_bloco.setdefault(bloco_do_no(env.agent.lane_index[1])[1],
                                                 {'frames': 0, 'com_faixa': 0})
                    bloco['frames'] += 1
                    bloco['com_faixa'] += completa
                    gabarito = bordas_da_faixa(env)
                    if gabarito:
                        e = erro_das_bordas(resultado, gabarito)
                        erros['esquerda'] += e['esquerda']
                        erros['direita'] += e['direita']
                imagem = sobrepor(frame, resultado)
                if completa and not dentro and contagem['com_faixa'] % guardar_a_cada == 0:
                    candidatos.append((passo, frame, resultado))
                if contagem['frames'] % salvar_a_cada == 0 and salvos < args.amostras:
                    salvos += 1
                    cv2.imwrite(os.path.join(pasta, 'faixas_%02d.jpg' % salvos), imagem,
                                [cv2.IMWRITE_JPEG_QUALITY, 92])
                if args.video:
                    if video is None:
                        video = cv2.VideoWriter(os.path.join(pasta, 'faixas.mp4'),
                                                cv2.VideoWriter_fourcc(*'mp4v'),
                                                10.0 / max(1, args.cada),
                                                (imagem.shape[1], imagem.shape[0]))
                    video.write(imagem)
                if args.mostrar:
                    cv2.imshow('faixas (q sai)', imagem)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            if terminou or truncou:
                semente = args.semente + (semente - args.semente + 1) % max(1, args.cenarios)
                obs, _ = env.reset(seed=semente)
    finally:
        env.close()
        if video is not None:
            video.release()
        if args.mostrar:
            cv2.destroyAllWindows()

    if candidatos:
        escolhidos = [candidatos[int(i)] for i in
                      np.linspace(0, len(candidatos) - 1, min(3, len(candidatos)))]
        tiras = [paineis_das_etapas(frame, resultado) for _, frame, resultado in escolhidos]
        cv2.imwrite(os.path.join(pasta, 'faixas_etapas.jpg'), np.vstack(tiras),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
    na_via = max(1, contagem['na_via'])
    resumo = {
        'mapa': args.mapa, 'semente': args.semente, 'trafego': args.trafego,
        'resolucao': [args.largura, args.altura], 'passos': args.passos, 'cada': args.cada,
        'frames_processados': contagem['frames'],
        'frames_dentro_de_cruzamento': contagem['no_cruzamento'],
        'frames_na_via': contagem['na_via'],
        'na_via_com_alguma_linha': contagem['com_linha'],
        'na_via_com_as_duas_linhas_da_faixa': contagem['com_faixa'],
        'taxa_alguma_linha': round(contagem['com_linha'] / na_via, 4),
        'taxa_faixa_do_veiculo': round(contagem['com_faixa'] / na_via, 4),
        'faixa_do_veiculo_por_bloco': {
            letra: dict(b, taxa=round(b['com_faixa'] / max(1, b['frames']), 4))
            for letra, b in sorted(por_bloco.items())},
        'linhas_por_frame': round(float(np.mean(n_linhas)), 2) if n_linhas else 0.0,
        'segmentos_hough_por_frame': round(float(np.mean(n_segmentos)), 1) if n_segmentos else 0.0,
        'ms_por_frame': round(float(np.mean(tempos)), 2) if tempos else None,
        'erro_px_borda_esquerda': _estatistica(erros['esquerda']),
        'erro_px_borda_direita': _estatistica(erros['direita']),
        'parametros': {k: v for k, v in PARAMETROS.items()},
    }
    with open(os.path.join(pasta, 'faixas_resumo.json'), 'w', encoding='utf-8') as arquivo:
        json.dump(resumo, arquivo, indent=2, ensure_ascii=False)
    print('[faixas] %d frames (%d na via, %d dentro de cruzamento)'
          % (contagem['frames'], contagem['na_via'], contagem['no_cruzamento']))
    print('[faixas] na via: alguma linha em %.1f%%, as duas linhas da faixa do veiculo em %.1f%%'
          % (100.0 * resumo['taxa_alguma_linha'], 100.0 * resumo['taxa_faixa_do_veiculo']))
    print('[faixas] faixa do veiculo por bloco (I inicio, S reta, C curva): '
          + ', '.join('%s %d/%d' % (letra, b['com_faixa'], b['frames'])
                      for letra, b in sorted(por_bloco.items())))
    for lado in ('esquerda', 'direita'):
        e = resumo['erro_px_borda_' + lado]
        if e:
            print('[faixas] erro da borda %s contra o simulador: mediana %.1f px, p90 %.1f px '
                  '(%d medidas)' % (lado, e['mediana'], e['p90'], e['n']))
    print('[faixas] %.1f ms por frame; saidas em %s' % (resumo['ms_por_frame'] or 0.0, pasta))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
