"""
CNN (TensorFlow/Keras) que classifica o estado da via nos frames do MetaDrive.

Classes: via_livre, obstaculo e cruzamento (dataset do `coletar_dataset`). A mesma rede e treinada
duas vezes, com os mesmos pesos iniciais, os mesmos dados e a mesma ordem de lotes: uma vez SEM e
outra COM data augmentation (rotacao de +-15 graus, zoom de 0,2 e flip horizontal). As curvas de
accuracy e de loss das duas ficam lado a lado, e o resumo mede a generalizacao de tres jeitos:
diferenca entre a accuracy de treino e a de validacao; accuracy e loss no teste (mapas que a rede
nao viu nem no treino nem na escolha da epoca); e accuracy no teste PERTURBADO, os mesmos frames
de teste com rotacao, zoom e flip sorteados uma unica vez (semente fixa), que mede se a rede
aguenta uma camera inclinada ou com outro enquadramento.

Rede: 4 blocos Conv2D 3x3 -> BatchNormalization -> ReLU -> MaxPooling2D (32, 64, 128 e 128
filtros), Flatten, Dense(128), Dropout(0,5) e Dense(3, softmax); entrada 90x160 RGB; Adam com
taxa 3e-4, lote de 32. A augmentation fica dentro do modelo (camadas RandomFlip, RandomRotation e
RandomZoom), ativa so no treino. Os pesos guardados e avaliados sao os da epoca de maior accuracy
de validacao (no empate, a de menor loss de validacao).

Dispositivo: `--dispositivo auto` (padrao) usa a GPU se o TensorFlow a encontrar e a CPU caso
contrario; `gpu` exige a GPU; `cpu` ignora a GPU. A GPU depende do extra and-cuda do TensorFlow
(scripts/setup.sh o instala quando ha placa NVIDIA).

  ros2 run tp4_extras treinar_cnn                       # 60 epocas por variante
  ros2 run tp4_extras treinar_cnn --dispositivo cpu --epocas 20
  ros2 run tp4_extras treinar_cnn --dataset outra/pasta/dataset --saida outra/pasta

Saidas em <ws>/log/tp4_extras/cnn: cnn_curvas.png, cnn_resumo.json, cnn_resumo.txt,
cnn_historico.csv, cnn_teste_perturbado.jpg (amostras do teste perturbado) e os modelos
cnn_sem_augmentation.keras e cnn_com_augmentation.keras.
"""

import argparse
import csv
import json
import os
import time

import numpy as np

from tp4_extras.ambiente import expor_cuda, pasta_saida, pasta_ws

VARIANTES = (('sem_augmentation', False), ('com_augmentation', True))
ROTACAO_GRAUS = 15.0
ZOOM = 0.2
# Com 350 frames sao so 11 lotes por epoca: com o momento padrao do Keras (0,99) as medias moveis
# do BatchNormalization levam centenas de lotes para assentar e, ate la, a validacao (que usa
# essas medias) fica presa no acaso mesmo com o treino ja em 90%.
MOMENTO_BN = 0.9
ACURACIA = 'acurácia'
VALIDACAO = 'validação'
EPOCA = 'época'


def preparar_dispositivo(escolha):
    """Importa o TensorFlow ja com o dispositivo escolhido; devolve (tf, descricao, usa_gpu)."""
    if escolha == 'cpu':
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
    else:
        expor_cuda()                     # reinicia o processo uma vez, se houver CUDA do pip
    os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '2')
    import tensorflow as tf

    gpus = [] if escolha == 'cpu' else tf.config.list_physical_devices('GPU')
    if gpus:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        detalhes = tf.config.experimental.get_device_details(gpus[0])
        return tf, 'GPU (%s)' % detalhes.get('device_name', gpus[0].name), True
    if escolha == 'gpu':
        raise SystemExit('[cnn] --dispositivo gpu, mas o TensorFlow nao encontrou GPU (falta o '
                         'driver NVIDIA ou o extra and-cuda no venv). Use --dispositivo auto.')
    motivo = 'pedida' if escolha == 'cpu' else 'nenhuma GPU encontrada pelo TensorFlow'
    return tf, 'CPU (%s)' % motivo, False


def carregar_dataset(pasta, altura, largura):
    """Le o manifesto e as imagens; devolve ({particao: (x uint8 RGB, y int)}, classes)."""
    import cv2

    manifesto = os.path.join(pasta, 'manifesto.csv')
    if not os.path.isfile(manifesto):
        raise SystemExit('[cnn] dataset nao encontrado em %s: rode antes `coletar_dataset`'
                         % pasta)
    with open(manifesto, newline='', encoding='utf-8') as arquivo:
        linhas = list(csv.DictReader(arquivo))
    classes = [None] * (1 + max(int(linha['rotulo']) for linha in linhas))
    dados = {}
    for linha in linhas:
        classes[int(linha['rotulo'])] = linha['classe']
        imagem = cv2.imread(os.path.join(pasta, linha['arquivo']))
        imagem = cv2.resize(imagem, (largura, altura), interpolation=cv2.INTER_AREA)
        x, y = dados.setdefault(linha['particao'], ([], []))
        x.append(cv2.cvtColor(imagem, cv2.COLOR_BGR2RGB))
        y.append(int(linha['rotulo']))
    return {nome: (np.stack(x), np.array(y, np.int32)) for nome, (x, y) in dados.items()}, classes


def criar_modelo(tf, formato, n_classes, aumentar, taxa, momento_bn=MOMENTO_BN):
    """Monta e compila a CNN; com `aumentar`, a augmentation vem logo apos a entrada."""
    camadas = tf.keras.layers
    entrada = tf.keras.Input(shape=formato)
    x = entrada
    if aumentar:
        x = camadas.RandomFlip('horizontal')(x)
        x = camadas.RandomRotation(ROTACAO_GRAUS / 360.0)(x)     # fracao de volta: +-15 graus
        x = camadas.RandomZoom(ZOOM)(x)
    x = camadas.Rescaling(1.0 / 255.0)(x)
    for filtros in (32, 64, 128, 128):
        x = camadas.Conv2D(filtros, 3, padding='same', use_bias=False)(x)
        x = camadas.BatchNormalization(momentum=momento_bn)(x)
        x = camadas.Activation('relu')(x)
        x = camadas.MaxPooling2D()(x)
    x = camadas.Flatten()(x)
    x = camadas.Dense(128, activation='relu')(x)
    x = camadas.Dropout(0.5)(x)
    saida = camadas.Dense(n_classes, activation='softmax')(x)
    modelo = tf.keras.Model(entrada, saida)
    modelo.compile(optimizer=tf.keras.optimizers.Adam(taxa),
                   loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return modelo


def perturbar(tf, imagens, semente):
    """
    Aplica rotacao, zoom e flip sorteados (os da augmentation) a um conjunto de imagens.

    O sorteio e feito uma unica vez, com semente fixa: as duas variantes sao avaliadas nas mesmas
    imagens perturbadas. Devolve um array uint8 com o mesmo formato da entrada.
    """
    camadas = tf.keras.layers
    transformacao = tf.keras.Sequential([
        camadas.RandomFlip('horizontal', seed=semente),
        camadas.RandomRotation(ROTACAO_GRAUS / 360.0, seed=semente + 1),
        camadas.RandomZoom(ZOOM, seed=semente + 2)])
    saida = np.asarray(transformacao(imagens.astype('float32'), training=True))
    return np.clip(np.rint(saida), 0, 255).astype(np.uint8)


def treinar_variante(tf, nome, aumentar, dados, n_classes, pesos_iniciais, args, pasta):
    """Treina uma variante e devolve (historico por epoca, metricas com os melhores pesos)."""
    x_treino, y_treino = dados['treino']
    x_val, y_val = dados['validacao']
    x_teste, y_teste = dados['teste']
    tf.keras.utils.set_random_seed(args.semente)
    modelo = criar_modelo(tf, x_treino.shape[1:], n_classes, aumentar, args.taxa,
                          args.momento_bn)
    modelo.set_weights(pesos_iniciais)          # mesma partida nas duas variantes
    melhor = {'nota': (-1.0, 0.0), 'pesos': None, 'epoca': 0}

    class GuardarMelhor(tf.keras.callbacks.Callback):

        def on_epoch_end(self, epoch, logs=None):
            nota = (float(logs['val_accuracy']), -float(logs['val_loss']))
            if nota > melhor['nota']:
                melhor.update(nota=nota, pesos=self.model.get_weights(), epoca=epoch + 1)

    print('[cnn] ===== %s: %d epocas, lote %d =====' % (nome, args.epocas, args.lote), flush=True)
    t0 = time.time()
    historico = modelo.fit(x_treino, y_treino, validation_data=(x_val, y_val),
                           epochs=args.epocas, batch_size=args.lote, shuffle=True, verbose=2,
                           callbacks=[GuardarMelhor()]).history
    tempo = time.time() - t0
    modelo.set_weights(melhor['pesos'])
    medidas = {}
    for particao, (x, y) in (('treino', (x_treino, y_treino)), ('validacao', (x_val, y_val)),
                             ('teste', (x_teste, y_teste)),
                             ('teste_perturbado', dados['teste_perturbado'])):
        loss, acuracia = modelo.evaluate(x, y, batch_size=args.lote, verbose=0)
        medidas[particao] = {'accuracy': round(float(acuracia), 4), 'loss': round(float(loss), 4)}
    previsto = np.argmax(modelo.predict(x_teste, batch_size=args.lote, verbose=0), axis=1)
    confusao = np.zeros((n_classes, n_classes), int)
    for real, pred in zip(y_teste, previsto):
        confusao[real, pred] += 1
    modelo.save(os.path.join(pasta, 'cnn_%s.keras' % nome))
    ultimas = slice(-5, None)
    metricas = {
        'melhor_epoca': melhor['epoca'], 'tempo_treino_s': round(tempo, 1),
        'parametros': int(modelo.count_params()),
        'avaliacao_melhor_epoca': medidas,
        'gap_treino_validacao': round(medidas['treino']['accuracy']
                                      - medidas['validacao']['accuracy'], 4),
        'gap_treino_teste': round(medidas['treino']['accuracy'] - medidas['teste']['accuracy'], 4),
        'curvas_media_5_ultimas_epocas': {
            chave: round(float(np.mean(historico[chave][ultimas])), 4)
            for chave in ('accuracy', 'val_accuracy', 'loss', 'val_loss')},
        'menor_val_loss': round(float(min(historico['val_loss'])), 4),
        'maior_val_accuracy': round(float(max(historico['val_accuracy'])), 4),
        'matriz_de_confusao_teste': confusao.tolist(),
    }
    return historico, metricas


def plotar_curvas(historicos, arquivo, dispositivo):
    """Grava a figura 2x3: accuracy e loss de cada variante e a comparacao na validacao."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    titulos = {'sem_augmentation': 'sem augmentation', 'com_augmentation': 'com augmentation'}
    cores = {'sem_augmentation': 'tab:red', 'com_augmentation': 'tab:blue'}
    figura, eixos = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    for linha, (metrica, nome) in enumerate((('accuracy', ACURACIA), ('loss', 'loss'))):
        teto = max(max(h[metrica] + h['val_' + metrica]) for h in historicos.values())
        for coluna, variante in enumerate(titulos):
            h = historicos[variante]
            epocas = range(1, len(h[metrica]) + 1)
            eixo = eixos[linha][coluna]
            eixo.plot(epocas, h[metrica], color='tab:gray', label='treino')
            eixo.plot(epocas, h['val_' + metrica], color=cores[variante], label=VALIDACAO)
            eixo.set_title('%s: %s' % (nome, titulos[variante]))
        eixo = eixos[linha][2]
        for variante in titulos:
            h = historicos[variante]
            eixo.plot(range(1, len(h[metrica]) + 1), h['val_' + metrica], color=cores[variante],
                      label=titulos[variante])
        eixo.set_title('%s de %s: sem x com augmentation' % (nome, VALIDACAO))
        for eixo in eixos[linha]:
            eixo.set_ylim(0.0, 1.02 if metrica == 'accuracy' else min(teto * 1.05, 3.0))
            eixo.set_ylabel(nome)
            eixo.grid(alpha=0.3)
            eixo.legend(loc='lower right' if metrica == 'accuracy' else 'upper right')
    for eixo in eixos[1]:
        eixo.set_xlabel(EPOCA)
    figura.suptitle('CNN de estados da via (MetaDrive): augmentation = rotação '
                    '±15°, zoom 0,2, flip horizontal; treino em %s' % dispositivo)
    figura.tight_layout()
    figura.savefig(arquivo, dpi=110)
    plt.close(figura)


def texto_do_resumo(resumo):
    """Monta o relato em texto da comparacao entre as duas variantes."""
    sem = resumo['variantes']['sem_augmentation']
    com = resumo['variantes']['com_augmentation']
    comp = resumo['comparacao']

    def linha(nome, v):
        m = v['avaliacao_melhor_epoca']
        return ('  %-17s epoca %2d | accuracy treino %.3f, validacao %.3f, teste %.3f, teste '
                'perturbado %.3f | loss validacao %.3f, teste %.3f | gap treino-validacao %+.3f, '
                'treino-teste %+.3f | %.0f s'
                % (nome, v['melhor_epoca'], m['treino']['accuracy'], m['validacao']['accuracy'],
                   m['teste']['accuracy'], m['teste_perturbado']['accuracy'],
                   m['validacao']['loss'], m['teste']['loss'], v['gap_treino_validacao'],
                   v['gap_treino_teste'], v['tempo_treino_s']))

    partes = [
        'CNN de estados da via (MetaDrive) - com e sem data augmentation',
        'dataset: %s (treino %d, validacao %d, teste %d frames; classes %s)'
        % (resumo['dataset'], resumo['frames']['treino'], resumo['frames']['validacao'],
           resumo['frames']['teste'], ', '.join(resumo['classes'])),
        'rede: %d parametros; entrada %dx%d; %d epocas; lote %d; Adam %g; dispositivo: %s; '
        'TensorFlow %s'
        % (sem['parametros'], resumo['entrada'][0], resumo['entrada'][1], resumo['epocas'],
           resumo['lote'], resumo['taxa_de_aprendizado'], resumo['dispositivo'],
           resumo['tensorflow']),
        'augmentation: rotacao +-%g graus, zoom %g, flip horizontal' % (ROTACAO_GRAUS, ZOOM),
        '',
        'avaliacao com os pesos da epoca de maior accuracy de validacao:',
        linha('sem augmentation', sem),
        linha('com augmentation', com),
        '',
        'media das 5 ultimas epocas (curvas): accuracy de validacao %.3f sem e %.3f com; loss de '
        'validacao %.3f sem e %.3f com'
        % (sem['curvas_media_5_ultimas_epocas']['val_accuracy'],
           com['curvas_media_5_ultimas_epocas']['val_accuracy'],
           sem['curvas_media_5_ultimas_epocas']['val_loss'],
           com['curvas_media_5_ultimas_epocas']['val_loss']),
        '',
        'efeito da augmentation (com - sem): accuracy no teste %+.3f; accuracy no teste '
        'perturbado %+.3f; accuracy na validacao %+.3f; loss no teste %+.3f; gap '
        'treino-validacao %+.3f; gap treino-teste %+.3f; loss de validacao no fim do treino %+.3f'
        % (comp['accuracy_teste'], comp['accuracy_teste_perturbado'],
           comp['accuracy_validacao'], comp['loss_teste'], comp['gap_treino_validacao'],
           comp['gap_treino_teste'], comp['val_loss_fim_do_treino']),
        'teste perturbado = os frames de teste com rotacao de +-%g graus, zoom de %g e flip '
        'horizontal sorteados uma vez (semente %d)'
        % (ROTACAO_GRAUS, ZOOM, resumo['semente_do_teste_perturbado']),
        'matriz de confusao no teste (linhas = classe real, colunas = prevista; %s):'
        % ', '.join(resumo['classes']),
        '  sem augmentation: %s' % sem['matriz_de_confusao_teste'],
        '  com augmentation: %s' % com['matriz_de_confusao_teste'],
    ]
    return '\n'.join(partes) + '\n'


def main(argv=None):
    import cv2

    from tp4_extras.desenho import grade

    ap = argparse.ArgumentParser(description='CNN de estados da via, com e sem augmentation')
    ap.add_argument('--dataset', default=None,
                    help='pasta do dataset (padrao <ws>/log/tp4_extras/dataset)')
    ap.add_argument('--saida', default=None, help='pasta base (padrao <ws>/log/tp4_extras)')
    ap.add_argument('--dispositivo', choices=('auto', 'gpu', 'cpu'), default='auto',
                    help='auto = GPU se o TensorFlow a encontrar, senao CPU')
    ap.add_argument('--epocas', type=int, default=60)
    ap.add_argument('--lote', type=int, default=32)
    ap.add_argument('--taxa', type=float, default=3e-4, help='taxa de aprendizado do Adam')
    ap.add_argument('--momento_bn', type=float, default=MOMENTO_BN,
                    help='momento das medias moveis do BatchNormalization')
    ap.add_argument('--altura', type=int, default=90, help='altura da entrada da rede (px)')
    ap.add_argument('--largura', type=int, default=160, help='largura da entrada da rede (px)')
    ap.add_argument('--semente', type=int, default=42)
    args = ap.parse_args(argv)

    tf, dispositivo, usa_gpu = preparar_dispositivo(args.dispositivo)
    print('[cnn] TensorFlow %s, dispositivo: %s' % (tf.__version__, dispositivo), flush=True)
    pasta_dataset = os.path.abspath(os.path.expanduser(args.dataset)) if args.dataset else \
        os.path.join(pasta_ws(), 'log', 'tp4_extras', 'dataset')
    pasta = pasta_saida('cnn', args.saida)
    dados, classes = carregar_dataset(pasta_dataset, args.altura, args.largura)
    for particao in ('treino', 'validacao', 'teste'):
        if particao not in dados:
            raise SystemExit('[cnn] o dataset nao tem a particao %s' % particao)
    print('[cnn] dataset %s: %s' % (pasta_dataset, ', '.join(
        '%s %d' % (p, len(dados[p][1])) for p in ('treino', 'validacao', 'teste'))), flush=True)

    semente_perturbacao = args.semente + 1000
    dados['teste_perturbado'] = (perturbar(tf, dados['teste'][0], semente_perturbacao),
                                 dados['teste'][1])
    amostras = [cv2.cvtColor(imagem, cv2.COLOR_RGB2BGR)
                for imagem in dados['teste_perturbado'][0][::max(1, len(dados['teste'][1]) // 12)]]
    cv2.imwrite(os.path.join(pasta, 'cnn_teste_perturbado.jpg'), grade(amostras[:12], 6, 240),
                [cv2.IMWRITE_JPEG_QUALITY, 90])

    tf.keras.utils.set_random_seed(args.semente)
    pesos_iniciais = criar_modelo(tf, dados['treino'][0].shape[1:], len(classes), False,
                                  args.taxa, args.momento_bn).get_weights()
    historicos, variantes = {}, {}
    for nome, aumentar in VARIANTES:
        historicos[nome], variantes[nome] = treinar_variante(
            tf, nome, aumentar, dados, len(classes), pesos_iniciais, args, pasta)

    plotar_curvas(historicos, os.path.join(pasta, 'cnn_curvas.png'), dispositivo)
    with open(os.path.join(pasta, 'cnn_historico.csv'), 'w', newline='', encoding='utf-8') as arq:
        escritor = csv.writer(arq)
        escritor.writerow(['variante', 'epoca', 'accuracy', 'val_accuracy', 'loss', 'val_loss'])
        for nome, h in historicos.items():
            for i in range(len(h['loss'])):
                escritor.writerow([nome, i + 1] + [
                    '%.4f' % h[c][i] for c in ('accuracy', 'val_accuracy', 'loss', 'val_loss')])
    sem, com = variantes['sem_augmentation'], variantes['com_augmentation']

    def diferenca(caminho):
        a, b = com, sem
        for chave in caminho:
            a, b = a[chave], b[chave]
        return round(a - b, 4)

    resumo = {
        'dataset': pasta_dataset, 'classes': classes,
        'frames': {p: int(len(dados[p][1])) for p in ('treino', 'validacao', 'teste')},
        'entrada': [args.altura, args.largura], 'epocas': args.epocas, 'lote': args.lote,
        'taxa_de_aprendizado': args.taxa, 'semente': args.semente,
        'semente_do_teste_perturbado': semente_perturbacao,
        'dispositivo': dispositivo, 'gpu': usa_gpu, 'tensorflow': tf.__version__,
        'augmentation': {'rotacao_graus': ROTACAO_GRAUS, 'zoom': ZOOM, 'flip': 'horizontal'},
        'variantes': variantes,
        'comparacao': {
            'accuracy_teste': diferenca(('avaliacao_melhor_epoca', 'teste', 'accuracy')),
            'accuracy_teste_perturbado': diferenca(
                ('avaliacao_melhor_epoca', 'teste_perturbado', 'accuracy')),
            'gap_treino_teste': diferenca(('gap_treino_teste',)),
            'accuracy_validacao': diferenca(('avaliacao_melhor_epoca', 'validacao', 'accuracy')),
            'loss_teste': diferenca(('avaliacao_melhor_epoca', 'teste', 'loss')),
            'gap_treino_validacao': diferenca(('gap_treino_validacao',)),
            'val_loss_fim_do_treino': diferenca(('curvas_media_5_ultimas_epocas', 'val_loss')),
        },
    }
    with open(os.path.join(pasta, 'cnn_resumo.json'), 'w', encoding='utf-8') as arq:
        json.dump(resumo, arq, indent=2, ensure_ascii=False)
    texto = texto_do_resumo(resumo)
    with open(os.path.join(pasta, 'cnn_resumo.txt'), 'w', encoding='utf-8') as arq:
        arq.write(texto)
    print()
    print(texto)
    print('[cnn] saidas em %s' % pasta)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
