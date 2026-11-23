"""
Ambiente do pacote: raiz do workspace, pasta de saida e venv das dependencias.

O MetaDrive e o TensorFlow ficam num venv proprio, <ws>/.venv_tp4 (requirements_tp4_extras.txt,
criado pelo scripts/setup.sh), separado do venv do YOLO. Os executaveis rodam com o Python do
sistema (`ros2 run tp4_extras ...`): ao importar o pacote, o site-packages desse venv entra na
frente do sys.path. Com o Python do proprio venv (`.venv_tp4/bin/python -m tp4_extras...`) nada
muda.

<ws> e a pasta que contem src/tp4_extras, achada subindo a partir deste arquivo (vale para a
instalacao do colcon, com ou sem symlink, e para o pacote rodado direto de src); a variavel
TP4_EXTRAS_WS troca essa pasta. As saidas vao para <ws>/log/tp4_extras/<etapa>.
"""

import glob
import os
import sys

VENV = '.venv_tp4'


def pasta_ws():
    """Devolve a raiz do workspace (a pasta que contem src/tp4_extras)."""
    forcada = os.environ.get('TP4_EXTRAS_WS')
    if forcada:
        return os.path.abspath(os.path.expanduser(forcada))
    for inicio in (os.path.realpath(__file__), os.path.abspath(__file__)):
        pasta = os.path.dirname(inicio)
        for _ in range(8):
            pasta = os.path.dirname(pasta)
            if os.path.isdir(os.path.join(pasta, 'src', 'tp4_extras')):
                return pasta
    return os.getcwd()


def pasta_saida(etapa='', base=None):
    """Cria e devolve a pasta de saida da etapa (<ws>/log/tp4_extras/<etapa> por padrao)."""
    raiz = os.path.abspath(os.path.expanduser(base)) if base else \
        os.path.join(pasta_ws(), 'log', 'tp4_extras')
    pasta = os.path.join(raiz, etapa) if etapa else raiz
    os.makedirs(pasta, exist_ok=True)
    return pasta


def ativar_venv():
    """
    Poe o site-packages de <ws>/.venv_tp4 na frente do sys.path, se ele existir.

    Na frente mesmo quando ja esta no sys.path: com o `source ros2_ws/ambiente.sh` carregado, o
    PYTHONPATH traz o venv do YOLO (numpy 1.26, OpenCV 4.11), que entraria antes do numpy 2 e do
    OpenCV deste venv.
    """
    padrao = os.path.join(pasta_ws(), VENV, 'lib', 'python%d.%d' % sys.version_info[:2],
                          'site-packages')
    if not os.path.isdir(padrao):
        return None
    for pasta in (padrao, os.path.realpath(padrao)):
        while pasta in sys.path:
            sys.path.remove(pasta)
    sys.path.insert(0, padrao)
    return padrao


def pastas_cuda():
    """Lista as pastas lib das bibliotecas CUDA instaladas pelo pip (nvidia-*-cu12)."""
    pastas = []
    for base in sys.path:
        for lib in sorted(glob.glob(os.path.join(base, 'nvidia', '*', 'lib'))):
            if lib not in pastas:
                pastas.append(lib)
    return pastas


def expor_cuda():
    """
    Poe as bibliotecas CUDA do pip no LD_LIBRARY_PATH e reinicia o processo, uma unica vez.

    O TensorFlow abre libcudart, libcudnn etc. pelo nome; instaladas pelo pip (tensorflow com o
    extra and-cuda), elas ficam em site-packages/nvidia/*/lib, fora do caminho do carregador, e
    sem isso a GPU nao e registrada ("Cannot dlopen some GPU libraries"). O carregador so le o
    LD_LIBRARY_PATH na partida do processo, por isso a reexecucao. Sem essas bibliotecas (venv
    so de CPU) nada acontece.
    """
    if os.environ.get('TP4_EXTRAS_CUDA') == '1' or not sys.platform.startswith('linux'):
        return False
    pastas = pastas_cuda()
    if not pastas:
        return False
    atual = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(pastas + ([atual] if atual else []))
    os.environ['TP4_EXTRAS_CUDA'] = '1'
    argumentos = list(getattr(sys, 'orig_argv', None) or [sys.executable] + sys.argv)
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable] + argumentos[1:])
    return True
