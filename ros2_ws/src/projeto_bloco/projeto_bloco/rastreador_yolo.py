"""
Rastreamento de objetos com YOLO (ultralytics) + ByteTrack + filtro de Kalman.

Assina /camera/image_raw e guarda so o frame mais recente. Um timer a `frequencia_hz` roda
model.track() nesse frame (o ByteTrack da um id a cada objeto), suaviza o centro de cada id com
um filtro de Kalman (filtro_kalman.py) e publica:
  /vision/deteccoes     projeto_bloco_interfaces/msg/DeteccaoArray (caixa, classe, confianca, id,
                        centro filtrado e velocidade)
  /vision/rastreamento  frame anotado: caixas, trilha bruta (fina, branca) e filtrada (grossa)

Parametros (config/params.yaml), todos alteraveis em execucao com `ros2 param set`:
  modelo, pasta_modelos  pesos .pt. Nome sem caminho: procurado em share/projeto_bloco/modelos
                         (pesos treinados no projeto) e senao em pasta_modelos (pesos oficiais da
                         Ultralytics, baixados na 1a vez)
  confianca, iou         limiares do YOLO
  frequencia_hz          taxa de inferencia e publicacao
  classes                nomes COCO a rastrear (["*"] = todas)
  dispositivo, imgsz     'cpu' ou 'cuda:0'; lado da imagem de entrada da rede
  threads_cpu            threads do torch na CPU (poucas: nao disputa CPU com os outros nos)
  kalman.sigma_a, kalman.sigma_z   ruido de aceleracao (px/s^2) e de medicao (px) do filtro
  trilha_max, perda_max_s          pontos por trilha; tempo sem ver um id antes de descarta-lo
  show                   janela cv2.imshow
"""

import os
import signal
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor,
                                SetParametersResult)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from projeto_bloco.filtro_kalman import FiltroKalman2D
from projeto_bloco_interfaces.msg import DeteccaoArray, DeteccaoObj

try:
    import torch
    from ultralytics import YOLO
except ImportError:      # venv do YOLO fora do PYTHONPATH (ver ros2_ws/ambiente.sh)
    torch = YOLO = None

INTERVALO_RUGOSIDADE_S = 10.0   # periodo do log que compara trilha bruta x filtrada
# imagem anotada: confiavel atende assinantes reliable (RViz) e best effort (rqt_image_view)
QOS_VISUALIZACAO = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)


def faixa(de, ate, inteiro=False):
    """Descritor de parametro com faixa valida (o rclpy recusa valores fora dela)."""
    if inteiro:
        return {'integer_range': [IntegerRange(from_value=de, to_value=ate, step=1)]}
    return {'floating_point_range': [FloatingPointRange(from_value=de, to_value=ate, step=0.0)]}


def cor_do_id(tid):
    """Cor BGR fixa por id (matiz espalhado pelo circulo)."""
    pixel = np.array([[[(tid * 47) % 180, 220, 255]]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


class Trilha:
    """Historico de um id do ByteTrack: filtro de Kalman + pontos brutos e filtrados."""

    def __init__(self, cx, cy, t, classe, cfg):
        self.kf = FiltroKalman2D(cx, cy, t, cfg['kalman.sigma_a'], cfg['kalman.sigma_z'])
        self.bruta = deque(maxlen=cfg['trilha_max'])
        self.filtrada = deque(maxlen=cfg['trilha_max'])
        self.classe = classe
        self.visto = t


class RastreadorYolo(Node):

    PARAMETROS = [   # nome, padrao, descricao, faixa
        ('modelo', 'alvo_yolov8s_640.pt', 'pesos YOLO (.pt); ver a docstring do modulo', {}),
        ('pasta_modelos', '~/.cache/projeto_bloco/modelos', 'onde baixam os pesos oficiais', {}),
        ('confianca', 0.5, 'limiar de confianca do YOLO', faixa(0.05, 0.95)),
        ('iou', 0.5, 'limiar de IoU da supressao de nao-maximos', faixa(0.1, 0.9)),
        ('frequencia_hz', 10.0, 'taxa de inferencia e publicacao', faixa(0.5, 30.0)),
        ('classes', ['alvo'], 'classes rastreadas (["*"] = todas as do modelo)', {}),
        ('dispositivo', 'cpu', "'cpu' ou 'cuda:0'", {}),
        ('imgsz', 640, 'lado da entrada da rede (multiplo de 32)', faixa(160, 1280, True)),
        ('threads_cpu', 4, 'threads do torch na CPU', faixa(1, 64, True)),
        ('kalman.sigma_a', 200.0, 'ruido de aceleracao do Kalman (px/s^2)', faixa(1.0, 10000.0)),
        ('kalman.sigma_z', 15.0, 'ruido de medicao do Kalman (px)', faixa(0.5, 100.0)),
        ('trilha_max', 40, 'pontos por trilha desenhada', faixa(2, 500, True)),
        ('perda_max_s', 1.0, 'tempo sem ver um id antes de descartar a trilha', faixa(0.1, 10.0)),
        ('show', False, 'janela cv2.imshow', {}),
    ]
    RECARREGA_MODELO = {'modelo', 'pasta_modelos', 'dispositivo'}

    def __init__(self):
        super().__init__('rastreador_yolo')
        if YOLO is None:
            self.get_logger().fatal('ultralytics nao encontrado: rode ./scripts/setup.sh e '
                                    '`source ros2_ws/ambiente.sh` (venv do YOLO no PYTHONPATH)')
            raise RuntimeError('ultralytics')
        for nome, padrao, descricao, extra in self.PARAMETROS:
            self.declare_parameter(nome, padrao,
                                   ParameterDescriptor(description=descricao, **extra))
        self.cfg = {nome: self.get_parameter(nome).value for nome, *_ in self.PARAMETROS}
        torch.set_num_threads(self.cfg['threads_cpu'])

        self.modelo = self.carregar_modelo(self.cfg)
        self.ids_classes = self.validar_classes(self.cfg['classes'], self.modelo)

        self.bridge = CvBridge()
        self.ultimo_frame = None
        self.frame_processado = None
        self.trilhas = {}
        self.rugosidade = [0.0, 0.0, 0]            # soma |2a dif.| bruta, filtrada, amostras
        self.t_rugosidade = time.monotonic()

        self.create_subscription(Image, '/camera/image_raw', self.receber, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(DeteccaoArray, '/vision/deteccoes', 10)
        self.pub_img = self.create_publisher(Image, '/vision/rastreamento', QOS_VISUALIZACAO)
        self.timer = self.create_timer(1.0 / self.cfg['frequencia_hz'], self.processar)
        self.add_on_set_parameters_callback(self.ao_mudar_parametros)
        self.get_logger().info(
            f"{self.cfg['modelo']} ({self.cfg['dispositivo']}), confianca >= "
            f"{self.cfg['confianca']:.2f}, {self.cfg['frequencia_hz']:.1f} Hz, classes "
            f"{self.cfg['classes']} -> /vision/deteccoes, /vision/rastreamento")

    # ---------- modelo e parametros ----------
    def carregar_modelo(self, cfg):
        nome = os.path.expanduser(cfg['modelo'])
        if os.sep not in nome:
            do_pacote = os.path.join(get_package_share_directory('projeto_bloco'), 'modelos', nome)
            if os.path.exists(do_pacote):
                nome = do_pacote
            else:                        # peso oficial: baixado para pasta_modelos na 1a vez
                pasta = os.path.expanduser(cfg['pasta_modelos'])
                os.makedirs(pasta, exist_ok=True)
                nome = os.path.join(pasta, nome)
        dispositivo = cfg['dispositivo']
        if dispositivo.startswith('cuda') and not torch.cuda.is_available():
            raise ValueError(f'{dispositivo} indisponivel (torch sem CUDA)')
        t0 = time.monotonic()
        modelo = YOLO(nome)              # pesos oficiais sao baixados na 1a vez
        modelo.predict(np.zeros((64, 64, 3), np.uint8), device=dispositivo, verbose=False)
        self.get_logger().info(f'modelo {nome} carregado em {dispositivo} '
                               f'({time.monotonic() - t0:.1f} s)')
        return modelo

    @staticmethod
    def validar_classes(classes, modelo):
        if list(classes) == ['*']:
            return None                  # todas
        por_nome = {v: k for k, v in modelo.names.items()}
        desconhecidas = [c for c in classes if c not in por_nome]
        if desconhecidas or not classes:
            raise ValueError(f'classes desconhecidas {desconhecidas}; exemplos validos: '
                             + ', '.join(list(por_nome)[:12]) + ' ...')
        return [por_nome[c] for c in classes]

    def ao_mudar_parametros(self, params):
        """Valida e aplica mudancas feitas com `ros2 param set/load` durante a execucao."""
        # `ros2 param load` reenvia todos os valores do YAML: so reage ao que mudou de fato
        novos = {p.name: p.value for p in params
                 if p.name in self.cfg and p.value != self.cfg[p.name]}
        if not novos:
            return SetParametersResult(successful=True)
        cfg = {**self.cfg, **novos}
        try:
            modelo = self.carregar_modelo(cfg) if self.RECARREGA_MODELO & novos.keys() else None
            ids = self.validar_classes(cfg['classes'], modelo or self.modelo)
        except Exception as e:           # noqa: BLE001 - qualquer falha recusa a mudanca
            self.get_logger().warn(f'parametros recusados {novos}: {e}')
            return SetParametersResult(successful=False, reason=str(e))

        self.cfg = cfg
        self.ids_classes = ids
        if modelo is not None:
            self.modelo = modelo         # modelo novo = rastreador novo: ids recomecam
            self.trilhas.clear()
        if 'threads_cpu' in novos:
            torch.set_num_threads(cfg['threads_cpu'])
        if 'frequencia_hz' in novos:
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / cfg['frequencia_hz'], self.processar)
        for tr in self.trilhas.values():
            tr.kf.sigma_a, tr.kf.sigma_z = cfg['kalman.sigma_a'], cfg['kalman.sigma_z']
            tr.bruta = deque(tr.bruta, maxlen=cfg['trilha_max'])
            tr.filtrada = deque(tr.filtrada, maxlen=cfg['trilha_max'])
        self.get_logger().info('parametros alterados: '
                               + ', '.join(f'{k}={v}' for k, v in novos.items()))
        return SetParametersResult(successful=True)

    # ---------- processamento ----------
    def receber(self, msg):
        self.ultimo_frame = msg

    def processar(self):
        msg = self.ultimo_frame
        if msg is None or msg is self.frame_processado:
            return                       # sem frame novo desde a ultima inferencia
        self.frame_processado = msg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cfg = self.cfg

        t0 = time.perf_counter()
        r = self.modelo.track(frame, persist=True, conf=cfg['confianca'], iou=cfg['iou'],
                              classes=self.ids_classes, imgsz=cfg['imgsz'],
                              device=cfg['dispositivo'], tracker='bytetrack.yaml',
                              verbose=False)[0]
        ms = (time.perf_counter() - t0) * 1000
        if ms > 3000 / cfg['frequencia_hz']:
            self.get_logger().warn(f'inferencia lenta: {ms:.0f} ms (> 3 periodos)')

        arr = DeteccaoArray(header=msg.header, largura=frame.shape[1], altura=frame.shape[0])
        vistos = set()
        b = r.boxes
        if b.id is not None:
            for (x1, y1, x2, y2), conf, cls, tid in zip(b.xyxy.tolist(), b.conf.tolist(),
                                                        b.cls.tolist(), b.id.int().tolist()):
                classe = self.modelo.names[int(cls)]
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                tr = self.trilhas.get(tid)
                if tr is None:
                    tr = self.trilhas[tid] = Trilha(cx, cy, t, classe, cfg)
                fx, fy = tr.kf.atualizar(cx, cy, t)
                tr.bruta.append((cx, cy))
                tr.filtrada.append((fx, fy))
                tr.visto, tr.classe = t, classe
                self.acumular_rugosidade(tr)
                vistos.add(tid)
                vx, vy = tr.kf.velocidade
                arr.deteccoes.append(DeteccaoObj(
                    classe=classe, confianca=float(conf), x=int(x1), y=int(y1),
                    w=int(x2 - x1), h=int(y2 - y1), id=tid, cx=fx, cy=fy, vx=vx, vy=vy))

        for tid in list(self.trilhas):   # ids sumidos: so predicao, ate perda_max_s
            if tid in vistos:
                continue
            tr = self.trilhas[tid]
            if t - tr.visto > cfg['perda_max_s']:
                del self.trilhas[tid]
            else:
                tr.kf.prever(t)

        self.pub_det.publish(arr)
        self.publicar_imagem(frame, arr, vistos, ms, msg.header)
        self.logar_rugosidade()

    def acumular_rugosidade(self, tr):
        """|2a diferenca| do ultimo ponto: mede o 'tremido' da trilha bruta e da filtrada."""
        if len(tr.bruta) < 3:
            return
        for i, pts in enumerate((tr.bruta, tr.filtrada)):
            (x0, y0), (x1, y1), (x2, y2) = pts[-3], pts[-2], pts[-1]
            self.rugosidade[i] += float(np.hypot(x2 - 2 * x1 + x0, y2 - 2 * y1 + y0))
        self.rugosidade[2] += 1

    def logar_rugosidade(self):
        if time.monotonic() - self.t_rugosidade < INTERVALO_RUGOSIDADE_S:
            return
        bruta, filtrada, n = self.rugosidade
        if n:
            bruta, filtrada = bruta / n, filtrada / n
            self.get_logger().info(
                f'rugosidade media da trilha (|2a diferenca|, {n} pontos): bruta {bruta:.1f} px,'
                f' Kalman {filtrada:.1f} px ({100 * (filtrada - bruta) / max(bruta, 1e-6):+.0f}%)')
        self.rugosidade = [0.0, 0.0, 0]
        self.t_rugosidade = time.monotonic()

    # ---------- visualizacao ----------
    def publicar_imagem(self, frame, arr, vistos, ms, header):
        out = frame.copy()
        for tid, tr in self.trilhas.items():
            cor = cor_do_id(tid)
            if len(tr.bruta) > 1:        # filtrada grossa embaixo, bruta fina por cima
                cv2.polylines(out, [np.int32(tr.filtrada)], False, cor, 4, cv2.LINE_AA)
                cv2.polylines(out, [np.int32(tr.bruta)], False, (255, 255, 255), 1, cv2.LINE_AA)
                for x, y in tr.bruta:
                    cv2.circle(out, (int(x), int(y)), 2, (255, 255, 255), -1, cv2.LINE_AA)
            if tid not in vistos:        # so predicao: circulo vazado
                x, y = tr.kf.posicao
                cv2.circle(out, (int(x), int(y)), 8, cor, 1, cv2.LINE_AA)
        for d in arr.deteccoes:
            cor = cor_do_id(d.id)
            cv2.rectangle(out, (d.x, d.y), (d.x + d.w, d.y + d.h), cor, 2)
            rotulo = f'{d.classe} #{d.id} {d.confianca:.2f}'
            (tw, th), _ = cv2.getTextSize(rotulo, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (d.x, d.y - th - 8), (d.x + tw + 6, d.y), cor, -1)
            cv2.putText(out, rotulo, (d.x + 3, d.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)
            c = (int(d.cx), int(d.cy))
            cv2.circle(out, c, 5, cor, -1, cv2.LINE_AA)
            cv2.arrowedLine(out, c, (int(d.cx + 0.25 * d.vx), int(d.cy + 0.25 * d.vy)), cor, 2,
                            cv2.LINE_AA, tipLength=0.3)
        cfg = self.cfg
        hud = (f"{os.path.basename(cfg['modelo'])}  conf>={cfg['confianca']:.2f}  "
               f"{cfg['frequencia_hz']:.1f} Hz  {ms:.0f} ms  objetos: {len(arr.deteccoes)}")
        cv2.rectangle(out, (0, 0), (out.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(out, hud, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(out, 'trilha: branca = centro do YOLO | colorida = filtro de Kalman',
                    (8, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
        img = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        img.header = header
        self.pub_img.publish(img)
        if cfg['show']:
            cv2.imshow('rastreamento', out)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RastreadorYolo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (launch) nao interrompe a limpeza
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
