"""
Segmentacao por instancia com YOLO-seg (ultralytics): mascaras dos alvos sobre o frame.

Assina a imagem da camera (topico_entrada, por padrao /camera/image_raw) e guarda so o frame mais
recente. Um timer a `frequencia_hz` roda model.predict() com um modelo YOLOv8n-seg treinado para a
classe `alvo` (a mesma arte impressa do detector; dataset sintetico com poligonos) e publica:
  /vision/segmented      frame com as mascaras pintadas POR CLASSE (cor fixa por classe, uma por
                         instancia na borda) com transparencia `alpha` (0,5), contorno de cada
                         instancia, rotulo "classe conf" e um cabecalho com modelo/limiar/tempo
  /vision/segmentacao    projeto_bloco_interfaces/msg/DeteccaoArray: uma DeteccaoObj por instancia
                         (caixa, classe, confianca, id = indice da instancia no frame e cx/cy =
                         centroide da MASCARA, que e o ponto de mira mais robusto que o centro da
                         caixa quando o alvo esta inclinado ou parcialmente cortado)

Parametros (config/params.yaml): modelo, pasta_modelos (como no rastreador_yolo), confianca, iou,
frequencia_hz, imgsz, dispositivo, threads_cpu, alpha, topico_entrada, show. `confianca`, `iou`,
`frequencia_hz`, `alpha` e `imgsz` aceitam `ros2 param set` em execucao.
"""

import os
import signal
import time

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

from projeto_bloco_interfaces.msg import DeteccaoArray, DeteccaoObj

try:
    import torch
    from ultralytics import YOLO
except ImportError:      # venv do YOLO fora do PYTHONPATH (ver ros2_ws/ambiente.sh)
    torch = YOLO = None

# imagem anotada: confiavel atende assinantes reliable (RViz) e best effort (rqt_image_view)
QOS_VISUALIZACAO = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
# cor (BGR) por classe: a classe 0 do modelo treinado e "alvo"; as demais cobrem modelos COCO
PALETA = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (0, 200, 255), (255, 0, 200), (200, 255, 0),
          (128, 0, 255), (0, 128, 255), (255, 255, 0), (0, 255, 128)]


def faixa(de, ate, inteiro=False):
    if inteiro:
        return {'integer_range': [IntegerRange(from_value=de, to_value=ate, step=1)]}
    return {'floating_point_range': [FloatingPointRange(from_value=de, to_value=ate, step=0.0)]}


def caminho_do_modelo(nome, pasta_modelos):
    """Nome sem caminho: share/projeto_bloco/modelos (treinados aqui) ou pasta_modelos (oficiais)."""
    nome = os.path.expanduser(nome)
    if os.sep in nome:
        return nome
    do_pacote = os.path.join(get_package_share_directory('projeto_bloco'), 'modelos', nome)
    if os.path.exists(do_pacote):
        return do_pacote
    pasta = os.path.expanduser(pasta_modelos)
    os.makedirs(pasta, exist_ok=True)
    return os.path.join(pasta, nome)


class SegmentadorYolo(Node):

    PARAMETROS = [   # nome, padrao, descricao, faixa
        ('modelo', 'alvo_yolov8n_seg_416.pt', 'pesos YOLO-seg (.pt)', {}),
        ('pasta_modelos', '~/.cache/projeto_bloco/modelos', 'onde baixam os pesos oficiais', {}),
        ('confianca', 0.3, 'limiar de confianca', faixa(0.05, 0.95)),
        ('iou', 0.5, 'limiar de IoU da supressao de nao-maximos', faixa(0.1, 0.9)),
        ('frequencia_hz', 5.0, 'taxa de inferencia e publicacao', faixa(0.2, 30.0)),
        ('imgsz', 416, 'lado da entrada da rede (multiplo de 32)', faixa(160, 1280, True)),
        ('dispositivo', 'cpu', "'cpu' ou 'cuda:0'", {}),
        ('threads_cpu', 4, 'threads do torch na CPU', faixa(1, 64, True)),
        ('alpha', 0.5, 'transparencia das mascaras (0 = invisivel, 1 = opaca)', faixa(0.0, 1.0)),
        ('topico_entrada', '/camera/image_raw', 'imagem de entrada', {}),
        ('show', False, 'janela cv2.imshow', {}),
    ]
    DINAMICOS = {'confianca', 'iou', 'frequencia_hz', 'imgsz', 'alpha', 'threads_cpu'}
    RECARREGA_MODELO = {'modelo', 'pasta_modelos', 'dispositivo'}

    def __init__(self):
        super().__init__('segmentador_yolo')
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

        self.bridge = CvBridge()
        self.ultimo_frame = None
        self.frame_processado = None
        self.create_subscription(Image, self.cfg['topico_entrada'], self.receber,
                                 qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, '/vision/segmented', QOS_VISUALIZACAO)
        self.pub_det = self.create_publisher(DeteccaoArray, '/vision/segmentacao', 10)
        self.timer = self.create_timer(1.0 / self.cfg['frequencia_hz'], self.processar)
        self.add_on_set_parameters_callback(self.ao_mudar_parametros)
        self.get_logger().info(
            f"{self.cfg['topico_entrada']} -> /vision/segmented (alpha {self.cfg['alpha']:.2f}), "
            f"/vision/segmentacao; conf >= {self.cfg['confianca']:.2f}, {self.cfg['frequencia_hz']:.1f} Hz")

    def carregar_modelo(self, cfg):
        caminho = caminho_do_modelo(cfg['modelo'], cfg['pasta_modelos'])
        if cfg['dispositivo'].startswith('cuda') and not torch.cuda.is_available():
            raise ValueError(f"{cfg['dispositivo']} indisponivel (torch sem CUDA)")
        t0 = time.monotonic()
        modelo = YOLO(caminho)
        if modelo.task != 'segment':
            raise ValueError(f'{caminho} nao e um modelo de segmentacao (task={modelo.task})')
        modelo.predict(np.zeros((64, 64, 3), np.uint8), device=cfg['dispositivo'], verbose=False)
        self.get_logger().info(f"modelo {caminho} carregado em {cfg['dispositivo']} "
                               f"({time.monotonic() - t0:.1f} s); classes {list(modelo.names.values())}")
        return modelo

    def ao_mudar_parametros(self, params):
        """Valida e aplica mudancas feitas com `ros2 param set/load` durante a execucao."""
        novos = {p.name: p.value for p in params
                 if p.name in self.cfg and p.value != self.cfg[p.name]}
        if not novos:
            return SetParametersResult(successful=True)
        fixos = set(novos) - self.DINAMICOS - self.RECARREGA_MODELO
        if fixos:
            return SetParametersResult(successful=False,
                                       reason=f'{sorted(fixos)}: so mudam ao reiniciar o no')
        cfg = {**self.cfg, **novos}
        if self.RECARREGA_MODELO & novos.keys():
            try:
                self.modelo = self.carregar_modelo(cfg)
            except Exception as e:       # noqa: BLE001 - qualquer falha recusa a mudanca
                self.get_logger().warn(f'parametros recusados {novos}: {e}')
                return SetParametersResult(successful=False, reason=str(e))
        self.cfg = cfg
        if 'threads_cpu' in novos:
            torch.set_num_threads(cfg['threads_cpu'])
        if 'frequencia_hz' in novos:
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / self.cfg['frequencia_hz'], self.processar)
        self.get_logger().info('parametros alterados: '
                               + ', '.join(f'{k}={v}' for k, v in novos.items()))
        return SetParametersResult(successful=True)

    # ---------- processamento ----------
    def receber(self, msg):
        self.ultimo_frame = msg

    def processar(self):
        msg = self.ultimo_frame
        if msg is None or msg is self.frame_processado:
            return
        self.frame_processado = msg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cfg = self.cfg
        t0 = time.perf_counter()
        r = self.modelo.predict(frame, conf=cfg['confianca'], iou=cfg['iou'], imgsz=cfg['imgsz'],
                                device=cfg['dispositivo'], verbose=False)[0]
        ms = (time.perf_counter() - t0) * 1000
        if ms > 3000 / cfg['frequencia_hz']:
            self.get_logger().warn(f'inferencia lenta: {ms:.0f} ms (> 3 periodos)')

        h, w = frame.shape[:2]
        arr = DeteccaoArray(header=msg.header, largura=w, altura=h)
        capa = frame.copy()                      # frame com as mascaras opacas
        uniao = np.zeros((h, w), np.uint8)       # onde ha mascara (para a mistura)
        contornos = []                           # (poligono, cor, rotulo, caixa)
        if r.masks is not None:
            for k, (poly, (x1, y1, x2, y2), conf, cls) in enumerate(
                    zip(r.masks.xy, r.boxes.xyxy.tolist(), r.boxes.conf.tolist(),
                        r.boxes.cls.tolist())):
                if len(poly) < 3:
                    continue
                pts = np.round(poly).astype(np.int32)
                cor = PALETA[int(cls) % len(PALETA)]
                cv2.fillPoly(capa, [pts], cor)
                mascara = np.zeros((h, w), np.uint8)
                cv2.fillPoly(mascara, [pts], 255)
                uniao |= mascara
                m = cv2.moments(mascara, binaryImage=True)
                cx = m['m10'] / m['m00'] if m['m00'] else (x1 + x2) / 2
                cy = m['m01'] / m['m00'] if m['m00'] else (y1 + y2) / 2
                classe = self.modelo.names[int(cls)]
                contornos.append((pts, cor, f'{classe} {conf:.2f}', (int(x1), int(y1)), (cx, cy)))
                arr.deteccoes.append(DeteccaoObj(
                    classe=classe, confianca=float(conf), x=int(x1), y=int(y1),
                    w=int(x2 - x1), h=int(y2 - y1), id=k, cx=float(cx), cy=float(cy),
                    vx=0.0, vy=0.0))
        self.pub_det.publish(arr)
        self.publicar_imagem(frame, capa, uniao, contornos, ms, msg.header)

    # ---------- visualizacao ----------
    def publicar_imagem(self, frame, capa, uniao, contornos, ms, header):
        cfg = self.cfg
        out = frame.copy()
        if contornos:
            sel = uniao.astype(bool)
            out[sel] = cv2.addWeighted(capa, cfg['alpha'], frame, 1.0 - cfg['alpha'], 0)[sel]
        for pts, cor, rotulo, (x, y), (cx, cy) in contornos:
            cv2.polylines(out, [pts], True, cor, 2, cv2.LINE_AA)
            cv2.drawMarker(out, (int(cx), int(cy)), (255, 255, 255), cv2.MARKER_CROSS, 14, 2,
                           cv2.LINE_AA)
            (tw, th), _ = cv2.getTextSize(rotulo, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x, max(y - th - 8, 0)), (x + tw + 6, max(y, th + 8)), cor, -1)
            cv2.putText(out, rotulo, (x + 3, max(y - 5, th + 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
        hud = (f"{os.path.basename(cfg['modelo'])}  conf>={cfg['confianca']:.2f}  "
               f"{cfg['frequencia_hz']:.1f} Hz  {ms:.0f} ms  alpha {cfg['alpha']:.1f}  "
               f"instancias: {len(contornos)}")
        escala = 0.6 if out.shape[1] >= 960 else 0.45      # cabe na camera do Gazebo (640 px)
        cv2.rectangle(out, (0, 0), (out.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(out, hud, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, escala, (0, 255, 255), 1,
                    cv2.LINE_AA)
        # legenda: cor por classe (as do modelo)
        x = 8
        for cid, nome in self.modelo.names.items():
            cor = PALETA[int(cid) % len(PALETA)]
            cv2.rectangle(out, (x, out.shape[0] - 24), (x + 16, out.shape[0] - 8), cor, -1)
            cv2.putText(out, nome, (x + 20, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
            x += 20 + 10 * len(nome) + 14
            if x > out.shape[1] - 80:
                break
        img = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        img.header = header
        self.pub_img.publish(img)
        if cfg['show']:
            cv2.imshow('segmentacao', out)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentadorYolo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
