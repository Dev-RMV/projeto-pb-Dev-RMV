"""
Subscriber: recebe /camera/image_raw, converte para HSV e segmenta por cor (varias cores).

Cada cor de `cores` tem sua faixa `<cor>.lower` / `<cor>.upper` ([H,S,V], OpenCV: H 0-179).
Se lower[H] > upper[H] a faixa "da a volta" pelo 0 (caso do vermelho).
Publica a mascara (pintada na cor de cada faixa) em /vision/mask e o frame anotado em
/vision/segmented. Servico /vision/status: total de objetos e contagem por cor no ultimo frame.
"""

import signal
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from projeto_bloco_interfaces.srv import VisionStatus


class ColorSegmenter(Node):

    def __init__(self):
        super().__init__('color_segmenter')
        self.declare_parameter('cores', ['azul'])
        self.declare_parameter('min_area', 1500)
        self.declare_parameter('show', False)
        self.min_area = int(self.get_parameter('min_area').value)
        self.show = bool(self.get_parameter('show').value)

        self.cores = []   # (nome, lower, upper, bgr para desenhar)
        for nome in self.get_parameter('cores').value:
            self.declare_parameter(f'{nome}.lower', [0, 0, 0])
            self.declare_parameter(f'{nome}.upper', [0, 0, 0])
            lo = np.array(self.get_parameter(f'{nome}.lower').value, dtype=np.uint8)
            hi = np.array(self.get_parameter(f'{nome}.upper').value, dtype=np.uint8)
            if not hi.any():
                self.get_logger().fatal(f'faixa da cor "{nome}" nao definida (params.yaml)')
                raise RuntimeError(nome)
            self.cores.append((nome, lo, hi, self.bgr_da_faixa(lo, hi)))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.contagem = {nome: 0 for nome, *_ in self.cores}
        self.frames = 0

        self.create_subscription(Image, '/camera/image_raw', self.processar,
                                 qos_profile_sensor_data)
        self.pub_mask = self.create_publisher(Image, '/vision/mask', qos_profile_sensor_data)
        self.pub_seg = self.create_publisher(Image, '/vision/segmented', qos_profile_sensor_data)
        self.create_service(VisionStatus, '/vision/status', self.status)
        self.get_logger().info('cores: ' + ', '.join(
            f'{n} {lo.tolist()}..{hi.tolist()}' for n, lo, hi, _ in self.cores)
            + '; /vision/status pronto')

    @staticmethod
    def bgr_da_faixa(lo, hi):
        """Cor BGR representativa da faixa (matiz central, saturacao e valor maximos)."""
        h = (int(lo[0]) + int(hi[0]) + (180 if lo[0] > hi[0] else 0)) // 2 % 180
        pixel = np.array([[[h, 255, 255]]], dtype=np.uint8)
        return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])

    @staticmethod
    def mascara(hsv, lo, hi):
        if lo[0] <= hi[0]:
            return cv2.inRange(hsv, lo, hi)
        # matiz da volta pelo 0 (vermelho): [lo..179] U [0..hi]
        return (cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], dtype=np.uint8))
                | cv2.inRange(hsv, np.array([0, lo[1], lo[2]], dtype=np.uint8), hi))

    def processar(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        out = frame.copy()
        mask_rgb = np.zeros_like(frame)
        contagem = {}
        for nome, lo, hi, bgr in self.cores:
            mask = self.mascara(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # tira ruido
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # fecha buracos
            contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            objetos = [c for c in contornos if cv2.contourArea(c) >= self.min_area]
            contagem[nome] = len(objetos)
            mask_rgb[mask > 0] = bgr
            for i, c in enumerate(objetos, start=1):
                x, y, w, h = cv2.boundingRect(c)
                cv2.drawContours(out, [c], -1, bgr, 2)
                cv2.rectangle(out, (x, y), (x + w, y + h), bgr, 2)
                cv2.putText(out, f'{nome} #{i}', (x, max(y - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        total = sum(contagem.values())
        detalhe = ', '.join(f'{n} {c}' for n, c in contagem.items() if c)
        cv2.putText(out, f'objetos: {total}' + (f' ({detalhe})' if detalhe else ''), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        with self.lock:
            self.contagem = contagem
            self.frames += 1

        m = self.bridge.cv2_to_imgmsg(mask_rgb, encoding='bgr8')
        m.header = msg.header
        self.pub_mask.publish(m)
        s = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        s.header = msg.header
        self.pub_seg.publish(s)
        if self.show:
            cv2.imshow('segmentado', out)
            cv2.imshow('mascara', mask_rgb)
            cv2.waitKey(1)

    def status(self, request, response):
        with self.lock:
            contagem = dict(self.contagem)
            response.frames_processed = self.frames
        response.colors = list(contagem.keys())
        response.counts = list(contagem.values())
        response.num_objects = sum(contagem.values())
        response.message = (f'{response.num_objects} objeto(s) no ultimo frame: '
                            + ', '.join(f'{n}={c}' for n, c in contagem.items())
                            + f' ({response.frames_processed} frames processados)')
        self.get_logger().info(f'/vision/status -> {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ColorSegmenter()
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
