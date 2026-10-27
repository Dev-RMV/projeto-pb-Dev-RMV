"""
Salva um quadro de cada topico de imagem (evidencias): espera o primeiro frame de cada topico da
lista e grava <pasta>/<nome_do_topico>.jpg; sai quando todos chegaram ou no timeout.

  ros2 run projeto_bloco salvar_imagens --ros-args -p pasta:=ros2_ws/log/evidencias \
      -p "topicos:=['/vision/segmented', '/vision/rastreamento', '/torre/visao', '/camera/image_raw']"
"""

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICOS_PADRAO = ['/vision/segmented', '/vision/rastreamento', '/torre/visao', '/camera/image_raw']


class SalvarImagens(Node):

    def __init__(self):
        super().__init__('salvar_imagens')
        self.declare_parameter('topicos', TOPICOS_PADRAO)
        self.declare_parameter('pasta', 'evidencias')
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('espera_s', 0.0)        # descarta frames nos primeiros segundos
        self.pasta = os.path.expanduser(self.get_parameter('pasta').value)
        os.makedirs(self.pasta, exist_ok=True)
        self.bridge = CvBridge()
        self.t0 = time.monotonic()
        self.pendentes = set(self.get_parameter('topicos').value)
        for t in list(self.pendentes):
            self.create_subscription(Image, t, lambda m, t=t: self.receber(t, m),
                                     qos_profile_sensor_data)
        self.get_logger().info(f'aguardando {sorted(self.pendentes)} -> {self.pasta}')
        self.create_timer(0.5, self.checar)

    def receber(self, topico, msg):
        if topico not in self.pendentes:
            return
        if time.monotonic() - self.t0 < self.get_parameter('espera_s').value:
            return
        nome = topico.strip('/').replace('/', '_') + '.jpg'
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.pasta, nome), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.pendentes.discard(topico)
        self.get_logger().info(f'{topico} ({msg.width}x{msg.height}) -> {nome}')

    def checar(self):
        if not self.pendentes:
            self.get_logger().info('todos os quadros salvos')
            raise SystemExit(0)
        if time.monotonic() - self.t0 > self.get_parameter('timeout_s').value:
            self.get_logger().warn(f'timeout; sem quadros de {sorted(self.pendentes)}')
            raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    codigo = 0
    try:
        node = SalvarImagens()
        rclpy.spin(node)
    except SystemExit as e:
        codigo = int(e.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return codigo


if __name__ == '__main__':
    raise SystemExit(main())
