"""
Publisher: captura frames de camera ou video com OpenCV e publica em /camera/image_raw.

Parametros:
  source  '0','1',... = indice de camera; caminho = arquivo de video (obrigatorio)
  fps     taxa de publicacao (0 = a do video, ou 30 na camera)
"""

import os
import signal

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FimDaFonte(Exception):
    """Camera desconectada / leitura falhou (levantada no timer para sair do spin)."""


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        # dynamic_typing: `-p source:=0` chega como inteiro, `source:=video.mp4` como string
        self.declare_parameter('source', '', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('fps', 0.0)
        source = str(self.get_parameter('source').value).strip()
        fps = float(self.get_parameter('fps').value)

        self.is_camera = source.isdigit()
        if self.is_camera:
            self.cap = cv2.VideoCapture(int(source), cv2.CAP_V4L2)
        else:
            if source == '':
                self.get_logger().fatal('informe o parametro source (camera ou caminho de video)')
                raise RuntimeError('source')
            self.cap = cv2.VideoCapture(os.path.expanduser(source))
        if not self.cap.isOpened():
            self.get_logger().fatal(f'nao abriu a fonte: {source}')
            raise RuntimeError(source)

        if fps <= 0.0:
            fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'fonte {source} ({w}x{h}) -> /camera/image_raw a {fps:.0f} FPS')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.publicar)

    def publicar(self):
        ok, frame = self.cap.read()
        if not ok and not self.is_camera:            # fim do video: recomeca
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('falha de leitura; encerrando')
            raise FimDaFonte()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, FimDaFonte, RuntimeError):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (launch) nao interrompe a limpeza
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
