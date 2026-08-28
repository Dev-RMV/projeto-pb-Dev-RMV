"""Publisher de exemplo com boas práticas: parâmetro declarado, timer, logging.

Adapte: troque String pelo tipo do seu domínio (ex.: sensor_msgs/Image no pipeline de visão).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NoPublisher(Node):
    def __init__(self):
        super().__init__('no_publisher')
        # Boas práticas: frequências/limiares como PARÂMETROS, nunca números mágicos
        self.declare_parameter('frequencia_hz', 1.0)
        freq = self.get_parameter('frequencia_hz').value
        self._pub = self.create_publisher(String, 'exemplo/mensagens', 10)
        self._timer = self.create_timer(1.0 / freq, self._publicar)
        self._contador = 0
        self.get_logger().info(f'Publicando em exemplo/mensagens a {freq} Hz')

    def _publicar(self):
        msg = String()
        msg.data = f'Mensagem {self._contador}'
        self._pub.publish(msg)
        self._contador += 1


def main():
    rclpy.init()
    no = NoPublisher()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
