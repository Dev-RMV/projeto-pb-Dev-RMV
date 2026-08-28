"""Subscriber de exemplo: callback enxuto, logging com parcimônia."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NoSubscriber(Node):
    def __init__(self):
        super().__init__('no_subscriber')
        self._sub = self.create_subscription(String, 'exemplo/mensagens', self._ao_receber, 10)

    def _ao_receber(self, msg: String):
        # Boas práticas: callback curto — processamento pesado vai para métodos/módulos próprios
        self.get_logger().info(f'Recebido: {msg.data}')


def main():
    rclpy.init()
    no = NoSubscriber()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
