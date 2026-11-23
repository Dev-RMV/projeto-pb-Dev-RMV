"""
Obstaculo surpresa: faz uma caixa aparecer na frente do robo, no Gazebo, durante a patrulha.

Serve para exercitar a behavior tree (arvore_navegacao) e o desvio do Nav2 com um obstaculo que
nao esta no mapa e surge ja perto: quando o robo entra andando na zona de gatilho (circulo de
`gatilho_raio` em torno de `gatilho_x`, `gatilho_y`), o no chama o servico /spawn_entity do
Gazebo e cria uma caixa estatica com a face a `distancia_m` do lidar, na direcao em que o robo
esta virado. Dispara uma vez so. Com `remover_apos_s` > 0 a caixa some depois desse tempo
(/delete_entity), como alguem que cruzou o caminho e saiu.

A pose do robo vem do /odom: no Gazebo o plugin das esteiras publica a pose no mundo, que e o
mesmo referencial do mapa. Publica /obstaculo_surpresa/status (String: "criada x=.. y=.." e
"removida") para o bag e o grafico da trajetoria.

  ros2 run projeto_bloco obstaculo_surpresa --ros-args -p use_sim_time:=true \
      -p gatilho_x:=0.8 -p gatilho_y:=1.37
"""

import math

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

SDF_CAIXA = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{nome}">
    <static>true</static>
    <link name="link">
      <collision name="c">
        <geometry><box><size>{lado} {lado} {altura}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>{lado} {lado} {altura}</size></box></geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ObstaculoSurpresa(Node):

    PARAMETROS = {
        'gatilho_x': 0.80,               # centro da zona de gatilho (frame do mundo = map)
        'gatilho_y': 1.37,
        'gatilho_raio': 0.30,
        'velocidade_min': 0.10,          # so dispara com o robo andando para a frente (m/s)
        'distancia_m': 0.55,             # do lidar ate a face da caixa
        'lado_m': 0.30,
        'altura_m': 0.50,                # mais alta que o plano do lidar (0,285 m)
        'lidar_x': -0.09,                # lidar no base_footprint (mastro na traseira, URDF)
        'remover_apos_s': 0.0,           # 0 = a caixa fica
        'nome': 'caixa_surpresa',
    }

    def __init__(self):
        super().__init__('obstaculo_surpresa')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)
        self.p = {nome: self.get_parameter(nome).value for nome in self.PARAMETROS}
        self.criar = self.create_client(SpawnEntity, '/spawn_entity')
        self.apagar = self.create_client(DeleteEntity, '/delete_entity')
        self.pub = self.create_publisher(String, '/obstaculo_surpresa/status', 10)
        self.disparou = False
        self.t_criada = None
        self.create_subscription(Odometry, '/odom', self.ao_odom, 10)
        self.create_timer(0.5, self.checar_remocao)
        self.get_logger().info(
            f"gatilho em ({self.p['gatilho_x']:.2f}, {self.p['gatilho_y']:.2f}), raio "
            f"{self.p['gatilho_raio']:.2f} m; caixa a {self.p['distancia_m']:.2f} m do lidar")

    def ao_odom(self, msg):
        if self.disparou:
            return
        pos = msg.pose.pose.position
        if math.hypot(pos.x - self.p['gatilho_x'], pos.y - self.p['gatilho_y']) > \
                self.p['gatilho_raio']:
            return
        if msg.twist.twist.linear.x < self.p['velocidade_min']:
            return
        if not self.criar.service_is_ready():
            self.get_logger().warn('/spawn_entity indisponivel', throttle_duration_sec=5.0)
            return
        self.disparou = True
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # centro da caixa: lidar + (distancia ate a face + meio lado) na direcao do robo
        avanco = self.p['lidar_x'] + self.p['distancia_m'] + self.p['lado_m'] / 2.0
        x = pos.x + avanco * math.cos(yaw)
        y = pos.y + avanco * math.sin(yaw)
        pedido = SpawnEntity.Request()
        pedido.name = self.p['nome']
        pedido.xml = SDF_CAIXA.format(nome=self.p['nome'], lado=self.p['lado_m'],
                                      altura=self.p['altura_m'])
        pedido.initial_pose.position.x = x
        pedido.initial_pose.position.y = y
        pedido.initial_pose.position.z = self.p['altura_m'] / 2.0
        pedido.initial_pose.orientation.z = math.sin(yaw / 2.0)
        pedido.initial_pose.orientation.w = math.cos(yaw / 2.0)
        pedido.reference_frame = 'world'
        self.get_logger().warn(
            f"robo em ({pos.x:.2f}, {pos.y:.2f}) a {msg.twist.twist.linear.x:.2f} m/s: criando "
            f"{self.p['nome']} em ({x:.2f}, {y:.2f}), a {self.p['distancia_m']:.2f} m do lidar")
        self.criar.call_async(pedido).add_done_callback(lambda f: self.ao_criar(f, x, y))

    def ao_criar(self, futuro, x, y):
        resp = futuro.result()
        if resp is None or not resp.success:
            self.get_logger().error(
                f"/spawn_entity falhou: {resp.status_message if resp else 'sem resposta'}")
            return
        self.t_criada = self.get_clock().now().nanoseconds * 1e-9
        self.pub.publish(String(data=f'criada x={x:.3f} y={y:.3f}'))
        self.get_logger().info(f"{self.p['nome']} criada no Gazebo")

    def checar_remocao(self):
        if self.t_criada is None or self.p['remover_apos_s'] <= 0.0:
            return
        agora = self.get_clock().now().nanoseconds * 1e-9
        if agora - self.t_criada < self.p['remover_apos_s']:
            return
        self.t_criada = None
        self.apagar.call_async(DeleteEntity.Request(name=self.p['nome']))
        self.pub.publish(String(data='removida'))
        self.get_logger().info(f"{self.p['nome']} removida do Gazebo")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstaculoSurpresa()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
