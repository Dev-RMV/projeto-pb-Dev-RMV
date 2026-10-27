"""
Piloto da demo: publica /cmd_vel seguindo uma rota fixa, para o SLAM construir o mapa sem teclado.

A rota e uma lista de passos (parametro `rota`, strings):
  "reto <metros> <m/s>"        anda em linha reta (metros < 0 = de re)
  "girar <graus> <graus/s>"    gira no lugar (graus > 0 = esquerda)
  "parar <segundos>"           fica parado
Cada passo dura distancia/velocidade; o tempo e o do relogio do no (com use_sim_time, o do Gazebo),
entao a rota rende o mesmo caminho em qualquer maquina. Depois do ultimo passo (ou de `repetir`
voltas) publica velocidade zero e para. `espera_inicial_s` deixa o Gazebo e o SLAM assentarem.
Dirigir a mao: `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (nao suba os dois juntos).
"""

import math
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

ROTA_PADRAO = ['parar 3', 'reto 1.4 0.25', 'girar 90 45', 'reto 1.4 0.25', 'girar 90 45',
               'reto 1.4 0.25', 'girar 90 45', 'reto 1.4 0.25', 'girar 90 45', 'girar 360 30']


class PilotoDemo(Node):

    def __init__(self):
        super().__init__('piloto_demo')
        self.declare_parameter('rota', ROTA_PADRAO)
        self.declare_parameter('repetir', 1)
        self.declare_parameter('espera_inicial_s', 5.0)
        self.declare_parameter('taxa_hz', 20.0)
        rota = self.get_parameter('rota').value
        self.passos = self.compilar(rota) * max(1, int(self.get_parameter('repetir').value))
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t_inicio = None
        self.indice = -1                 # -1 = espera inicial
        self.t_passo = None
        self.encerrado = False
        self.create_timer(1.0 / self.get_parameter('taxa_hz').value, self.tick)
        total = sum(d for _, _, d in self.passos)
        self.get_logger().info(f'{len(self.passos)} passos, ~{total:.0f} s de rota + '
                               f"{self.get_parameter('espera_inicial_s').value:.0f} s de espera")

    def compilar(self, rota):
        passos = []                      # (v, w, duracao)
        for linha in rota:
            p = linha.split()
            try:
                if p[0] == 'reto':
                    dist, vel = float(p[1]), abs(float(p[2]))
                    passos.append((math.copysign(vel, dist), 0.0, abs(dist) / vel))
                elif p[0] == 'girar':
                    ang, vel = math.radians(float(p[1])), math.radians(abs(float(p[2])))
                    passos.append((0.0, math.copysign(vel, ang), abs(ang) / vel))
                elif p[0] == 'parar':
                    passos.append((0.0, 0.0, float(p[1])))
                else:
                    raise ValueError('comando desconhecido')
            except (IndexError, ValueError, ZeroDivisionError) as e:
                self.get_logger().warn(f'passo ignorado "{linha}": {e}')
        return passos

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.encerrado:
            return
        t = self.agora()
        if t <= 0.0:                     # use_sim_time sem /clock ainda
            return
        if self.t_inicio is None:
            self.t_inicio = t
        if self.indice < 0:
            if t - self.t_inicio < self.get_parameter('espera_inicial_s').value:
                self.pub.publish(Twist())
                return
            self.avancar(t)
        v, w, dur = self.passos[self.indice]
        if t - self.t_passo >= dur:
            if self.indice + 1 >= len(self.passos):
                self.pub.publish(Twist())
                self.encerrado = True
                self.get_logger().info('rota concluida: robo parado')
                return
            self.avancar(t)
            v, w, dur = self.passos[self.indice]
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)

    def avancar(self, t):
        self.indice += 1
        self.t_passo = t
        v, w, dur = self.passos[self.indice]
        self.get_logger().info(f'passo {self.indice + 1}/{len(self.passos)}: v={v:.2f} m/s '
                               f'w={math.degrees(w):.0f} graus/s por {dur:.1f} s')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PilotoDemo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:
                node.pub.publish(Twist())    # robo parado ao sair
            except Exception:                # noqa: BLE001 - contexto ja encerrado
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
