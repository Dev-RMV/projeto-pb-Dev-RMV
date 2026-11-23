"""
Behavior tree da navegacao: pausa o Nav2 quando o lidar ve um obstaculo a menos de 0,5 m.

Behavior Tree em Python puro, hospedada num no ROS 2 e ticada por um timer. Padrao tick/status:
cada no responde SUCCESS, FAILURE ou RUNNING a cada tick, e o tick desce pela arvore a partir da
raiz (parametro `periodo_tick`, no relogio do no; com use_sim_time, o do Gazebo).

Arvore montada:
    Sequence "Raiz"
      Fallback "CaminhoSeguro"
        CaminhoLivre               Condition: lidar (/scan), setor frontal, sem obstaculo novo a
                                   menos de `distancia_parada_m`
        Sequence "PausaPorObstaculo"
          InterromperNavegacao     Action: cancela o goal no Nav2 e espera o robo parar
          Esperar                  Action: RUNNING ate completar `espera_s`
          RetomarGoal              Action: libera a retomada do mesmo goal
      NavegarParaPose              Action: goal NavigateToPose no Nav2; RUNNING ate chegar

Com o caminho livre, o Fallback responde SUCCESS ja no primeiro filho e a raiz tica o
NavegarParaPose, que mantem o goal no Nav2. Quando a condicao falha, o Fallback desce para a
pausa: o goal e cancelado (o "halt" da Action), o robo fica `espera_s` parado e so entao a raiz
volta a ticar o NavegarParaPose, que reenvia o MESMO goal. O obstaculo que causou a pausa fica
"tratado": enquanto ele continuar no setor a menos de `distancia_parada_m` + `histerese_m`, a
condicao responde SUCCESS, para o Nav2 poder contorna-lo pelo costmap. Um obstaculo novo, ou o
mesmo depois de sair do setor e voltar, gera outra pausa.

O goal entra pelo action server `navegar_com_pausa` (nav2_msgs/action/NavigateToPose, o mesmo
tipo do Nav2), entao serve o no `patrulha`, o RViz ou a linha de comando:

  ros2 run projeto_bloco arvore_navegacao --ros-args -p use_sim_time:=true
  ros2 action send_goal /navegar_com_pausa nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.3}, orientation: {w: 1.0}}}}"

Publica /arvore/estado (String: OCIOSO, NAVEGANDO, PAUSADO, CHEGOU ou FALHOU) e
/arvore/distancia_frontal (Float32, metros; inf = nada no setor) a cada tick.
"""

import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

V_PARADO = 0.02          # m/s: abaixo disso o robo conta como parado
W_PARADO = 0.05          # rad/s


def distancia_frontal(scan, setor_graus):
    """Menor distancia valida do LaserScan no setor frontal (+-setor/2 em torno de 0 rad)."""
    meio = math.radians(setor_graus) / 2.0
    menor = math.inf
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min <= r <= scan.range_max):      # inf e nan ficam de fora
            continue
        ang = scan.angle_min + i * scan.angle_increment
        ang = math.atan2(math.sin(ang), math.cos(ang))
        if abs(ang) <= meio and r < menor:
            menor = r
    return menor


# ===================== os tres estados =====================
class Estado:
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


# ===================== no base =====================
class No:
    """Todo no de uma Behavior Tree sabe responder a um tick."""

    def __init__(self, nome):
        self.nome = nome

    def tick(self):
        raise NotImplementedError


# ===================== nos de CONTROLE =====================
class Sequence(No):
    """E logico: executa os filhos em ordem, para no primeiro FAILURE ou RUNNING."""

    def __init__(self, nome, filhos):
        super().__init__(nome)
        self.filhos = filhos

    def tick(self):
        for filho in self.filhos:
            st = filho.tick()
            if st in (Estado.FAILURE, Estado.RUNNING):
                return st
        return Estado.SUCCESS


class Fallback(No):
    """OU logico: executa os filhos em ordem, para no primeiro SUCCESS ou RUNNING."""

    def __init__(self, nome, filhos):
        super().__init__(nome)
        self.filhos = filhos

    def tick(self):
        for filho in self.filhos:
            st = filho.tick()
            if st in (Estado.SUCCESS, Estado.RUNNING):
                return st
        return Estado.FAILURE


# ===================== nos de EXECUCAO =====================
class CaminhoLivre(No):
    """Condition: reprova quando o lidar ve um obstaculo novo perto demais no setor frontal."""

    LIVRE = 'LIVRE'              # nada a menos de `parada`: proxima deteccao gera pausa
    BLOQUEADO = 'BLOQUEADO'      # obstaculo detectado: FAILURE ate a pausa terminar
    TRATADO = 'TRATADO'          # pausa feita: SUCCESS ate o obstaculo ficar alem de `rearme`

    def __init__(self, no_ros, parada, rearme, setor_graus):
        super().__init__('CaminhoLivre')
        self.no_ros = no_ros
        self.parada = parada
        self.rearme = rearme
        self.setor_graus = setor_graus
        self.distancia = math.inf
        self.recebeu = False
        self.situacao = self.LIVRE
        no_ros.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.distancia = distancia_frontal(msg, self.setor_graus)
        self.recebeu = True

    def tick(self):
        d = self.distancia
        if self.situacao == self.LIVRE and d < self.parada:
            self.situacao = self.BLOQUEADO
            self.no_ros.get_logger().warn(
                f'{self.nome}: FAILURE  (obstaculo a {d:.2f} m, limite {self.parada:.2f} m)')
        elif self.situacao == self.TRATADO and d >= self.rearme:
            self.situacao = self.LIVRE
            self.no_ros.get_logger().info(
                f'{self.nome}: SUCCESS  (setor livre de novo, {d:.2f} m)')
        if self.situacao == self.BLOQUEADO:
            return Estado.FAILURE
        return Estado.SUCCESS

    def liberar(self):
        """Marca o obstaculo atual como ja tratado (chamado ao fim da pausa)."""
        self.situacao = self.TRATADO if self.distancia < self.rearme else self.LIVRE


class NavegarParaPose(No):
    """Action: mantem um goal NavigateToPose no Nav2; RUNNING ate o Nav2 terminar."""

    def __init__(self, no_ros, acao):
        super().__init__('NavegarParaPose')
        self.no_ros = no_ros
        self.cliente = ActionClient(no_ros, NavigateToPose, acao, callback_group=no_ros.grupo)
        self.trava = threading.Lock()
        self.pose = None
        self.limpar()

    def limpar(self):
        self.handle = None           # goal aceito pelo Nav2 e ainda em curso
        self.enviando = False        # goal enviado, aceite ainda nao chegou
        self.cancelando = False      # halt pedido, resultado ainda nao chegou
        self.desfecho = None         # SUCCESS/FAILURE quando o Nav2 termina por conta propria
        self.envios = 0
        self.feedback = None

    def definir_goal(self, pose):
        with self.trava:
            self.limpar()
            self.pose = pose

    def ativo(self):
        return self.enviando or self.handle is not None

    def tick(self):
        with self.trava:
            if self.desfecho is not None:
                return self.desfecho
            if self.ativo():
                return Estado.RUNNING
            if not self.cliente.server_is_ready():
                self.no_ros.get_logger().info(
                    f'{self.nome}: RUNNING  (aguardando o action server do Nav2)',
                    throttle_duration_sec=5.0)
                return Estado.RUNNING
            self.enviando = True
            self.envios += 1
            envio = self.envios
        goal = NavigateToPose.Goal(pose=self.pose)
        goal.pose.header.stamp = self.no_ros.get_clock().now().to_msg()
        p = self.pose.pose.position
        if envio == 1:
            self.no_ros.get_logger().info(
                f'{self.nome}: RUNNING  (goal ({p.x:.2f}, {p.y:.2f}) enviado ao Nav2)')
        else:
            self.no_ros.get_logger().info(
                f'{self.nome}: RUNNING  (goal ({p.x:.2f}, {p.y:.2f}) retomado, envio {envio})')
        futuro = self.cliente.send_goal_async(goal, feedback_callback=self.ao_feedback)
        futuro.add_done_callback(self.ao_aceite)
        return Estado.RUNNING

    def ao_feedback(self, msg):
        self.feedback = msg.feedback

    def ao_aceite(self, futuro):
        handle = futuro.result()
        with self.trava:
            self.enviando = False
            if not handle.accepted:
                self.desfecho = Estado.FAILURE
                self.no_ros.get_logger().error(f'{self.nome}: FAILURE  (goal recusado pelo Nav2)')
                return
            self.handle = handle
            cancelar = self.cancelando
        handle.get_result_async().add_done_callback(lambda f, h=handle: self.ao_resultado(h, f))
        if cancelar:                 # o halt chegou antes do aceite
            handle.cancel_goal_async()

    def ao_resultado(self, handle, futuro):
        status = futuro.result().status
        with self.trava:
            if handle is not self.handle:        # resultado de um goal antigo
                return
            self.handle = None
            if self.cancelando or status == GoalStatus.STATUS_CANCELED:
                self.cancelando = False          # interrompido: o proximo tick reenvia o goal
                return
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.desfecho = Estado.SUCCESS
                self.no_ros.get_logger().info(f'{self.nome}: SUCCESS  (o Nav2 chegou ao goal)')
            else:
                self.desfecho = Estado.FAILURE
                self.no_ros.get_logger().error(
                    f'{self.nome}: FAILURE  (o Nav2 abortou o goal, status {status})')

    def resetar(self):
        """Halt da Action: cancela o goal em curso no Nav2; a pose da missao continua guardada."""
        with self.trava:
            if not self.ativo() or self.cancelando:
                return
            self.cancelando = True
            handle = self.handle
        if handle is not None:
            handle.cancel_goal_async()
        self.no_ros.get_logger().warn(f'{self.nome}: interrompido - goal cancelado no Nav2')


class InterromperNavegacao(No):
    """Action: manda o Nav2 parar e so conclui com o robo parado."""

    TEMPO_MAX_S = 3.0            # se a odometria nao zerar ate aqui, segue assim mesmo

    def __init__(self, no_ros, navegar):
        super().__init__('InterromperNavegacao')
        self.no_ros = no_ros
        self.navegar = navegar
        self.v = 0.0
        self.w = 0.0
        self.inicio = None
        self.concluido = False
        self.pub = no_ros.create_publisher(Twist, '/cmd_vel', 10)
        no_ros.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def tick(self):
        if self.concluido:
            return Estado.SUCCESS
        agora = self.no_ros.agora()
        if self.inicio is None:
            self.inicio = agora
        self.navegar.resetar()
        andando = abs(self.v) > V_PARADO or abs(self.w) > W_PARADO
        if (self.navegar.ativo() or andando) and agora - self.inicio < self.TEMPO_MAX_S:
            return Estado.RUNNING
        self.pub.publish(Twist())            # um Twist zerado: nada fica comandando as esteiras
        self.concluido = True
        self.no_ros.get_logger().info(
            f'{self.nome}: SUCCESS  (robo parado em {agora - self.inicio:.2f} s)')
        return Estado.SUCCESS

    def rearmar(self):
        self.inicio = None
        self.concluido = False


class Esperar(No):
    """Action: fica RUNNING ate completar `duracao_s` no relogio do no."""

    def __init__(self, no_ros, duracao_s):
        super().__init__('Esperar')
        self.no_ros = no_ros
        self.duracao_s = duracao_s
        self.inicio = None

    def tick(self):
        agora = self.no_ros.agora()
        if self.inicio is None:
            self.inicio = agora
            self.no_ros.get_logger().info(f'{self.nome}: RUNNING  (0.0/{self.duracao_s:.1f} s)')
        decorrido = agora - self.inicio
        if decorrido < self.duracao_s:
            return Estado.RUNNING
        self.no_ros.get_logger().info(f'{self.nome}: SUCCESS  ({decorrido:.1f} s parado)')
        self.inicio = None
        return Estado.SUCCESS

    def rearmar(self):
        self.inicio = None


class RetomarGoal(No):
    """Action: encerra a pausa; a raiz volta a ticar o NavegarParaPose, que reenvia o goal."""

    def __init__(self, no_ros, caminho, interromper):
        super().__init__('RetomarGoal')
        self.no_ros = no_ros
        self.caminho = caminho
        self.interromper = interromper
        self.pausas = 0

    def tick(self):
        self.caminho.liberar()
        self.interromper.rearmar()
        self.pausas += 1
        self.no_ros.get_logger().info(f'{self.nome}: SUCCESS  (pausa {self.pausas} encerrada)')
        return Estado.SUCCESS


# ===================== o no ROS 2 que hospeda a arvore =====================
class ArvoreNavegacao(Node):

    PARAMETROS = {
        'periodo_tick': 0.1,             # s entre ticks
        'distancia_parada_m': 0.5,       # obstaculo mais perto que isso interrompe a navegacao
        'histerese_m': 0.1,              # rearma a condicao quando o setor volta a parada + isto
        'setor_graus': 60.0,             # abertura total do setor frontal do lidar
        'espera_s': 2.0,                 # tempo parado antes de retomar o goal
        'acao': 'navegar_com_pausa',     # action server deste no (entrada do goal)
        'acao_nav2': 'navigate_to_pose',     # action do Nav2 (bt_navigator)
    }

    def __init__(self):
        super().__init__('arvore_navegacao')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)
        p = {nome: self.get_parameter(nome).value for nome in self.PARAMETROS}
        self.grupo = ReentrantCallbackGroup()
        self.trava = threading.Lock()
        self.missao = None               # goal handle do action server em execucao
        self.desfecho = None
        self.fim = threading.Event()
        self.rotulo = None
        self.encerrando = False

        self.caminho = CaminhoLivre(self, p['distancia_parada_m'],
                                    p['distancia_parada_m'] + p['histerese_m'], p['setor_graus'])
        self.navegar = NavegarParaPose(self, p['acao_nav2'])
        self.interromper = InterromperNavegacao(self, self.navegar)
        self.esperar = Esperar(self, p['espera_s'])
        self.retomar = RetomarGoal(self, self.caminho, self.interromper)
        self.raiz = Sequence('Raiz', [
            Fallback('CaminhoSeguro', [
                self.caminho,
                Sequence('PausaPorObstaculo', [self.interromper, self.esperar, self.retomar]),
            ]),
            self.navegar,
        ])

        self.pub_estado = self.create_publisher(String, '/arvore/estado', 10)
        self.pub_distancia = self.create_publisher(Float32, '/arvore/distancia_frontal', 10)
        self.servidor = ActionServer(
            self, NavigateToPose, p['acao'], execute_callback=self.executar,
            goal_callback=self.ao_goal, cancel_callback=self.ao_cancelar,
            callback_group=self.grupo)
        self.create_timer(p['periodo_tick'], self.tick_arvore)
        self.get_logger().info(
            f"Arvore no ar. Tick a cada {p['periodo_tick']:.2f} s; pausa de {p['espera_s']:.1f} s "
            f"com obstaculo a menos de {p['distancia_parada_m']:.2f} m no setor frontal de "
            f"{p['setor_graus']:.0f} graus. Goals em /{p['acao']}.")

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- tick ----------
    def tick_arvore(self):
        self.pub_distancia.publish(Float32(data=float(self.caminho.distancia)))
        with self.trava:
            if self.missao is None or self.fim.is_set():
                self.publicar('OCIOSO')
                return
            if not self.caminho.recebeu:
                self.get_logger().info('aguardando o /scan para comecar',
                                       throttle_duration_sec=5.0)
                self.publicar('OCIOSO')
                return
            st = self.raiz.tick()
            if st == Estado.RUNNING:
                pausado = self.caminho.situacao == CaminhoLivre.BLOQUEADO
                self.publicar('PAUSADO' if pausado else 'NAVEGANDO', st)
                return
            self.publicar('CHEGOU' if st == Estado.SUCCESS else 'FALHOU', st)
            self.desfecho = st
            self.fim.set()

    def publicar(self, rotulo, st=None):
        self.pub_estado.publish(String(data=rotulo))
        if rotulo != self.rotulo:
            self.rotulo = rotulo
            if st is not None:
                self.get_logger().info(f'--- raiz: {st} ---  ({rotulo})')

    # ---------- action server: um goal por vez ----------
    def ao_goal(self, pedido):
        if self.missao is not None:
            self.get_logger().warn('goal recusado: ja ha um goal em curso')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def ao_cancelar(self, goal_handle):
        return CancelResponse.ACCEPT

    def executar(self, goal_handle):
        pose = goal_handle.request.pose
        with self.trava:
            self.navegar.definir_goal(pose)
            self.interromper.rearmar()
            self.esperar.rearmar()
            self.desfecho = None
            self.fim.clear()
            self.missao = goal_handle
        self.get_logger().info(
            f'goal recebido: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) '
            f'no frame {pose.header.frame_id or "map"}')
        enviado = None
        while not self.fim.wait(0.2):
            if self.encerrando or not rclpy.ok():        # no saindo: nao ha a quem responder
                return NavigateToPose.Result()
            if goal_handle.is_cancel_requested:
                with self.trava:
                    self.missao = None
                    self.navegar.resetar()
                goal_handle.canceled()
                self.get_logger().warn('goal cancelado pelo cliente')
                return NavigateToPose.Result()
            fb = self.navegar.feedback
            if fb is not None and fb is not enviado:
                goal_handle.publish_feedback(fb)
                enviado = fb
        with self.trava:
            self.missao = None
            desfecho = self.desfecho
        if desfecho == Estado.SUCCESS:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    no = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        no = ArvoreNavegacao()
        executor.add_node(no)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if no is not None:
            no.encerrando = True         # solta o execute callback que estiver esperando
            no.navegar.resetar()         # nao deixa um goal correndo sozinho no Nav2
            executor.shutdown(timeout_sec=1.0)
            no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
