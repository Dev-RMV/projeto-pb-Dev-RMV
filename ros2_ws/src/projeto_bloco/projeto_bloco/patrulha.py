"""
Patrulha dos postos de tiro: envia goals de pose em sequencia e engaja o alvo de cada posto.

A rota e a lista `postos`; cada posto tem <posto>.x e <posto>.y (metros, frame map),
<posto>.yaw_graus (para onde o robo fica virado) e <posto>.engajar (true = posto de tiro: ao
chegar, arma a torre e envia um goal /engajar_alvo, a action do TP2). Um goal
nav2_msgs/action/NavigateToPose por posto, um de cada vez: o proximo so sai quando o anterior
termina. Goal abortado e repetido ate `tentativas` vezes e depois o posto e pulado.

A action de destino e o parametro `acao`:
  navegar_com_pausa   o no arvore_navegacao (behavior tree: pausa de 2 s com obstaculo a 0,5 m)
  navigate_to_pose    direto no bt_navigator do Nav2, sem a arvore

Antes do primeiro goal espera o Nav2 ficar ativo (lifecycle managers) e o AMCL publicar
map -> odom. Publica /patrulha/rota (nav_msgs/Path, para o RViz e o bag) e /patrulha/status
(String: "indo|chegou|falhou|disparo|sem_disparo <posto> ..." e "fim ..."); com `arquivo_csv`
grava uma linha por posto. Sai com codigo 0 quando todos os postos foram alcancados.

  ros2 run projeto_bloco patrulha --ros-args --params-file config/params.yaml \
      -p use_sim_time:=true -p voltas:=2
"""

import csv
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from projeto_bloco_interfaces.action import EngajarAlvo
from projeto_bloco_interfaces.srv import ControleArma

# rota padrao no sala_patrulha.world: (x, y, yaw em graus, posto de tiro?)
POSTOS_PADRAO = {
    'posto_alvo_2': (1.30, -1.61, 23.0, True),     # 1,3 m a frente do alvo 2 (folha a 23 graus)
    'posto_alvo_1': (0.70, 0.30, 0.0, True),       # 1,5 m a frente do alvo 1
    'posto_alvo_3': (1.44, 1.60, 0.0, True),       # 1,5 m a frente do alvo 3 (parede norte)
    'vigia_oeste': (-0.80, 0.80, 180.0, False),    # ponto de vigia do lado oeste da sala
    'base': (0.00, 0.00, 0.0, False),              # volta para onde nasceu
}
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCESSO', GoalStatus.STATUS_CANCELED: 'CANCELADO',
          GoalStatus.STATUS_ABORTED: 'ABORTADO'}
CAMPOS_CSV = ['data_hora', 'volta', 'posto', 'x', 'y', 'yaw_graus', 'status', 'tentativas',
              'duracao_s', 'erro_posicao_m', 'erro_yaw_graus', 'recuperacoes', 'engajou',
              'disparou', 'tempo_ate_disparo_s']


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Patrulha(Node):

    PARAMETROS = {
        'acao': 'navegar_com_pausa',
        'postos': list(POSTOS_PADRAO),
        'voltas': 1,
        'tentativas': 2,                 # envios do mesmo goal antes de pular o posto
        'espera_posto_s': 2.0,           # parado em cada posto antes de seguir
        'tempo_max_goal_s': 180.0,       # cancela o goal que passar disso
        'tempo_max_nav2_s': 180.0,       # espera pelo Nav2 ativo e pelo AMCL
        'engajar_classe': 'alvo',
        'engajar_tempo_max_s': 10.0,     # campo tempo_max_s do goal /engajar_alvo
        'arquivo_csv': '',               # '' = so log
    }

    def __init__(self):
        super().__init__('patrulha')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)
        self.p = {nome: self.get_parameter(nome).value for nome in self.PARAMETROS}
        self.postos = {}
        for nome in self.p['postos']:
            x, y, yaw, engajar = POSTOS_PADRAO.get(nome, (0.0, 0.0, 0.0, False))
            self.declare_parameter(f'{nome}.x', x)
            self.declare_parameter(f'{nome}.y', y)
            self.declare_parameter(f'{nome}.yaw_graus', yaw)
            self.declare_parameter(f'{nome}.engajar', engajar)
            self.postos[nome] = {c: self.get_parameter(f'{nome}.{c}').value
                                 for c in ('x', 'y', 'yaw_graus', 'engajar')}

        self.nav = ActionClient(self, NavigateToPose, self.p['acao'])
        self.engajamento = ActionClient(self, EngajarAlvo, '/engajar_alvo')
        self.arma = self.create_client(ControleArma, '/arma/controle')
        self.armada = False
        self.tf = Buffer()
        self.ouvinte_tf = TransformListener(self.tf, self)
        fixo = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_rota = self.create_publisher(Path, '/patrulha/rota', fixo)
        self.pub_status = self.create_publisher(String, '/patrulha/status', 10)
        self.create_subscription(Odometry, '/odom', self.ao_odom, 10)
        self.ultima_pos = None
        self.percorrido = 0.0
        self.fb = None
        self.t_log_fb = 0.0

    # ---------- utilitarios ----------
    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def esperar(self, futuro, timeout=None):
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=timeout)
        return futuro.result() if futuro.done() else None

    def dormir(self, segundos):
        """Espera no relogio do no (o do Gazebo com use_sim_time), atendendo os callbacks."""
        fim = self.agora() + segundos
        while rclpy.ok() and self.agora() < fim:
            rclpy.spin_once(self, timeout_sec=0.05)

    def status(self, texto):
        self.pub_status.publish(String(data=texto))

    def ao_odom(self, msg):
        pos = msg.pose.pose.position
        if self.ultima_pos is not None:
            self.percorrido += math.hypot(pos.x - self.ultima_pos[0], pos.y - self.ultima_pos[1])
        self.ultima_pos = (pos.x, pos.y)

    def pose_do(self, nome):
        posto = self.postos[nome]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(posto['x'])
        pose.pose.position.y = float(posto['y'])
        yaw = math.radians(posto['yaw_graus'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def pose_atual(self):
        """Pose do robo no mapa pelo TF map -> base_footprint (None se ainda nao houver)."""
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_de(t.transform.rotation)

    # ---------- preparacao ----------
    def publicar_rota(self):
        rota = Path()
        rota.header.frame_id = 'map'
        rota.header.stamp = self.get_clock().now().to_msg()
        rota.poses = [self.pose_do(nome) for nome in self.p['postos']]
        self.pub_rota.publish(rota)

    def esperar_nav2(self):
        log = self.get_logger()
        limite = time.monotonic() + self.p['tempo_max_nav2_s']
        for gerente in ('lifecycle_manager_localization', 'lifecycle_manager_navigation'):
            cliente = self.create_client(Trigger, f'/{gerente}/is_active')
            ativo = False
            while rclpy.ok() and time.monotonic() < limite and not ativo:
                if not cliente.wait_for_service(timeout_sec=1.0):
                    continue
                resp = self.esperar(cliente.call_async(Trigger.Request()), 5.0)
                ativo = resp is not None and resp.success
                if not ativo:
                    self.dormir(1.0)
            self.destroy_client(cliente)
            if not ativo:
                log.error(f'{gerente} nao ficou ativo em {self.p["tempo_max_nav2_s"]:.0f} s')
                return False
            log.info(f'{gerente}: ativo')
        while rclpy.ok() and time.monotonic() < limite and self.pose_atual() is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        pose = self.pose_atual()
        if pose is None:
            log.error('o AMCL nao publicou map -> odom (falta a pose inicial?)')
            return False
        log.info(f'localizado no mapa: ({pose[0]:.2f}, {pose[1]:.2f}), '
                 f'{math.degrees(pose[2]):.0f} graus')
        if not self.nav.wait_for_server(timeout_sec=30.0):
            log.error(f"action server /{self.p['acao']} nao apareceu em 30 s")
            return False
        return True

    # ---------- patrulha ----------
    def rodar(self):
        log = self.get_logger()
        self.publicar_rota()
        log.info(f"rota: {' -> '.join(self.p['postos'])} ({self.p['voltas']} volta(s)), "
                 f"goals em /{self.p['acao']}")
        if not self.esperar_nav2():
            return 1
        total = len(self.p['postos']) * self.p['voltas']
        chegadas = disparos = postos_de_tiro = n = 0
        inicio = self.agora()
        for volta in range(1, self.p['voltas'] + 1):
            for nome in self.p['postos']:
                n += 1
                linha = self.ir(nome, n, total)
                linha['volta'] = volta
                if linha['status'] == 'SUCESSO':
                    chegadas += 1
                    if self.postos[nome]['engajar']:
                        postos_de_tiro += 1
                        self.engajar(nome, linha)
                        disparos += int(bool(linha['disparou']))
                    self.dormir(self.p['espera_posto_s'])
                self.registrar(linha)
        duracao = self.agora() - inicio
        resumo = (f'{chegadas}/{total} postos alcancados, {disparos}/{postos_de_tiro} disparos, '
                  f'{self.percorrido:.1f} m em {duracao:.0f} s')
        log.info(f'patrulha concluida: {resumo}')
        self.status(f'fim postos={chegadas}/{total} disparos={disparos}/{postos_de_tiro} '
                    f'percorrido_m={self.percorrido:.2f} duracao_s={duracao:.1f}')
        self.dormir(0.5)                 # da tempo de o status sair antes de o no encerrar
        return 0 if chegadas == total else 1

    def ir(self, nome, n, total):
        log = self.get_logger()
        posto = self.postos[nome]
        linha = dict.fromkeys(CAMPOS_CSV, '')
        linha.update(posto=nome, x=posto['x'], y=posto['y'], yaw_graus=posto['yaw_graus'],
                     status='ABORTADO', engajou=False, disparou=False)
        inicio = self.agora()
        for tentativa in range(1, self.p['tentativas'] + 1):
            linha['tentativas'] = tentativa
            log.info(f"[{n}/{total}] indo para {nome}: ({posto['x']:.2f}, {posto['y']:.2f}), "
                     f"{posto['yaw_graus']:.0f} graus"
                     + (f' (tentativa {tentativa})' if tentativa > 1 else ''))
            self.status(f'indo {nome} n={n}/{total} tentativa={tentativa}')
            self.fb = None
            goal = NavigateToPose.Goal(pose=self.pose_do(nome))
            gh = self.esperar(self.nav.send_goal_async(goal, feedback_callback=self.ao_feedback),
                              10.0)
            if gh is None or not gh.accepted:
                log.warn(f'[{n}/{total}] goal recusado')
                linha['status'] = 'REJEITADO'
                self.dormir(2.0)
                continue
            futuro = gh.get_result_async()
            final = self.esperar(futuro, self.p['tempo_max_goal_s'])
            if final is None:
                log.warn(f"[{n}/{total}] passou de {self.p['tempo_max_goal_s']:.0f} s: cancelando")
                self.esperar(gh.cancel_goal_async(), 5.0)
                final = self.esperar(futuro, 10.0)
            linha['status'] = STATUS.get(final.status, 'ABORTADO') if final else 'ABORTADO'
            if self.fb is not None:
                linha['recuperacoes'] = self.fb.number_of_recoveries
            if linha['status'] == 'SUCESSO':
                break
            log.warn(f"[{n}/{total}] {nome}: {linha['status']}")
        linha['duracao_s'] = round(self.agora() - inicio, 2)
        if linha['status'] != 'SUCESSO':
            log.error(f'[{n}/{total}] {nome}: posto pulado')
            self.status(f'falhou {nome}')
            return linha
        pose = self.pose_atual()
        if pose is not None:
            erro_yaw = math.radians(posto['yaw_graus']) - pose[2]
            erro_yaw = math.degrees(math.atan2(math.sin(erro_yaw), math.cos(erro_yaw)))
            linha['erro_posicao_m'] = round(math.hypot(posto['x'] - pose[0],
                                                       posto['y'] - pose[1]), 3)
            linha['erro_yaw_graus'] = round(erro_yaw, 1)
        log.info(f"[{n}/{total}] CHEGOU em {nome} em {linha['duracao_s']:.1f} s "
                 f"(erro {linha['erro_posicao_m']} m, {linha['erro_yaw_graus']} graus)")
        self.status(f"chegou {nome} erro_m={linha['erro_posicao_m']} "
                    f"duracao_s={linha['duracao_s']}")
        return linha

    def ao_feedback(self, msg):
        self.fb = msg.feedback
        agora = time.monotonic()
        if agora - self.t_log_fb >= 2.0:
            self.t_log_fb = agora
            self.get_logger().info(
                f'  feedback: faltam {self.fb.distance_remaining:.2f} m, '
                f'recuperacoes {self.fb.number_of_recoveries}')

    def engajar(self, nome, linha):
        """Posto de tiro: arma a torre (uma vez) e engaja o alvo que a camera ve dali."""
        log = self.get_logger()
        if not self.engajamento.wait_for_server(timeout_sec=5.0):
            log.warn(f'{nome}: /engajar_alvo fora do ar (percepcao desligada?), sem disparo')
            self.status(f'sem_disparo {nome} motivo=sem_servidor')
            return
        if not self.armada and self.arma.wait_for_service(timeout_sec=5.0):
            resp = self.esperar(self.arma.call_async(ControleArma.Request(armar=True)), 5.0)
            self.armada = resp is not None
            log.info(f"/arma/controle: {resp.mensagem if resp else 'sem resposta'}")
        goal = EngajarAlvo.Goal(classe=self.p['engajar_classe'],
                                tempo_max_s=float(self.p['engajar_tempo_max_s']))
        gh = self.esperar(self.engajamento.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            log.warn(f'{nome}: goal de engajamento recusado')
            self.status(f'sem_disparo {nome} motivo=recusado')
            return
        linha['engajou'] = True
        final = self.esperar(gh.get_result_async(), self.p['engajar_tempo_max_s'] + 15.0)
        r = final.result if final is not None else None
        if r is not None and r.disparou:
            linha['disparou'] = True
            linha['tempo_ate_disparo_s'] = round(r.tempo_ate_disparo_s, 2)
            log.info(f'{nome}: DISPARO em {r.tempo_ate_disparo_s:.1f} s '
                     f'(erro final {r.erro_final_px:.1f} px, alvo #{r.id_alvo})')
            self.status(f'disparo {nome} tempo_s={r.tempo_ate_disparo_s:.2f} '
                        f'erro_px={r.erro_final_px:.1f}')
        else:
            log.warn(f"{nome}: sem disparo ({r.mensagem if r is not None else 'sem result'})")
            self.status(f'sem_disparo {nome} motivo=nao_travou')

    def registrar(self, linha):
        arquivo = os.path.expanduser(self.p['arquivo_csv'])
        if not arquivo:
            return
        novo = not os.path.exists(arquivo)
        os.makedirs(os.path.dirname(os.path.abspath(arquivo)), exist_ok=True)
        linha['data_hora'] = datetime.now().isoformat(timespec='seconds')
        with open(arquivo, 'a', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=CAMPOS_CSV)
            if novo:
                w.writeheader()
            w.writerow(linha)


def main(args=None):
    rclpy.init(args=args)
    node = Patrulha()
    rc = 1
    try:
        rc = node.rodar()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
