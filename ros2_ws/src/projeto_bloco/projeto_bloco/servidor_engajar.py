"""
Action server /engajar_alvo: mira a torre pan/tilt num alvo e "dispara" o emissor de luz.

Tarefa central do Robo Sniper (projeto_bloco_interfaces/action/EngajarAlvo). A cada ciclo de
controle (taxa_controle_hz):
  1. escolhe o alvo em /vision/deteccoes: classe pedida e confianca >= confianca_min; o mesmo id
     do ciclo anterior se ainda visivel, senao o mais proximo da mira. Usa o centro filtrado pelo
     Kalman projetado para o instante atual com a velocidade estimada (compensa a latencia) e,
     se o alvo sumir por menos de tempo_perda_s, segue so com essa predicao (sem disparar);
  2. converte o ponto em angulos pan/tilt (camera pinhole com campo de visao hfov_graus; torre e
     camera tratadas como no mesmo ponto) e move a torre na direcao deles: velocidade =
     kp * erro angular + feedforward da velocidade angular do alvo (vinda da velocidade do Kalman;
     sem ele um controle so proporcional fica sempre atrasado de v/kp num alvo em movimento),
     limitada a vel_max_graus_s e aos limites das juntas do URDF;
  3. erro de mira = distancia (px) entre o ponto para onde a torre aponta e o alvo. Com
     erro <= tolerancia_px por tempo_travar_s seguidos: DISPARO (alvo publicado em /arma/disparo).
Feedback a cada ciclo; cancelamento, timeout e desarme encerram o goal com as metricas ate ali.
Um goal por vez: chegando outro com um engajamento em curso, ele e rejeitado.

Servico /arma/controle (ControleArma): trava de seguranca. Desarmada, a action rejeita goals;
desarmar durante um engajamento aborta o goal.
Publica /torre/joint_states (pan/tilt para o robot_state_publisher/RViz) e /torre/visao (imagem da
camera com a mira, o alvo e o estado). Parametros em config/params.yaml; os do goal e do
controle sao lidos a cada goal, entao `ros2 param set` vale a partir do goal seguinte.
"""

import math
import signal
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState

from projeto_bloco_interfaces.action import EngajarAlvo
from projeto_bloco_interfaces.msg import DeteccaoArray, DeteccaoObj
from projeto_bloco_interfaces.srv import ControleArma

JUNTAS = ['torre_pan_joint', 'torre_tilt_joint']      # nomes das juntas no URDF
# imagem anotada: confiavel atende assinantes reliable (RViz) e best effort (rqt_image_view)
QOS_VISUALIZACAO = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
CORES = {'OCIOSO': (160, 160, 160), 'PROCURANDO': (200, 200, 0), 'MIRANDO': (0, 220, 255),
         'TRAVADO': (0, 140, 255), 'DISPARO': (0, 0, 255)}


def limitar(v, lo, hi):
    return max(lo, min(hi, v))


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ServidorEngajar(Node):

    PARAMETROS = {
        'taxa_controle_hz': 10.0,    # ciclos de controle (e de feedback) por segundo
        'hfov_graus': 70.0,          # campo de visao horizontal da camera
        'kp': 4.0,                   # ganho proporcional da torre (1/s)
        'vel_max_graus_s': 90.0,     # velocidade maxima de cada junta
        'pan_limite_graus': 170.0,   # limites das juntas (iguais aos do URDF)
        'tilt_min_graus': -40.0,     # tilt negativo = cano para cima
        'tilt_max_graus': 20.0,
        'tempo_perda_s': 0.5,        # alvo sumido por menos que isso: segue a predicao do Kalman
        'confianca_min': 0.5,        # padroes dos campos do goal enviados com 0
        'tolerancia_px': 15.0,
        'tempo_travar_s': 0.5,
        'tempo_max_s': 30.0,
        'armada_inicial': False,     # trava de seguranca comeca fechada
        'taxa_visao_hz': 10.0,       # publicacao de /torre/visao
        'show': False,               # janela cv2.imshow com a mira
    }

    def __init__(self):
        super().__init__('servidor_engajar')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)

        self.lock = threading.Lock()
        self.pan = self.tilt = 0.0                       # rad
        self.armada = bool(self.param('armada_inicial'))
        self.ocupado = False
        self.deteccoes = None                            # ultimo DeteccaoArray
        self.frame = None                                # ultimo Image da camera
        self.hud = {'estado': 'OCIOSO', 'alvo': None, 'uv': None, 'erro': -1.0,
                    'progresso': 0.0, 't_disparo': -1e9}
        self.bridge = CvBridge()
        self.img_janela = None                           # mostrada pela thread principal (main)

        self.create_subscription(DeteccaoArray, '/vision/deteccoes', self.ao_receber_deteccoes, 10)
        self.create_subscription(Image, '/camera/image_raw', self.ao_receber_frame,
                                 qos_profile_sensor_data)
        self.pub_juntas = self.create_publisher(JointState, '/torre/joint_states', 10)
        self.pub_visao = self.create_publisher(Image, '/torre/visao', QOS_VISUALIZACAO)
        self.pub_disparo = self.create_publisher(DeteccaoObj, '/arma/disparo', 10)
        self.create_service(ControleArma, '/arma/controle', self.controlar_arma)
        self.create_timer(0.05, self.publicar_juntas)
        self.create_timer(1.0 / self.param('taxa_visao_hz'), self.publicar_visao)
        self.action = ActionServer(
            self, EngajarAlvo, '/engajar_alvo', execute_callback=self.executar,
            goal_callback=self.ao_receber_goal, cancel_callback=self.ao_pedir_cancelamento,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            '/engajar_alvo e /arma/controle prontos; arma '
            f"{'ARMADA' if self.armada else 'DESARMADA'} (ros2 service call /arma/controle "
            'projeto_bloco_interfaces/srv/ControleArma "{armar: true}")')

    def param(self, nome):
        return self.get_parameter(nome).value

    # ---------- entradas ----------
    def ao_receber_deteccoes(self, msg):
        with self.lock:
            self.deteccoes = msg

    def ao_receber_frame(self, msg):
        self.frame = msg

    def controlar_arma(self, req, resp):
        with self.lock:
            self.armada = bool(req.armar)
            ocupado = self.ocupado
        resp.sucesso = True
        resp.armada = self.armada
        if self.armada:
            resp.mensagem = 'arma ARMADA: goals de /engajar_alvo liberados'
        else:
            resp.mensagem = 'arma DESARMADA' + (': engajamento em curso sera abortado'
                                                if ocupado else '')
        self.get_logger().info(f'/arma/controle -> {resp.mensagem}')
        return resp

    # ---------- action ----------
    def ao_receber_goal(self, goal):
        with self.lock:
            if not self.armada:
                motivo = 'arma desarmada (chame /arma/controle com armar: true)'
            elif self.ocupado:
                motivo = 'ja existe um engajamento em curso'
            else:
                self.ocupado = True
                motivo = None
        if motivo:
            self.get_logger().warn(f'goal "{goal.classe}" REJEITADO: {motivo}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def ao_pedir_cancelamento(self, goal_handle):
        self.get_logger().info('cancelamento pedido pelo cliente: aceito')
        return CancelResponse.ACCEPT

    def geometria(self, largura, altura):
        """Distancia focal (px) do modelo pinhole e centro da imagem."""
        f = (largura / 2) / math.tan(math.radians(self.param('hfov_graus')) / 2)
        return f, largura / 2, altura / 2

    def mira_px(self, pan, tilt, f, u0, v0):
        """Pixel para onde a torre aponta (pan + = esquerda, tilt + = baixo, como no URDF)."""
        return u0 - f * math.tan(pan), v0 + f * math.tan(tilt)

    @staticmethod
    def escolher_alvo(dets, classe, conf_min, id_atual, mira):
        cands = [d for d in dets.deteccoes
                 if (not classe or d.classe == classe) and d.confianca >= conf_min]
        for d in cands:
            if d.id == id_atual:
                return d
        return min(cands, key=lambda d: math.hypot(d.cx - mira[0], d.cy - mira[1]),
                   default=None)

    def executar(self, gh):
        g = gh.request
        classe = g.classe
        conf_min = g.confianca_min or self.param('confianca_min')
        tol = g.tolerancia_px or self.param('tolerancia_px')
        t_travar = g.tempo_travar_s or self.param('tempo_travar_s')
        t_max = g.tempo_max_s or self.param('tempo_max_s')
        dt = 1.0 / self.param('taxa_controle_hz')
        kp, vmax = self.param('kp'), math.radians(self.param('vel_max_graus_s'))
        pan_lim = math.radians(self.param('pan_limite_graus'))
        tilt_lim = (math.radians(self.param('tilt_min_graus')),
                    math.radians(self.param('tilt_max_graus')))
        t_perda = self.param('tempo_perda_s')
        salto_px = max(4 * tol, 40.0)
        self.get_logger().info(
            f'goal aceito: classe="{classe or "qualquer"}" confianca>={conf_min:.2f} '
            f'tolerancia {tol:.0f} px, travar {t_travar:.1f} s, limite {t_max:.0f} s')

        res = EngajarAlvo.Result(classe=classe, id_alvo=-1, tempo_ate_deteccao_s=-1.0,
                                 tempo_ate_disparo_s=-1.0)
        fb = EngajarAlvo.Feedback()
        t0 = proximo = time.monotonic()
        id_alvo, ultimo = -1, None          # ultimo = (DeteccaoObj, stamp, instante visto)
        uv = None                           # posicao (px) do alvo no ciclo anterior
        travado_desde = erro_ref = None
        soma_erro, erro = 0.0, -1.0
        fim = None                          # (status, mensagem)
        try:
            while rclpy.ok() and fim is None:
                agora = time.monotonic()
                decorrido = agora - t0
                if gh.is_cancel_requested:
                    fim = ('cancelado', f'cancelado pelo cliente apos {decorrido:.1f} s')
                    break
                if not self.armada:
                    fim = ('abortado', 'arma desarmada durante o engajamento')
                    break
                if decorrido > t_max:
                    fim = ('abortado', f'tempo esgotado ({t_max:.0f} s) sem disparar')
                    break

                with self.lock:
                    dets, pan, tilt = self.deteccoes, self.pan, self.tilt
                alvo = visto = None
                if dets is not None and dets.largura:
                    f, u0, v0 = self.geometria(dets.largura, dets.altura)
                    visto = self.escolher_alvo(dets, classe, conf_min, id_alvo,
                                               self.mira_px(pan, tilt, f, u0, v0))
                    if visto is not None:
                        ultimo = (visto, segundos(dets.header.stamp), agora)
                    if ultimo is not None and agora - ultimo[2] <= t_perda:
                        alvo = ultimo[0]
                if alvo is None:
                    estado, progresso, erro = 'PROCURANDO', 0.0, -1.0
                    travado_desde = erro_ref = uv = None
                    id_alvo, ultimo = -1, None
                else:
                    if res.tempo_ate_deteccao_s < 0:
                        res.tempo_ate_deteccao_s = decorrido
                    if alvo.id != id_alvo:
                        # o ByteTrack as vezes troca o id do mesmo objeto: id novo perto do alvo
                        # anterior herda a trava; longe dele e outro alvo e a trava recomeca
                        if uv is None or math.hypot(alvo.cx - uv[0], alvo.cy - uv[1]) > salto_px:
                            travado_desde = erro_ref = None
                        id_alvo = alvo.id
                    # latencia camera -> agora; limitada porque extrapolar muito diverge (quique)
                    lat = limitar(segundos(self.get_clock().now().to_msg()) - ultimo[1], 0.0, 0.25)
                    u, v = uv = alvo.cx + alvo.vx * lat, alvo.cy + alvo.vy * lat
                    xn, yn = (u - u0) / f, (v - v0) / f      # coordenadas normalizadas
                    pan_alvo, tilt_alvo = -math.atan(xn), math.atan(yn)
                    ff_pan = -(alvo.vx / f) / (1 + xn * xn)    # d(pan_alvo)/dt
                    ff_tilt = (alvo.vy / f) / (1 + yn * yn)    # d(tilt_alvo)/dt
                    w_pan = limitar(kp * (pan_alvo - pan) + ff_pan, -vmax, vmax)
                    w_tilt = limitar(kp * (tilt_alvo - tilt) + ff_tilt, -vmax, vmax)
                    pan = limitar(pan + w_pan * dt, -pan_lim, pan_lim)
                    tilt = limitar(tilt + w_tilt * dt, *tilt_lim)
                    with self.lock:
                        self.pan, self.tilt = pan, tilt
                    mu, mv = self.mira_px(pan, tilt, f, u0, v0)
                    erro = math.hypot(mu - u, mv - v)
                    res.ciclos_com_alvo += 1
                    soma_erro += erro
                    if erro_ref is None:
                        erro_ref = max(erro, 2 * tol)
                    if erro <= tol:
                        travado_desde = travado_desde or agora
                        frac = (agora - travado_desde) / t_travar
                        estado, progresso = 'TRAVADO', 0.5 + 0.5 * min(frac, 1.0)
                        if frac >= 1.0 and visto is not None:   # so dispara com o alvo a vista
                            estado, progresso = 'DISPARO', 1.0
                            res.disparou = True
                            res.tempo_ate_disparo_s = decorrido
                            self.pub_disparo.publish(DeteccaoObj(
                                classe=alvo.classe, confianca=alvo.confianca, x=alvo.x, y=alvo.y,
                                w=alvo.w, h=alvo.h, id=alvo.id, cx=float(u), cy=float(v),
                                vx=alvo.vx, vy=alvo.vy))
                            self.hud['t_disparo'] = agora
                            fim = ('sucesso', f'DISPARO em {alvo.classe} #{alvo.id} com erro '
                                              f'{erro:.1f} px apos {decorrido:.1f} s')
                    else:
                        travado_desde = None
                        melhora = 1.0 - (erro - tol) / max(erro_ref - tol, 1e-6)
                        estado, progresso = 'MIRANDO', 0.1 + 0.4 * limitar(melhora, 0.0, 1.0)

                res.ciclos += 1
                res.id_alvo = id_alvo if id_alvo >= 0 else res.id_alvo
                if alvo is not None:
                    res.classe = alvo.classe
                fb.estado, fb.progresso, fb.id_alvo = estado, float(progresso), id_alvo
                fb.erro_px, fb.tempo_decorrido_s = float(erro), float(decorrido)
                fb.pan_graus, fb.tilt_graus = math.degrees(pan), math.degrees(tilt)
                gh.publish_feedback(fb)
                self.hud.update(estado=estado, alvo=alvo, uv=uv, erro=erro, progresso=progresso)
                proximo += dt
                time.sleep(max(0.0, proximo - time.monotonic()))

            if fim is None:
                fim = ('abortado', 'no encerrado durante o engajamento')
            status, res.mensagem = fim
            res.duracao_s = time.monotonic() - t0
            res.erro_medio_px = soma_erro / res.ciclos_com_alvo if res.ciclos_com_alvo else -1.0
            res.erro_final_px = float(erro)
            res.pan_final_graus = math.degrees(self.pan)
            res.tilt_final_graus = math.degrees(self.tilt)
            {'sucesso': gh.succeed, 'cancelado': gh.canceled, 'abortado': gh.abort}[status]()
            resumo = (f'goal {status}: {res.mensagem} | ciclos {res.ciclos} (com alvo '
                      f'{res.ciclos_com_alvo}), erro medio {res.erro_medio_px:.1f} px')
            if status == 'sucesso':      # o rclpy nao aceita trocar a severidade na mesma linha
                self.get_logger().info(resumo)
            else:
                self.get_logger().warn(resumo)
            return res
        finally:
            with self.lock:
                self.ocupado = False
            if not res.disparou:
                self.hud.update(estado='OCIOSO', alvo=None, uv=None, erro=-1.0, progresso=0.0)

    # ---------- saidas continuas ----------
    def publicar_juntas(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JUNTAS
        with self.lock:
            js.position = [self.pan, self.tilt]
        self.pub_juntas.publish(js)

    def publicar_visao(self):
        msg = self.frame
        if msg is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
        alt, larg = img.shape[:2]
        f, u0, v0 = self.geometria(larg, alt)
        with self.lock:
            pan, tilt, armada = self.pan, self.tilt, self.armada
        hud = dict(self.hud)
        disparo = time.monotonic() - hud['t_disparo'] < 1.0
        estado = 'DISPARO' if disparo else hud['estado']
        if not disparo and hud['estado'] == 'DISPARO':      # passou o flash: volta a ocioso
            estado = 'OCIOSO'
            self.hud.update(estado='OCIOSO', alvo=None, uv=None, erro=-1.0, progresso=0.0)
            hud['alvo'] = None
        cor = CORES[estado]
        mu, mv = (int(round(c)) for c in self.mira_px(pan, tilt, f, u0, v0))

        a = hud['alvo']
        if a is not None and hud['uv'] is not None:
            # caixa centrada na posicao prevista (a deteccao chega com ~100 ms de atraso)
            u, v = (int(c) for c in hud['uv'])
            x, y = u - a.w // 2, v - a.h // 2
            cv2.rectangle(img, (x, y), (x + a.w, y + a.h), cor, 2)
            cv2.line(img, (mu, mv), (u, v), cor, 1, cv2.LINE_AA)
            cv2.putText(img, f'{a.classe} #{a.id}', (x, max(y - 6, 40)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, cor, 2, cv2.LINE_AA)
        if disparo and a is not None:                       # feixe do emissor ate o alvo
            cv2.line(img, (larg // 2, alt), (mu, mv), (0, 0, 255), 4, cv2.LINE_AA)
            cv2.circle(img, (mu, mv), 30, (255, 255, 255), 3, cv2.LINE_AA)
        mira_cor = (0, 0, 255) if armada else (160, 160, 160)
        cv2.circle(img, (mu, mv), 18, mira_cor, 2, cv2.LINE_AA)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cv2.line(img, (mu + 8 * dx, mv + 8 * dy), (mu + 28 * dx, mv + 28 * dy), mira_cor, 2)

        erro = f"{hud['erro']:.1f} px" if hud['erro'] >= 0 else '-'
        texto = (f"ARMA {'ARMADA' if armada else 'DESARMADA'} | {estado} | erro {erro} | "
                 f'pan {math.degrees(pan):+.1f}  tilt {math.degrees(tilt):+.1f}')
        cv2.rectangle(img, (0, 0), (larg, 30), (0, 0, 0), -1)
        cv2.putText(img, texto, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, cor, 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, alt - 12), (int(larg * hud['progresso']), alt), cor, -1)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = msg.header
        self.pub_visao.publish(out)
        self.img_janela = img if self.param('show') else None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServidorEngajar()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        # O executor roda em outra thread e a janela fica na principal: o HighGUI (Qt) do OpenCV
        # trava se cv2.imshow for chamado das varias threads do MultiThreadedExecutor, e com ele
        # o grupo de callbacks do servico /arma/controle.
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        while rclpy.ok() and spin.is_alive():
            img = node.img_janela
            if img is None:
                time.sleep(0.05)
            else:
                cv2.imshow('mira', img)
                cv2.waitKey(30)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (launch) nao interrompe a limpeza
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
