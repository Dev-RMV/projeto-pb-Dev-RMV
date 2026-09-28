"""
Action client de /engajar_alvo: envia goals, mostra cada feedback e registra o result.

Arma a torre (servico /arma/controle), envia um goal por classe da lista `classes`, mostra cada
feedback recebido e registra o result final (no log e, se `arquivo_csv` for dado, uma linha por
goal no CSV).

Parametros (config/params.yaml ou `-p nome:=valor`):
  classes           um goal por item, em sequencia (ex.: ["alvo"])
  confianca_min, tolerancia_px, tempo_travar_s, tempo_max_s
                    campos do goal (0 = padrao do servidor)
  cancelar_apos_s   > 0: pede o cancelamento de cada goal depois desse tempo (demonstra o cancel)
  armar             chama /arma/controle {armar: true} antes dos goals (false: testa a rejeicao)
  intervalo_s       pausa entre um goal e o seguinte
  arquivo_csv       '' = so log

  ros2 run projeto_bloco cliente_engajar --ros-args \
      -p tempo_travar_s:=5.0 -p cancelar_apos_s:=2.0
"""

import csv
import os
import sys
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from projeto_bloco_interfaces.action import EngajarAlvo
from projeto_bloco_interfaces.srv import ControleArma

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCESSO', GoalStatus.STATUS_CANCELED: 'CANCELADO',
          GoalStatus.STATUS_ABORTED: 'ABORTADO'}
CAMPOS_RESULT = ['disparou', 'mensagem', 'id_alvo', 'classe', 'duracao_s', 'tempo_ate_deteccao_s',
                 'tempo_ate_disparo_s', 'ciclos', 'ciclos_com_alvo', 'erro_medio_px',
                 'erro_final_px', 'pan_final_graus', 'tilt_final_graus']


class ClienteEngajar(Node):

    PARAMETROS = {
        'classes': ['alvo'],
        'confianca_min': 0.0,
        'tolerancia_px': 0.0,
        'tempo_travar_s': 0.0,
        'tempo_max_s': 0.0,
        'cancelar_apos_s': 0.0,
        'armar': True,
        'intervalo_s': 2.0,
        'arquivo_csv': '',
    }

    def __init__(self):
        super().__init__('cliente_engajar')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)
        self.p = {nome: self.get_parameter(nome).value for nome in self.PARAMETROS}
        self.action = ActionClient(self, EngajarAlvo, '/engajar_alvo')
        self.arma = self.create_client(ControleArma, '/arma/controle')
        self.n_feedback = 0

    def esperar(self, future, timeout=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def rodar(self):
        log = self.get_logger()
        if not self.action.wait_for_server(timeout_sec=30.0):
            log.error('action server /engajar_alvo nao apareceu em 30 s')
            return 1
        if self.p['armar']:
            if not self.arma.wait_for_service(timeout_sec=10.0):
                log.error('servico /arma/controle indisponivel')
                return 1
            resp = self.esperar(self.arma.call_async(ControleArma.Request(armar=True)), 5.0)
            log.info(f"/arma/controle: {resp.mensagem if resp else 'sem resposta'}")
        for i, classe in enumerate(self.p['classes'], start=1):
            if i > 1:
                time.sleep(self.p['intervalo_s'])
            self.engajar(i, classe)
        return 0

    def engajar(self, i, classe):
        log = self.get_logger()
        goal = EngajarAlvo.Goal(classe=classe, confianca_min=self.p['confianca_min'],
                                tolerancia_px=self.p['tolerancia_px'],
                                tempo_travar_s=self.p['tempo_travar_s'],
                                tempo_max_s=self.p['tempo_max_s'])
        log.info(f'[goal {i}] enviando: classe="{classe}"'
                 + (f", cancelar apos {self.p['cancelar_apos_s']:.1f} s"
                    if self.p['cancelar_apos_s'] > 0 else ''))
        self.n_feedback = 0
        gh = self.esperar(self.action.send_goal_async(goal, feedback_callback=self.ao_feedback),
                          10.0)
        if gh is None or not gh.accepted:
            log.warn(f'[goal {i}] REJEITADO pelo servidor (arma desarmada ou outro goal em curso)')
            self.registrar(i, classe, 'REJEITADO', None)
            return
        log.info(f'[goal {i}] aceito')
        futuro = gh.get_result_async()
        if self.p['cancelar_apos_s'] > 0:
            self.esperar(futuro, self.p['cancelar_apos_s'])
            if not futuro.done():
                log.info(f'[goal {i}] pedindo cancelamento')
                resp = self.esperar(gh.cancel_goal_async(), 5.0)
                aceito = resp is not None and len(resp.goals_canceling) > 0
                log.info(f"[goal {i}] cancelamento {'aceito' if aceito else 'recusado'}")
        final = self.esperar(futuro)
        status = STATUS.get(final.status, f'status {final.status}')
        r = final.result
        log.info(
            f'[goal {i}] RESULT {status}: {r.mensagem}\n'
            f'    disparou={r.disparou}  alvo={r.classe or "-"} #{r.id_alvo}  '
            f'feedbacks recebidos={self.n_feedback}\n'
            f'    duracao {r.duracao_s:.2f} s | 1a deteccao {r.tempo_ate_deteccao_s:.2f} s | '
            f'disparo {r.tempo_ate_disparo_s:.2f} s\n'
            f'    ciclos {r.ciclos} (com alvo {r.ciclos_com_alvo}) | erro medio '
            f'{r.erro_medio_px:.1f} px, final {r.erro_final_px:.1f} px | torre pan '
            f'{r.pan_final_graus:+.1f} tilt {r.tilt_final_graus:+.1f} graus')
        self.registrar(i, classe, status, r)

    def ao_feedback(self, msg):
        fb = msg.feedback
        self.n_feedback += 1
        erro = f'{fb.erro_px:6.1f} px' if fb.erro_px >= 0 else '     - px'
        self.get_logger().info(
            f'  feedback {self.n_feedback:3d}: {fb.estado:<10} {100 * fb.progresso:5.1f}%  '
            f'erro {erro}  pan {fb.pan_graus:+6.1f}  tilt {fb.tilt_graus:+6.1f}  '
            f'id {fb.id_alvo:3d}  t={fb.tempo_decorrido_s:4.1f} s')

    def registrar(self, i, classe_pedida, status, r):
        arquivo = os.path.expanduser(self.p['arquivo_csv'])
        if not arquivo:
            return
        novo = not os.path.exists(arquivo)
        os.makedirs(os.path.dirname(os.path.abspath(arquivo)), exist_ok=True)
        with open(arquivo, 'a', newline='') as fh:
            w = csv.writer(fh)
            if novo:
                w.writerow(['data_hora', 'goal', 'classe_pedida', 'status', 'feedbacks']
                           + CAMPOS_RESULT)
            valores = [getattr(r, c) for c in CAMPOS_RESULT] if r is not None else []
            valores = [round(v, 3) if isinstance(v, float) else v for v in valores]
            w.writerow([datetime.now().isoformat(timespec='seconds'), i, classe_pedida, status,
                        self.n_feedback] + valores)
        self.get_logger().info(f'result registrado em {arquivo}')


def main(args=None):
    rclpy.init(args=args)
    node = ClienteEngajar()
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
