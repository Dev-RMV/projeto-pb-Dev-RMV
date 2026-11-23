"""
Analise offline da navegacao: ouve a reproducao de um bag e valida a pausa por obstaculo.

Roda SEM o Gazebo e sem o Nav2: os dados vem do `ros2 bag play` da sessao gravada. O no assina
/scan, /odom, /cmd_vel, /amcl_pose, /arvore/estado, /patrulha/status, /patrulha/rota e
/obstaculo_surpresa/status e, quando a reproducao termina (`silencio_s` sem mensagens), confere:

  1. cada vez que o lidar passou a ver um obstaculo a menos de `distancia_parada_m` no setor
     frontal (mesma conta e mesma histerese da arvore_navegacao), o robo parou em ate
     `reacao_max_s` e ficou parado pelo menos `espera_s` (velocidades do /odom) antes de retomar;
  2. o estado PAUSADO publicado pela arvore cobre esses mesmos intervalos;
  3. a patrulha alcancou todos os postos (mensagens "chegou" e "fim" do /patrulha/status).

Saidas em `pasta`: trajetoria.png (caminho do /odom e do /amcl_pose sobre o mapa, postos, pausas
e os obstaculos fora do mapa), pausa_obstaculo.png (distancia frontal, velocidade e estado da
arvore no tempo), validacao.txt e validacao.json. Codigo de saida 0 = tudo conferido.

  ros2 run projeto_bloco analise_bag --ros-args -p pasta:=log/tp4 &
  ros2 bag play log/tp4/bag_patrulha --clock
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from projeto_bloco.arvore_navegacao import distancia_frontal, V_PARADO, W_PARADO


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AnaliseBag(Node):

    PARAMETROS = {
        'pasta': 'analise_bag',
        'mapa': '',                      # yaml do mapa; '' = maps/sala_alvos.yaml do pacote
        'distancia_parada_m': 0.5,       # os mesmos valores da arvore_navegacao
        'histerese_m': 0.1,
        'setor_graus': 60.0,
        'espera_s': 2.0,
        'reacao_max_s': 1.5,             # da deteccao ate o robo parado
        'pausas_min': 1,                 # quantas pausas a sessao precisa ter para valer
        'silencio_s': 5.0,               # sem mensagens por esse tempo = reproducao terminou
        'espera_max_s': 120.0,           # desiste se nada chegar
        # obstaculos que nao estao no mapa, so para desenhar no grafico: "nome x y lado"
        'fora_do_mapa': ['caixote_novo 0.60 -0.75 0.30'],
    }

    def __init__(self):
        super().__init__('analise_bag')
        for nome, padrao in self.PARAMETROS.items():
            self.declare_parameter(nome, padrao)
        self.p = {nome: self.get_parameter(nome).value for nome in self.PARAMETROS}
        self.scan = []           # (t, distancia frontal)
        self.odom = []           # (t, x, y, yaw, v, w)
        self.amcl = []           # (t, x, y)
        self.cmd = []            # (t, v, w)
        self.estados = []        # (t, rotulo)
        self.status = []         # (t, texto)
        self.surpresa = []       # (t, texto)
        self.rota = []           # (x, y)
        self.t_ultima = None     # relogio de parede da ultima mensagem
        self.t_dados = 0.0       # carimbo mais recente visto (tempo da gravacao)
        self.create_subscription(LaserScan, '/scan', self.ao_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.ao_odom, 50)
        self.create_subscription(Twist, '/cmd_vel', self.ao_cmd, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.ao_amcl, 10)
        self.create_subscription(String, '/arvore/estado', self.ao_estado, 50)
        self.create_subscription(String, '/patrulha/status', self.ao_status, 50)
        self.create_subscription(String, '/obstaculo_surpresa/status', self.ao_surpresa, 10)
        self.create_subscription(Path, '/patrulha/rota', self.ao_rota, 10)

    # ---------- coleta ----------
    def chegou(self, t=None):
        self.t_ultima = time.monotonic()
        if t is not None:
            self.t_dados = max(self.t_dados, t)

    def ao_scan(self, msg):
        t = segundos(msg.header.stamp)
        self.scan.append((t, distancia_frontal(msg, self.p['setor_graus'])))
        self.chegou(t)

    def ao_odom(self, msg):
        t = segundos(msg.header.stamp)
        pos = msg.pose.pose.position
        self.odom.append((t, pos.x, pos.y, yaw_de(msg.pose.pose.orientation),
                          msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        self.chegou(t)

    def ao_amcl(self, msg):
        t = segundos(msg.header.stamp)
        self.amcl.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.chegou(t)

    # mensagens sem carimbo: valem o carimbo mais recente ja visto (scan a 10 Hz, odom a 30 Hz)
    def ao_cmd(self, msg):
        self.cmd.append((self.t_dados, msg.linear.x, msg.angular.z))
        self.chegou()

    def ao_estado(self, msg):
        self.estados.append((self.t_dados, msg.data))
        self.chegou()

    def ao_status(self, msg):
        self.status.append((self.t_dados, msg.data))
        self.chegou()

    def ao_surpresa(self, msg):
        self.surpresa.append((self.t_dados, msg.data))
        self.chegou()

    def ao_rota(self, msg):
        self.rota = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.chegou()

    # ---------- analise ----------
    def intervalos(self, rotulo):
        """Intervalos (inicio, fim) em que /arvore/estado ficou em `rotulo`."""
        saida, inicio = [], None
        for t, r in self.estados:
            if r == rotulo and inicio is None:
                inicio = t
            elif r != rotulo and inicio is not None:
                saida.append((inicio, t))
                inicio = None
        if inicio is not None:
            saida.append((inicio, self.estados[-1][0]))
        return saida

    def estado_em(self, t):
        rotulo = ''
        for te, r in self.estados:
            if te > t:
                break
            rotulo = r
        return rotulo

    def pausas(self):
        """Deteccoes de obstaculo novo no /scan e o que o robo fez depois de cada uma."""
        parada = self.p['distancia_parada_m']
        rearme = parada + self.p['histerese_m']
        odom = self.odom
        pausados = self.intervalos('PAUSADO')
        eventos, armado = [], True
        for t, d in self.scan:
            if armado and d < parada:
                armado = False
                eventos.append({'t': t, 'distancia_m': round(d, 3)})
            elif not armado and d >= rearme:
                armado = True
        for ev in eventos:
            ev['estado_arvore'] = self.estado_em(ev['t'])
            parou = retomou = None
            for i, (t, x, y, _, v, w) in enumerate(odom):
                if t < ev['t']:
                    continue
                parado = abs(v) <= V_PARADO and abs(w) <= W_PARADO
                if parou is None:
                    if parado:
                        parou = (t, x, y)
                    continue
                # retomada = 3 amostras seguidas em movimento (ruido de uma amostra nao conta)
                trecho = odom[i:i + 3]
                if len(trecho) == 3 and all(abs(a[4]) > V_PARADO or abs(a[5]) > W_PARADO
                                            for a in trecho):
                    retomou = t
                    break
            ev['reacao_s'] = round(parou[0] - ev['t'], 2) if parou else None
            ev['parado_s'] = round(retomou - parou[0], 2) if parou and retomou else None
            ev['x'], ev['y'] = (round(parou[1], 3), round(parou[2], 3)) if parou else (None, None)
            ev['t_parou'] = parou[0] if parou else None
            ev['t_retomou'] = retomou
            # intervalo PAUSADO da arvore que comeca logo depois da deteccao
            ev['pausado_arvore_s'] = next((round(b - a, 2) for a, b in pausados
                                           if ev['t'] - 0.5 <= a <= ev['t'] + 2.0), None)
            em_missao = ev['estado_arvore'] not in ('', 'OCIOSO')
            ev['em_missao'] = em_missao
            ev['ok'] = bool(
                em_missao and ev['reacao_s'] is not None and ev['parado_s'] is not None
                and ev['reacao_s'] <= self.p['reacao_max_s']
                and ev['parado_s'] >= self.p['espera_s'] - 0.05)
        return eventos

    def erro_amcl(self):
        """Distancia entre o /amcl_pose e o /odom (pose real no Gazebo) no mesmo instante."""
        if len(self.amcl) < 2 or len(self.odom) < 2:
            return None
        to = np.array([o[0] for o in self.odom])
        xo = np.array([o[1] for o in self.odom])
        yo = np.array([o[2] for o in self.odom])
        erros = [math.hypot(x - np.interp(t, to, xo), y - np.interp(t, to, yo))
                 for t, x, y in self.amcl if to[0] <= t <= to[-1]]
        if not erros:
            return None
        return {'amostras': len(erros), 'medio_m': round(float(np.mean(erros)), 3),
                'max_m': round(float(np.max(erros)), 3)}

    def analisar(self):
        log = self.get_logger()
        pasta = os.path.abspath(os.path.expanduser(self.p['pasta']))
        os.makedirs(pasta, exist_ok=True)
        if len(self.odom) < 10 or len(self.scan) < 10:
            log.error(f'dados insuficientes: {len(self.scan)} scans, {len(self.odom)} odometrias')
            return 1
        eventos = self.pausas()
        validos = [e for e in eventos if e['em_missao']]
        t0 = self.odom[0][0]
        percorrido = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                         for a, b in zip(self.odom, self.odom[1:]))
        chegadas = [s for _, s in self.status if s.startswith('chegou ')]
        falhas = [s for _, s in self.status if s.startswith('falhou ')]
        disparos = [s for _, s in self.status if s.startswith('disparo ')]
        fim = next((s for _, s in self.status if s.startswith('fim ')), '')
        postos = dict(c.split('=') for c in fim.split()[1:]).get('postos', '') if fim else ''
        ok_pausas = len(validos) >= self.p['pausas_min'] and all(e['ok'] for e in validos)
        feitos, _, previstos = postos.partition('/')
        ok_postos = (not falhas) and (postos == '' or feitos == previstos)
        ok_arvore = all(e['pausado_arvore_s'] is not None
                        and e['pausado_arvore_s'] >= self.p['espera_s'] - 0.05 for e in validos)
        resumo = {
            'duracao_s': round(self.odom[-1][0] - t0, 1),
            'percorrido_m': round(percorrido, 2),
            'velocidade_max_m_s': round(max(abs(o[4]) for o in self.odom), 3),
            'mensagens': {'scan': len(self.scan), 'odom': len(self.odom), 'cmd_vel': len(self.cmd),
                          'amcl_pose': len(self.amcl), 'arvore_estado': len(self.estados),
                          'patrulha_status': len(self.status)},
            'postos_alcancados': len(chegadas), 'postos_falhos': len(falhas),
            'disparos': len(disparos), 'fim_da_patrulha': fim,
            'erro_amcl': self.erro_amcl(),
            'pausas': [{c: e[c] for c in ('distancia_m', 'reacao_s', 'parado_s',
                                          'pausado_arvore_s', 'x', 'y', 'estado_arvore', 'ok')}
                       | {'t_s': round(e['t'] - t0, 2)} for e in eventos],
            'criterios': {'distancia_parada_m': self.p['distancia_parada_m'],
                          'espera_s': self.p['espera_s'], 'reacao_max_s': self.p['reacao_max_s'],
                          'setor_graus': self.p['setor_graus'],
                          'pausas_min': self.p['pausas_min']},
            'validado': {'pausa_por_obstaculo': ok_pausas, 'estado_da_arvore': ok_arvore,
                         'postos': ok_postos},
        }
        aprovado = ok_pausas and ok_arvore and ok_postos
        resumo['aprovado'] = aprovado
        with open(os.path.join(pasta, 'validacao.json'), 'w') as fh:
            json.dump(resumo, fh, indent=2, ensure_ascii=False)
        linhas = self.texto(resumo)
        with open(os.path.join(pasta, 'validacao.txt'), 'w') as fh:
            fh.write('\n'.join(linhas) + '\n')
        for linha in linhas:
            log.info(linha)
        self.grafico_trajetoria(os.path.join(pasta, 'trajetoria.png'), eventos, resumo)
        self.grafico_pausa(os.path.join(pasta, 'pausa_obstaculo.png'), eventos, t0)
        log.info(f'saidas em {pasta}: trajetoria.png, pausa_obstaculo.png, validacao.txt/json')
        return 0 if aprovado else 1

    def texto(self, r):
        sim = {True: 'OK', False: 'FALHOU'}
        linhas = [
            'Validacao offline da sessao de navegacao (dados do ros2 bag play)',
            f"  duracao {r['duracao_s']} s | percorrido {r['percorrido_m']} m | velocidade maxima "
            f"{r['velocidade_max_m_s']} m/s",
            f"  mensagens: {r['mensagens']}",
            f"  patrulha: {r['postos_alcancados']} postos alcancados, {r['postos_falhos']} "
            f"falhos, {r['disparos']} disparos | {r['fim_da_patrulha'] or 'sem mensagem de fim'}",
        ]
        if r['erro_amcl']:
            e = r['erro_amcl']
            linhas.append(f"  AMCL x pose real (/odom do Gazebo): erro medio {e['medio_m']} m, "
                          f"maximo {e['max_m']} m em {e['amostras']} amostras")
        linhas.append(f"  obstaculos a menos de {r['criterios']['distancia_parada_m']} m no setor "
                      f"frontal de {r['criterios']['setor_graus']:.0f} graus: {len(r['pausas'])}")
        for i, p in enumerate(r['pausas'], start=1):
            veredito = sim[p['ok']] if p['estado_arvore'] not in ('', 'OCIOSO') else 'fora de goal'
            linhas.append(
                f"    {i}. t={p['t_s']} s  distancia {p['distancia_m']} m  arvore "
                f"{p['estado_arvore'] or '-'}  parou em {p['reacao_s']} s  ficou parado "
                f"{p['parado_s']} s (PAUSADO {p['pausado_arvore_s']} s)  em ({p['x']}, {p['y']})  "
                f"-> {veredito}")
        v = r['validado']
        linhas += [
            f"  [{sim[v['pausa_por_obstaculo']]}] toda deteccao a menos de "
            f"{r['criterios']['distancia_parada_m']} m parou o robo em ate "
            f"{r['criterios']['reacao_max_s']} s por pelo menos {r['criterios']['espera_s']} s "
            f"(minimo de {r['criterios']['pausas_min']} pausa)",
            f"  [{sim[v['estado_da_arvore']]}] /arvore/estado ficou PAUSADO nesses intervalos",
            f"  [{sim[v['postos']]}] todos os postos da patrulha alcancados",
            f"  RESULTADO: {'APROVADO' if r['aprovado'] else 'REPROVADO'}",
        ]
        return linhas

    # ---------- graficos ----------
    def carregar_mapa(self):
        arquivo = self.p['mapa'] or os.path.join(get_package_share_directory('projeto_bloco'),
                                                 'maps', 'sala_alvos.yaml')
        try:
            with open(arquivo) as fh:
                meta = yaml.safe_load(fh)
            img = cv2.imread(os.path.join(os.path.dirname(arquivo), meta['image']),
                             cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise OSError(meta['image'])
            ox, oy = meta['origin'][0], meta['origin'][1]
            res = meta['resolution']
            return img, [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
        except (OSError, KeyError, yaml.YAMLError) as e:
            self.get_logger().warn(f'mapa nao carregado ({e}): grafico sem fundo')
            return None, None

    def grafico_trajetoria(self, arquivo, eventos, resumo):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=130)
        img, extensao = self.carregar_mapa()
        if img is not None:
            ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=extensao, origin='upper')
        ax.plot([o[1] for o in self.odom], [o[2] for o in self.odom], color='#1f77b4', lw=2.0,
                label='pose real (/odom do Gazebo)')
        if self.amcl:
            ax.plot([a[1] for a in self.amcl], [a[2] for a in self.amcl], color='#ff7f0e',
                    lw=1.2, ls='--', marker='.', ms=3, label='estimativa do AMCL (/amcl_pose)')
        ax.plot(self.odom[0][1], self.odom[0][2], 'o', color='green', ms=9, label='inicio')
        ax.plot(self.odom[-1][1], self.odom[-1][2], 'D', color='#17becf', ms=8, mec='black',
                label='fim')
        for i, (x, y) in enumerate(self.rota, start=1):
            ax.plot(x, y, '*', color='#9467bd', ms=15, mec='white',
                    label='postos da patrulha (goals)' if i == 1 else None)
            ax.annotate(str(i), (x, y), textcoords='offset points', xytext=(8, 8), fontsize=11,
                        color='#9467bd', weight='bold')
        for i, e in enumerate(e for e in eventos if e['x'] is not None):
            ax.plot(e['x'], e['y'], 'X', color='red', ms=13, mec='white',
                    label='pausa da behavior tree (obstaculo < 0,5 m)' if i == 0 else None)
        for _, texto in self.surpresa:
            campos = dict(c.split('=') for c in texto.split()[1:] if '=' in c)
            if texto.startswith('criada') and 'x' in campos:
                ax.plot(float(campos['x']), float(campos['y']), 's', color='#d62728', ms=11,
                        mfc='none', mew=2.5, label='caixa surpresa (aparece na frente do robo)')
        for i, item in enumerate(self.p['fora_do_mapa']):
            campos = item.split()
            if len(campos) != 4:
                continue
            x, y, lado = (float(c) for c in campos[1:])
            ax.add_patch(Rectangle((x - lado / 2, y - lado / 2), lado, lado, fill=False,
                                   ec='#bcbd22', lw=2.5, hatch='//',
                                   label='obstaculo fora do mapa (so o lidar ve)'
                                   if i == 0 else None))
        ax.set_xlabel('x no mapa (m)')
        ax.set_ylabel('y no mapa (m)')
        ax.set_title(f"Trajetoria da patrulha: {resumo['percorrido_m']} m em "
                     f"{resumo['duracao_s']} s, {resumo['postos_alcancados']} postos, "
                     f"{len([e for e in eventos if e['em_missao']])} pausa(s)")
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(loc='lower left', fontsize=8, framealpha=0.9)
        fig.tight_layout()
        fig.savefig(arquivo)
        plt.close(fig)

    def grafico_pausa(self, arquivo, eventos, t0):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(11, 7.5), dpi=130, sharex=True)
        ts = [t - t0 for t, _ in self.scan]
        ds = [min(d, 3.0) for _, d in self.scan]
        a1.plot(ts, ds, color='#1f77b4', lw=1.2)
        a1.axhline(self.p['distancia_parada_m'], color='red', ls='--', lw=1,
                   label=f"limite {self.p['distancia_parada_m']} m")
        a1.set_ylabel('distancia frontal\ndo lidar (m)')
        a1.set_ylim(0, 3.05)
        a1.legend(loc='upper right', fontsize=8)
        a2.plot([o[0] - t0 for o in self.odom], [o[4] for o in self.odom], color='#2ca02c',
                lw=1.2, label='linear (m/s)')
        a2.plot([o[0] - t0 for o in self.odom], [o[5] for o in self.odom], color='#ff7f0e',
                lw=0.9, alpha=0.8, label='angular (rad/s)')
        a2.set_ylabel('velocidade\n(/odom)')
        a2.legend(loc='upper right', fontsize=8)
        niveis = {'OCIOSO': 0, 'NAVEGANDO': 1, 'PAUSADO': 2, 'CHEGOU': 3, 'FALHOU': 3}
        if self.estados:
            a3.step([t - t0 for t, _ in self.estados], [niveis.get(r, 0) for _, r in self.estados],
                    where='post', color='#9467bd', lw=1.5)
        a3.set_yticks([0, 1, 2, 3])
        a3.set_yticklabels(['OCIOSO', 'NAVEGANDO', 'PAUSADO', 'CHEGOU'])
        a3.set_ylabel('/arvore/estado')
        a3.set_xlabel('tempo da sessao (s)')
        for e in eventos:
            if e['t_parou'] is None:
                continue
            fim = e['t_retomou'] if e['t_retomou'] is not None else e['t_parou']
            for ax in (a1, a2, a3):
                ax.axvspan(e['t_parou'] - t0, fim - t0, color='red', alpha=0.15)
            a2.annotate(f"parado {e['parado_s']} s", (e['t_parou'] - t0, 0.0),
                        textcoords='offset points', xytext=(4, 22), fontsize=9, color='red')
        for ax in (a1, a2, a3):
            ax.grid(alpha=0.3)
        a1.set_title('Pausa por obstaculo, vista no bag: o lidar cruza 0,5 m, o robo para, '
                     'espera e retoma o goal')
        fig.tight_layout()
        fig.savefig(arquivo)
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = AnaliseBag()
    codigo = 1
    inicio = time.monotonic()
    try:
        node.get_logger().info('aguardando a reproducao do bag (ros2 bag play <pasta> --clock)')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            agora = time.monotonic()
            if node.t_ultima is None:
                if agora - inicio > node.p['espera_max_s']:
                    node.get_logger().error('nenhuma mensagem chegou: o bag esta tocando?')
                    break
            elif agora - node.t_ultima > node.p['silencio_s']:
                codigo = node.analisar()
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
