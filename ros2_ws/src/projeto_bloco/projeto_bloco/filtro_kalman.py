"""
Filtro de Kalman 2D de velocidade constante: suaviza o centro de um objeto rastreado.

Estado s = [x, y, vx, vy] (px e px/s); medicao z = [x, y] (centro da caixa do YOLO).
  predicao:  s = F s,   P = F P F' + Q          F = [[I, dt*I], [0, I]]
  correcao:  K = P H' (H P H' + R)^-1,  s = s + K (z - H s),  P = (I - K H) P
Q: aceleracao aleatoria (ruido branco) com desvio sigma_a (px/s^2); R = sigma_z^2 * I (px).
sigma_z maior / sigma_a menor = trilha mais suave, porem mais atrasada em mudancas bruscas.
"""

import numpy as np

H = np.array([[1.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0]])


class FiltroKalman2D:

    def __init__(self, x, y, t, sigma_a=200.0, sigma_z=15.0):
        self.sigma_a = sigma_a
        self.sigma_z = sigma_z
        self.s = np.array([x, y, 0.0, 0.0])
        # posicao ~ medida; velocidade inicial desconhecida (desvio de 300 px/s)
        self.P = np.diag([sigma_z ** 2, sigma_z ** 2, 300.0 ** 2, 300.0 ** 2])
        self.t = t

    def prever(self, t):
        """Propaga o estado ate o instante t (s); dt <= 0 nao faz nada."""
        dt = t - self.t
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([dt * dt / 2, dt])           # efeito de uma aceleracao em (pos, vel)
        q = np.outer(g, g) * self.sigma_a ** 2
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = q             # eixo x
        Q[np.ix_([1, 3], [1, 3])] = q             # eixo y
        self.s = F @ self.s
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def corrigir(self, x, y):
        R = np.eye(2) * self.sigma_z ** 2
        inovacao = np.array([x, y]) - H @ self.s
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ inovacao
        self.P = (np.eye(4) - K @ H) @ self.P

    def atualizar(self, x, y, t):
        """Predicao ate t + correcao com a medida (x, y); devolve o centro filtrado."""
        self.prever(t)
        self.corrigir(x, y)
        return self.posicao

    @property
    def posicao(self):
        return float(self.s[0]), float(self.s[1])

    @property
    def velocidade(self):
        return float(self.s[2]), float(self.s[3])
