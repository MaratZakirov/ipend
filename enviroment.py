from scipy.integrate import solve_ivp
import numpy as np

class Enviroment():
    def __init__(self, m=0.1, M=1.0, l=0.5, g=9.8, a=10.0, dt=0.02):
        self.use_rk = True
        self.m = m
        self.M = M
        self.l = l
        self.g = g
        self.a = a
        self.dt = dt
        self.current_step = 0
        self.X = np.random.uniform(low=-0.05, high=0.05, size=(4,)).astype(np.float32)

    def reset(self):
        self.current_step = 0
        self.X = np.random.uniform(low=-0.05, high=0.05, size=(4,)).astype(np.float32)
        return np.copy(self.X), {}

    def close(self):
        pass

    def solution(self, t, X, u):
        """Чистая физика: вычисляет dX на основе текущего состояния X и внешней силы u"""
        m, M, l, g = self.m, self.M, self.l, self.g
        D = lambda x: M + m * (np.sin(x)**2)
        x, dx, theta, dtheta = X

        # Ограничение силы (чтобы не рвать систему)
        u_max = 20.0
        u = np.clip(u, -u_max, u_max)

        dX = np.zeros(4)
        dX[0] = dx
        dX[2] = dtheta
        dX[1] = (u + m*l*np.sin(theta)*dtheta**2 - m*g*np.sin(theta)*np.cos(theta)) / D(theta)
        dX[3] = (-u*np.cos(theta) - m*l*np.sin(theta)*np.cos(theta)*dtheta**2 + (M+m)*g*np.sin(theta)) / (l*D(theta))

        return dX

    def step(self, u: float | int):
        # 1. Переводим дискретное действие (0 или 1) в силу в Ньютонах
        if isinstance(u, (int, np.integer)):
            u = -10.0 if u == 0 else 10.0

        # 2. Физический движок (интегрирование)
        if self.use_rk:
            sol = solve_ivp(self.solution, [0, self.dt], self.X, args=(u,), method='RK45')
            self.X = sol.y[:, -1].astype(np.float32)
        else:
            self.X = (self.X + self.solution(0, self.X, u) * self.dt).astype(np.float32)

        # x - позиция, x_dot - скорость, theta - угол (рад), theta_dot - угл. скорость
        x, x_dot, theta, theta_dot = self.X

        # По правилам CartPole: позиция вышла за +-2.4 ИЛИ угол вышел за +-12 градусов (0.209 рад)
        terminated = bool(x < -2.4 or x > 2.4 or theta < -0.209 or theta > 0.209)

        # В CartPole агент получает +1.0 за каждый шаг, пока шест не упал
        reward = 0.0 if terminated else 1.0

        # Для CartPole-v1 лимит составляет 500 шагов.
        self.current_step += 1
        truncated = bool(self.current_step >= 500)

        info = {}

        # Возвращаем строго 5 элементов, как требует Gymnasium
        return np.copy(self.X), reward, terminated, truncated, info
