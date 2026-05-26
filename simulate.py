import pygame
import math
import sys
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
from pid import PID

# Hyper parameters
W, H = 800, 600
FPS = 30
dt = 1/FPS
m = 1
M = 10
g = 9.8
p_length = 100
l = p_length

# Initial state (example values)
# X | dX/dt | T | dT/dt
X = np.zeros(4)
X[2] -= 0.1
X[0] += 70

def transform(x: int, y: int):
    return x + W//2, 0.9*H - y

# --- Pygame Initialization ---
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Inverted Pendulum Simulation")
clock = pygame.time.Clock()

class Solution():
    def __init__(self, m, M, l, g):
        self.use_rk = True
        self.m = m
        self.M = M
        self.l = l
        self.g = g

    def calc_dX(self, t, X, u):
        """Чистая физика: вычисляет dX на основе текущего состояния X и внешней силы u"""
        m, M, l, g = self.m, self.M, self.l, self.g
        D = lambda x: M + m * (np.sin(x)**2)
        x, dx, theta, dtheta = X

        # Ограничение силы (чтобы не рвать систему)
        u_max = 1150.0
        u = np.clip(u, -u_max, u_max)

        dX = np.zeros(4)
        dX[0] = dx
        dX[2] = dtheta
        dX[1] = (u + m*l*np.sin(theta)*dtheta**2 - m*g*np.sin(theta)*np.cos(theta)) / D(theta)
        dX[3] = (-u*np.cos(theta) - m*l*np.sin(theta)*np.cos(theta)*dtheta**2 + (M+m)*g*np.sin(theta)) / (l*D(theta))
        return dX

    def step(self, X: np.array, u: float):
        """Делает шаг симуляции вперед во времени, используя силу управления u"""
        if self.use_rk:
            # Передаем u через аргумент args в solve_ivp
            sol = solve_ivp(self.calc_dX, [0, dt], X, args=(u,), method='RK45')
            X = sol.y[:, -1]
        else:
            # Для метода Эйлера t передаем как 0
            X = X + self.calc_dX(0, X, u) * dt

        return X


class Stuff():
    def __init__(self):
        self.cart_W = 60
        self.cart_H = 20
        self.p_length = p_length

    # --- Functions for drawing ---
    def draw_cart(self, screen, cart_x, cart_y):
        x, y = transform(cart_x, cart_y)
        pygame.draw.line(screen, (0, 0, 0), (0, H*0.9), (W, H*0.9), 2)
        cart_rect = pygame.Rect(x - self.cart_W//2, y - self.cart_H, self.cart_W, self.cart_H)
        pygame.draw.rect(screen, (0, 0, 255), cart_rect)

    def draw_pendulum(self, screen, cart_x, cart_y, end_x, end_y):
        x, y = transform(cart_x, cart_y)
        ex, ey = transform(end_x, end_y)
        pygame.draw.line(screen, (0,0,0), (x, y), (ex, ey), 5)
        pygame.draw.circle(screen, (255, 0, 0), (ex, ey), 15)

    def draw(self, screen, X):
        x, _, theta, _ = X
        end_x = x + self.p_length * math.sin(theta)
        end_y = 0 + self.p_length * math.cos(theta)
        self.draw_pendulum(screen, x, 0, end_x, end_y)
        self.draw_cart(screen, x, 0)

pid_params = {'Kp' : 4*1024, 'Ki' : 1, 'Kd' : 4*1024}
# Ku = 120
# Tu = 1700
pid_x_params = {'Kp' : 0.001, 'Ki' : 0.000001, 'Kd' : 0.001}

if __name__ == "__main__":
    stuff = Stuff()

    # 1. Инициализируем чистый физический движок
    sol = Solution(m, M, p_length, g)

    # 2. Инициализируем PID-контроллеры внешним образом
    pid = PID(Kp=pid_params['Kp'], Ki=pid_params['Ki'], Kd=pid_params['Kd'])
    pid_x = PID(Kp=pid_x_params['Kp'], Ki=pid_x_params['Ki'], Kd=pid_x_params['Kd'])

    x_ref = 0.0
    cnt = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- КОНТРОЛЛЕР (Внешний уровень управления) ---
        x, dx, theta, dtheta = X

        # Внешний контур: позиция тележки -> желаемый угол
        theta_ref_raw = pid_x.step(x_ref - x, dt)
        theta_max = 0.2
        theta_ref = 0  # np.tanh(theta_ref_raw / theta_max) * theta_max

        # Внутренний контур: угол -> необходимая сила u
        u = pid.step(theta - theta_ref, dt)

        # --- ФИЗИКА (Передаем вычисленное u в движок) ---
        X = sol.step(X, u)

        # Отрисовка
        screen.fill((255, 255, 255))
        stuff.draw(screen, X)

        cnt += 1
        if cnt % FPS == 0:
            print(cnt, f"Angle {X[2]:.2f} | Control Force U: {u:.2f}")

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()
