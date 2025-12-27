import pygame
import math
import sys
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

# Hyper parameters
W, H = 800, 600
FPS = 60
dt = 1/FPS
m = 1
M = 10
g = 9.8
p_length = 100
l = p_length

# Initial state (example values)
# X | dX/dt | T | dT/dt
X = np.zeros(4)
X[2] += 0.1

def transform(x: int, y: int):
    return x + W//2, 0.9*H - y

# --- Pygame Initialization ---
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Inverted Pendulum Simulation")
clock = pygame.time.Clock()

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class Solution():
    def __init__(self, m, M, l, g):
        self.m = m
        self.M = M
        self.l = l
        self.g = g

        # PID по углу: theta_ref = 0
        self.pid = PID(Kp=200.0, Ki=0.0, Kd=20.0)  # подберите

        # Внешний PID по позиции (медленный)
        # self.pid_x = PID(Kp=1.0, Ki=0.0, Kd=0.5)

        # Желаемая позиция тележки
        # self.x_ref = 0.0

    def calc_dX(self, t, X):
        m, M, l, g = self.m, self.M, self.l, self.g
        D = lambda x: self.M + self.m*(np.sin(x)**2)
        x, dx, theta, dtheta = X

        # ошибка: хотим theta = 0 (вертикально вверх)
        theta_ref = 0.0
        error = -theta_ref + theta

        # PID управление (ограничим по величине, чтобы не «рвать» систему)
        u_raw = self.pid.step(error, dt)
        u_max = 50.0     # подберите
        u = np.clip(u_raw, -u_max, u_max)

        dX = np.zeros(4)
        dX[0] = dx
        dX[2] = dtheta
        dX[1] = (u + m*l*np.sin(X[2])*X[3]**2 - m*g*np.sin(X[2])*np.cos(X[2])) / D(X[2])
        dX[3] = (-u*np.cos(X[2]) - m*l*np.sin(X[2])*np.cos(X[2])*X[3]**2 + (M+m)*g*np.sin(X[2])) / (l*D(X[2]))
        return dX

    def step(self, X: np.array):
        x, dx, theta, dtheta = X
        #L = 0.5*(self.M + self.m)*dx**2 + self.m*self.l*dx*dtheta*np.cos(theta) + 0.5*self.m*(self.l**2)*(dtheta**2) - self.m*self.g*self.l*np.cos(theta)
        sol = solve_ivp(self.calc_dX, [dt, 2*dt], X, method='RK45')
        X = sol.y[:, -1]
        #Euler integration
        #X = X + self.calc_dX(X) * dt
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

stuff = Stuff()
sol = Solution(m, M, p_length, g)

# --- Main simulation loop ---
running = True
while running:
    # 1. Event Handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 2. Game Logic and Physics Updates (Placeholder)
    # Update cart_x and pendulum_angle based on physics equations.

    # 3. Drawing
    screen.fill((255, 255, 255))
    stuff.draw(screen, X)
    X = sol.step(X)

    # 4. Update the display
    pygame.display.flip()

    # 5. Cap the frame rate
    clock.tick(FPS)

# --- Quit Pygame ---
pygame.quit()
sys.exit()