import pygame
import math
import sys
import numpy as np

# Hyper parameters
W, H = 800, 600
FPS = 60
dt = 1/FPS
m = 1
M = 10
p_length = 100

# Initial state (example values)
# X | dX/dt | T | dT/dt
X = np.zeros(4)

# Matrix transformation from coordinates to display coordinates
def convert_2_display(X: np.array):
    Y = np.copy(X)
    Y[0] += W/2
    return Y

# --- Pygame Initialization ---
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Inverted Pendulum Simulation")
clock = pygame.time.Clock()

class Solution():
    def __init__(self, m, M, l):
        self.m = m
        self.M = M
        self.l = l
        self.g = 9.8

    def step(self, X: np.array):
        x, dx, theta, dtheta = X
        L = 0.5*(self.M + self.m)*dx**2 + self.m*self.l*dx*dtheta*np.cos(theta) + 0.5*self.m*(self.l**2)*(dtheta**2) - self.m*self.g*self.l*np.cos(theta)
        return X

class Stuff():
    def __init__(self):
        self.cart_W = 60
        self.cart_H = 30
        self.p_length = p_length

    # --- Functions for drawing ---
    def draw_cart(self, screen, cart_x, cart_y):
        cart_rect = pygame.Rect(cart_x - self.cart_W // 2, cart_y - self.cart_H, self.cart_W, self.cart_H)
        pygame.draw.rect(screen, (0, 0, 255), cart_rect)

    def draw_pendulum(self, screen, cart_x, cart_y, end_x, end_y):
        pygame.draw.line(screen, (0,0,0), (cart_x, cart_y), (int(end_x), int(end_y)), 5)
        pygame.draw.circle(screen, (255, 0, 0), (int(end_x), int(end_y)), 15)

    def draw(self, screen, X):
        x, _, theta, _ = X
        pygame.draw.line(screen, (0, 0, 0), (0, H*0.9), (W, H*0.9), 2)
        end_x = x + self.p_length * math.sin(theta + np.pi)
        end_y = H * 0.9 + self.p_length * math.cos(theta + np.pi)
        self.draw_pendulum(screen, x, H*0.9, end_x, end_y)
        self.draw_cart(screen, x, H * 0.9)

stuff = Stuff()
sol = Solution(m, M, p_length)

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
    stuff.draw(screen, convert_2_display(X))
    X = sol.step(X)

    # 4. Update the display
    pygame.display.flip()

    # 5. Cap the frame rate
    clock.tick(FPS)

# --- Quit Pygame ---
pygame.quit()
sys.exit()