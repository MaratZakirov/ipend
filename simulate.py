import pygame
import math
import sys
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
from torch.distributed.elastic import agent
from pid import PID
from ppo import OnlineRLController
import copy
import torch


# Hyper parameters
W, H = 800, 600
FPS = 30
dt = 1/FPS
m = 1
M = 10
g = 9.8
p_length = 100
l = p_length

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

def get_random_state():
    S_0 = np.array([np.random.uniform(-150.0, 150.0),
                    np.random.uniform(-10.0, 10.0),
                    np.random.uniform(-0.3, 0.3),
                    np.random.uniform(-0.3, 0.3)])
    S_0 = np.array([70.0, 0, -0.2, 0])
    return S_0

# for training policy
def pretrain_agent_one_batch(agent, solution, batch_size: int = 8000):
    episode_num = 0

    batch_weights = []
    episode_lengths = []

    S_t, done, states, actions, episode_rewards = get_random_state(), False, [], [], []

    while True:
        # 1. Запоминаем состояние в тот момент, когда сеть РЕАЛЬНО принимает решение
        states.append(S_t)

        # 2. Агент выбирает действие
        a_t = agent.get_action(torch.as_tensor(S_t, dtype=torch.float32).to(agent.device))
        actions.append(a_t)
        force = agent.action_space[a_t]

        # 3. --- МЕХАНИЗМ FRAME SKIPPING (УДЕРЖАНИЕ ДЕЙСТВИЯ) ---
        # Применяем выбранную силу в течение 4 шагов симулятора подряд
        accumulated_reward = 0
        for _ in range(4):
            S_t = solution.step(S_t, force)
            r_t, done = agent.compute_reward(S_t)
            accumulated_reward += r_t
            if done:
                break  # Если упал посреди макро-шага, прерываемся

        # Сохраняем суммарную награду, полученную за время удержания этой силы
        episode_rewards.append(accumulated_reward)

        if done:
            episode_num += 1

            episode_lengths.append(len(episode_rewards))

            # Дисконтированный Reward-to-go
            discounted_reward = 0
            episode_weights = []
            for r in reversed(episode_rewards):
                discounted_reward = r + agent.gamma * discounted_reward
                episode_weights.insert(0, discounted_reward)

            batch_weights += episode_weights

            if len(states) > batch_size:
                break

            S_t, done, episode_rewards = get_random_state(), False, []

    # Обучение сети (код остается без изменений)
    agent.optimizer.zero_grad()
    weights_tensor = torch.as_tensor(batch_weights, dtype=torch.float32).to(agent.device)
    normalized_weights = (weights_tensor - weights_tensor.mean()) / (weights_tensor.std() + 1e-8)

    batch_loss = agent.compute_loss(
        obs=torch.as_tensor(states, dtype=torch.float32).to(agent.device),
        act=torch.as_tensor(actions, dtype=torch.int32).to(agent.device),
        weights=normalized_weights
    )
    batch_loss.backward()
    agent.optimizer.step()

    return batch_loss.item(), np.mean(episode_lengths), episode_num


if __name__ == "__main__":
    stuff = Stuff()

    # 1. Инициализируем чистый физический движок
    sol = Solution(m, M, p_length, g)

    # 2. Инициализируем PID-контроллеры внешним образом
    pid = PID(Kp=pid_params['Kp'], Ki=pid_params['Ki'], Kd=pid_params['Kd'])
    pid_x = PID(Kp=pid_x_params['Kp'], Ki=pid_x_params['Ki'], Kd=pid_x_params['Kd'])
    x_ref = 0.0

    # 3. Инициализируем RL-агент онлайн-обучения
    # state_dim=4 (x, dx, theta, dtheta), action_dim=3 (влево, стоп, вправо)
    rl_agent = OnlineRLController(state_dim=4, lr=1e-4)
    target_update_counter = 0

    # Выбор режима: "PID" или "RL"
    CONTROL_MODE = "RL"

    if CONTROL_MODE == "RL":
        print('Pretrain 100 batches with batch_size=10')
        for batch in range(10):
            loss, batch_len, ep_num = pretrain_agent_one_batch(rl_agent, sol, batch_size=1000)
            print(f'Batch {batch} len {batch_len:.2f} ep_num {ep_num}')

    # Выставляем начальные условия
    # Initial state (example values)
    # X | dX/dt | T | dT/dt
    S_0 = np.array([70.0, 0, -0.2, 0])

    cnt = 0
    running = True

    S_t = np.copy(S_0)

    while running:
        if abs(S_t[0]) > 300:
            S_t = np.copy(S_0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Интерактив: переключение режима по кнопке пробел
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    CONTROL_MODE = "RL" if CONTROL_MODE == "PID" else "PID"
                    print(f"--- РЕЖИМ ИЗМЕНЕН НА: {CONTROL_MODE} ---")

        # --- СБОР ТЕКУЩЕГО СОСТОЯНИЯ ---
        # Запоминаем текущее состояние перед шагом физики (нужно для RL)
        x, dx, theta, dtheta = S_t

        # --- КОНТРОЛЛЕР (Внешний уровень управления) ---
        if CONTROL_MODE == "PID":
            # Внешний контур PID: позиция тележки -> желаемый угол
            theta_ref_raw = pid_x.step(x_ref - x, dt)
            theta_max = 0.2
            theta_ref = 0 # np.tanh(theta_ref_raw / theta_max) * theta_max

            # Внутренний контур PID: угол -> необходимая сила u
            u = pid.step(theta - theta_ref, dt)

        elif CONTROL_MODE == "RL":
            # Нейросеть выбирает силу u на основе текущего состояния X
            u = rl_agent.action_space[rl_agent.get_action(torch.as_tensor(S_t, dtype=torch.float32))]

        # --- ФИЗИКА (Передаем вычисленное u в движок) ---
        u = np.clip(u, a_min=-100, a_max=100) # ограничим амплитуду силы
        S_t = sol.step(S_t, u)

        # --- ОТРИСОВКА ---
        screen.fill((255, 255, 255))
        stuff.draw(screen, S_t)

        # Вывод отладочной информации на экран
        cnt += 1
        if cnt % FPS == 0:
            print(f"Mode: {CONTROL_MODE} | Angle: {S_t[2]:.2f} | Force U: {u:.2f}", S_t.round(2))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()