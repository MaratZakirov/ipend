import pygame
import math
import sys
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
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

def get_random_state():
    S_0 = np.array([np.random.uniform(-150.0, 150.0),
                    np.random.uniform(-10.0, 10.0),
                    np.random.uniform(-0.3, 0.3),
                    np.random.uniform(-0.3, 0.3)])
    return S_0

# for training policy
def pretrain_agent_one_batch(agent, solution, batch_size: int=32):
    batch_weights = []
    episode_lengths = []  # --- ТЕПЕРЬ СЛЕДИМ ЗА ДЛИНОЙ РАУНДОВ ---

    S_t, done, states, actions, episode_rewards = get_random_state(), False, [], [], []

    while True:
        states.append(S_t)

        a_t = agent.get_action(torch.as_tensor(S_t, dtype=torch.float32).to(agent.device))
        actions.append(a_t)

        S_t = solution.step(S_t, a_t)
        r_t, done = agent.compute_reward(S_t)
        episode_rewards.append(r_t)

        if done:
            # Длина этого раунда — это просто количество шагов в episode_rewards
            episode_lengths.append(len(episode_rewards))

            R = np.array(episode_rewards)
            batch_weights += (R.sum() + R - np.cumsum(R)).tolist()

            if len(states) > batch_size:
                break

            S_t, done, episode_rewards = get_random_state(), False, []

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

    # --- ВОЗВРАЩАЕМ СРЕДНЮЮ ДЛИНУ РАУНДА В ЭТОМ БАТЧЕ ---
    return batch_loss.item(), np.mean(episode_lengths)


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
    rl_agent = OnlineRLController(state_dim=4)
    target_update_counter = 0

    # Выбор режима: "PID" или "RL"
    CONTROL_MODE = "PID"

    if CONTROL_MODE == "RL":
        print('Pretrain 100 epoches with batch_size=10')
        for epoch in range(100):
            print(f'Run epoch {epoch}')
            loss, R = pretrain_agent_one_batch(rl_agent, sol)

    # Выставляем начальные условия
    X[2] -= 0.1  # Начальный небольшой наклон
    X[0] += 70  # Смещение

    cnt = 0
    running = True
    while running:
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
        current_state = np.copy(X)
        x, dx, theta, dtheta = current_state

        # --- КОНТРОЛЛЕР (Внешний уровень управления) ---
        if CONTROL_MODE == "PID":
            # Внешний контур PID: позиция тележки -> желаемый угол
            theta_ref_raw = pid_x.step(x_ref - x, dt)
            theta_max = 0.2
            theta_ref = 0  # np.tanh(theta_ref_raw / theta_max) * theta_max

            # Внутренний контур PID: угол -> необходимая сила u
            u = pid.step(theta - theta_ref, dt)

        elif CONTROL_MODE == "RL":
            # Нейросеть выбирает силу u на основе текущего состояния X
            u = rl_agent.get_action(current_state)

        # --- ФИЗИКА (Передаем вычисленное u в движок) ---
        u = np.clip(u, a_min=-200, a_max=200) # ограничим амплитуду силы
        X = sol.step(X, u)

        # --- ОНЛАЙН-ОБУЧЕНИЕ (Только в режиме RL) ---
        if CONTROL_MODE == "RL":
            # Передаем НОВОЕ состояние в агент для расчета награды и шага оптимизации сети
            done, reward = rl_agent.train_step(X)

            # Периодически синхронизируем Target-сеть DQN для стабильности
            target_update_counter += 1
            if target_update_counter % 10 == 0:
                rl_agent.update_target_network()

            # Если маятник упал или улетел за экран — сбрасываем среду
            if done:
                X = np.zeros(4)
                X[2] -= 0.1  # Начальный небольшой наклон
                X[0] += 70  # Смещение
                rl_agent.last_state = None  # Сбрасываем историю шага агента
                pid.reset()  # На всякий случай сбрасываем интеграторы PID
                pid_x.reset()

        # --- ОТРИСОВКА ---
        screen.fill((255, 255, 255))
        stuff.draw(screen, X)

        # Вывод отладочной информации на экран
        cnt += 1
        if cnt % FPS == 0:
            info_str = f"Mode: {CONTROL_MODE} | Angle: {X[2]:.2f} | Force U: {u:.2f}"
            if CONTROL_MODE == "RL":
                info_str += f" | Eps: {rl_agent.epsilon:.2f} | Rew: {reward:.1f}"
            print(info_str)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()
