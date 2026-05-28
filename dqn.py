import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque
import numpy as np


# --- Нейросеть Агента ---
class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(QNetwork, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim)
        )

    def forward(self, x):
        return self.net(x)


# --- Класс RL Управления с онлайн-обучением ---
class OnlineRLController:
    def __init__(self, state_dim=4, action_dim=7, lr=0.001, gamma=0.99):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Модели: текущая и целевая (для стабильности DQN)
        self.policy_net = QNetwork(state_dim, action_dim).to(self.device)
        self.target_net = QNetwork(state_dim, action_dim).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())

        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.memory = deque(maxlen=10000)  # Буфер памяти реплеев

        # Дискретные действия: [Сила влево, Ноль, Сила вправо]
        self.action_space = [-200, -100.0, -50.0, 0.0, 50.0, 100.0, 200.0]

        # Гиперпараметры исследования (Epsilon-greedy)
        self.epsilon = 1.0
        self.epsilon_decay = 0.999995
        self.epsilon_min = 0.05
        self.gamma = gamma
        self.batch_size = 32

        self.last_state = None
        self.last_action_idx = None

    def get_action(self, state):
        """Выбор действия (исследование / эксплуатация)"""
        self.last_state = np.array(state, dtype=np.float32)

        if random.random() < self.epsilon:
            self.last_action_idx = random.randint(0, len(self.action_space) - 1)
        else:
            state_t = torch.FloatTensor(self.last_state).to(self.device)
            with torch.no_grad():
                q_values = self.policy_net(state_t)
                self.last_action_idx = q_values.argmax().item()

        return self.action_space[self.last_action_idx]

    def compute_reward(self, state):
        """Функция награды: штраф за падение маятника и уход тележки"""
        x, dx, theta, dtheta = state

        # Нормализация угла в пределах [-pi, pi]
        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        # Штрафы (чем ближе к 0 угол и позиция, тем лучше)
        reward = -(theta ** 2) - 0.1 * (dtheta ** 2) - 0.01 * (x / 100.0) ** 2

        # Бонус за удержание в вертикальном положении
        if abs(theta) < 0.1:
            reward += 10.0

        # Терминальное состояние (сильное падение или вылет за экран)
        done = abs(theta) > np.pi / 3 or abs(x) > 400
        if done:
            reward -= 100.0

        return reward, done

    def train_step(self, next_state):
        """Один шаг онлайн-обучения сети из буфера памяти"""
        if self.last_state is None or self.last_action_idx is None:
            return False, 0

        next_state = np.array(next_state, dtype=np.float32)
        reward, done = self.compute_reward(next_state)

        # Сохраняем опыт в память
        self.memory.append((self.last_state, self.last_action_idx, reward, next_state, done))

        # Уменьшаем epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        if len(self.memory) < self.batch_size:
            return done, reward

        # Сэмплируем батч
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.LongTensor(actions).view(-1, 1).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)

        # Расчет текущих Q-значений
        current_q = self.policy_net(states).gather(1, actions)

        # Расчет целевых Q-значений (DQN)
        with torch.no_grad():
            max_next_q = self.target_net(next_states).max(1)[0]
            target_q = rewards + (1 - dones) * self.gamma * max_next_q

        # Функция потерь и оптимизация
        loss = nn.MSELoss()(current_q.squeeze(), target_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return done, reward

    def update_target_network(self):
        """Синхронизация целевой сети"""
        self.target_net.load_state_dict(self.policy_net.state_dict())
