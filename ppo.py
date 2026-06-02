import torch
import torch.nn as nn
import torch.optim as optim
import random
import numpy as np
from torch.distributions.categorical import Categorical


# --- Нейросеть Агента ---
class PolicyNet(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(PolicyNet, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, action_dim)
        )

    def forward(self, x):
        return self.net(x)


# --- Класс RL Управления с онлайн-обучением ---
class OnlineRLController:
    def __init__(self, state_dim=4, action_dim=7, lr=0.001, gamma=0.99):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # PPO model
        self.policy_net = PolicyNet(state_dim, action_dim).to(self.device)
        self.optimizer = optim.AdamW(self.policy_net.parameters(), lr=lr)

        # Дискретные действия: [Сила влево, Ноль, Сила вправо]
        self.action_space = [-100, -50.0, -25.0, 0.0, 25.0, 50.0, 100]
        self.gamma = gamma

        self.num_policy_call = 0
        self.compute_loss_counter = 0


    def compute_reward_old(self, state):
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
        done = abs(theta) > np.pi / 10 or abs(x) > 400

        if done:
            reward -= 100.0

        return reward, done


    def compute_reward_old2(self, state):
        x, dx, theta, dtheta = state
        # Упал или вылетел?
        done = abs(theta) > np.pi / 10 or abs(x) > 400

        # Если не упал — держи жирный плюс за выживание. Упал — 0.
        reward = 1.0 if not done else 0.0
        return reward, done

    def compute_reward_old3(self, state):
        x, dx, theta, dtheta = state
        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        done = abs(theta) > np.pi / 6 or abs(x) > 4000  # Расширил угол до 30 градусов (pi/6)

        if done:
            return 0.0, True

        # Награда: 1.0 когда угол идеальный (0), и падает до 0, когда угол близок к падению
        reward = 1.0 - (theta / (np.pi / 6)) ** 2
        return reward, done

    def compute_reward(self, state):
        x, dx, theta, dtheta = state
        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        done = abs(theta) > np.pi / 6  # Упал
        if done:
            return 0.0, True

        # Жирный плюс (1.0), только если угол идеален (0).
        # Если угол отклоняется, награда падает до нуля.
        reward = 1.0 - (theta / (np.pi / 6)) ** 2
        return reward, done

    # make function to compute action distribution
    def get_policy(self, obs):
        logits = self.policy_net(obs)

        self.num_policy_call += 1
        if self.num_policy_call % 10000 == 0:
            print('Logits:', nn.Softmax()(logits).detach().cpu().numpy().round(2))

        return Categorical(logits=logits)

    # make action selection function (outputs int actions, sampled from policy)
    def get_action(self, obs):
        return self.get_policy(obs).sample().item()

    # make loss function whose gradient, for the right data, is policy gradient
    def compute_loss_old(self, obs, act, weights):
        logp = self.get_policy(obs).log_prob(act)
        return -(logp * weights).mean()

    def compute_loss(self, obs, act, weights, entropy_coef=0.3):
        policy = self.get_policy(obs)
        logp = policy.log_prob(act)

        # Классический лосс градиента стратегии
        policy_loss = -(logp * weights).mean()

        # Бонус за энтропию (минусуем среднюю энтропию распределения)
        entropy_loss = -policy.entropy().mean()

        # Итоговый лосс для оптимизатора
        self.compute_loss_counter += 1
        if self.compute_loss_counter % 2 == 0:
            print(f'Policy_loss {policy_loss.item():.4f} | Entropy_loss {entropy_loss.item():.4f}')

        return policy_loss + entropy_coef * entropy_loss
