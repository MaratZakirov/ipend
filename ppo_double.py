import time
import gymnasium as gym
import numpy as np
from torch import nn
import torch
import torch.optim as optim
from torch.distributions import Normal

class PolicyNetwork(nn.Module):
    def __init__(self):
        super(PolicyNetwork, self).__init__()

        # Общая (скрытая) часть сети для извлечения признаков из состояния
        self.shared_net = nn.Sequential(
            nn.Linear(9, 64),  # 9 входов для InvertedDoublePendulum-v5
            nn.ReLU(),
            nn.Linear(64, 64),  # Дополнительный слой для обработки сложной физики
            nn.ReLU()
        )

        # Выход для среднего значения силы (mu).
        # Tanh сжимает выход в диапазон [-1.0, 1.0], что идеально подходит под экшн-спейс MuJoCo
        self.mu_head = nn.Sequential(
            nn.Linear(64, 1),
            nn.Tanh()
        )

        # Выход для стандартного отклонения (sigma), то есть для уровня шума/исследования.
        # Softplus гарантирует, что значение всегда будет строго положительным (больше 0)
        self.sigma_head = nn.Sequential(
            nn.Linear(64, 1),
            nn.Softplus()
        )

    def forward(self, state):
        """Возвращает параметры распределения Гаусса для текущего состояния"""
        features = self.shared_net(state)
        mu = self.mu_head(features)

        # На всякий случай добавляем крошечную константу 1e-5,
        # чтобы из-за машинного округления sigma никогда не стала ровно нулем (это вызвало бы ошибку деления на ноль)
        sigma = self.sigma_head(features) + 1e-5

        return mu, sigma

    def get_action(self, state):
        mu, sigma = self.forward(state)
        dist = Normal(mu, sigma)
        action = dist.sample()

        # Считаем логарифм плотности вероятности — это критически важно для формулы PPO
        log_prob = dist.log_prob(action).sum(dim=-1)

        # Ограничиваем действие под границы среды на всякий случай
        action = torch.clamp(action, -1.0, 1.0)

        return action, log_prob

    def get_deterministic_action(self, state):
        """
        Используется во время ДЕМОНСТРАЦИИ (после обучения).
        Игнорирует шум и выдает строго лучшее среднее значение.
        """
        mu, _ = self.forward(state)
        return torch.clamp(mu, -1.0, 1.0)


# --- 2. Логика PPO Агента ---
class PPOAgent:
    def __init__(self, lr=0.002, gamma=0.99, clip_ratio=0.2):
        self.policy = PolicyNetwork()
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.gamma = gamma
        self.clip_ratio = clip_ratio

    def compute_returns(self, rewards):
        returns = []
        R = 0
        for r in reversed(rewards):
            R = r + self.gamma * R
            returns.insert(0, R)
        returns = np.array(returns)
        returns = (returns - np.mean(returns)) / (np.std(returns) + 1e-8)
        return returns

    def update_policy(self, states, actions, rewards, old_log_probs):
        states_t = torch.tensor(np.array(states), dtype=torch.float32)
        actions_t = torch.tensor(np.array(actions), dtype=torch.float32)
        old_log_probs_t = torch.tensor(np.array(old_log_probs), dtype=torch.float32)

        returns = self.compute_returns(rewards)
        advantages_t = torch.tensor(returns, dtype=torch.float32)

        new_probs = self.policy(states_t)
        chosen_probs = new_probs.gather(1, actions_t.unsqueeze(1)).squeeze(1)

        ratio = chosen_probs / old_probs_t
        surr1 = ratio * advantages_t
        surr2 = torch.clamp(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * advantages_t
        loss = -torch.mean(torch.min(surr1, surr2))

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

# --- 3. ЧЕСТНОЕ ОБУЧЕНИЕ (Без графики) ---
def train_agent():
    env = gym.make("InvertedDoublePendulum-v5", render_mode=None)
    agent = PPOAgent()

    print("Начало честного обучения PPO в фоне (без читов)...")

    # Обучаем 150 эпизодов. Сеть сама набивает шишки.
    for episode in range(1000):
        state, info = env.reset()
        states, actions, rewards, old_probs = [], [], [], []
        episode_reward = 0

        while True:
            state_t = torch.tensor(state, dtype=torch.float32)

            with torch.no_grad():
                action, log_prob = agent.policy.get_action(state_t)

            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            states.append(state)
            actions.append(action)
            rewards.append(reward)
            old_probs.append(log_prob)

            episode_reward += reward
            state = next_state

            if done:
                break

        # Обновляем политику на основе полученного опыта
        agent.update_policy(states, actions, rewards, old_probs)

        if episode % 15 == 0:
            print(f"Эпизод: {episode} \t Результат: {episode_reward} очков")

        if episode_reward >= 500:
            print(f"\n Агент честно обучился на {episode} эпизоде!")
            break

    env.close()
    return agent

def run_double_pendulum_demo(agent):
    # Загружаем индустриальный движок MuJoCo для двойного маятника
    env = gym.make("InvertedDoublePendulum-v5", render_mode="human")
    observation, info = env.reset()

    for _ in range(1000):
        action = agent.policy.get_deterministic_action(observation)
        observation, reward, terminated, truncated, info = env.step(action)

        time.sleep(0.5)

        if terminated or truncated:
            print("Система потеряла баланс, перезапуск...")
            observation, info = env.reset()

    env.close()


if __name__ == "__main__":
    agent = train_agent()
    run_double_pendulum_demo(agent)
