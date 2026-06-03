import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import time

# --- 1. Архитектура сети ---
class PolicyNetwork(nn.Module):
    def __init__(self, num_actions):
        super(PolicyNetwork, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 6),
            nn.ReLU(),
            nn.Linear(6, num_actions),
            nn.Softmax(dim=-1)
        )

    def forward(self, state):
        return self.net(state)


# --- 2. Логика PPO Агента ---
class PPOAgent:
    def __init__(self, num_actions, lr=0.002, gamma=0.99, clip_ratio=0.2):
        self.policy = PolicyNetwork(num_actions)
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

    def update_policy(self, states, actions, rewards, old_probs):
        states_t = torch.tensor(np.array(states), dtype=torch.float32)
        actions_t = torch.tensor(np.array(actions), dtype=torch.long)
        old_probs_t = torch.tensor(np.array(old_probs), dtype=torch.float32)

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
def train_agent_honestly():
    env = gym.make("CartPole-v1", render_mode=None)
    agent = PPOAgent(num_actions=2)

    print("Начало честного обучения PPO в фоне (без читов)...")

    # Обучаем 150 эпизодов. Сеть сама набивает шишки.
    for episode in range(1000):
        state, info = env.reset()
        states, actions, rewards, old_probs = [], [], [], []
        episode_reward = 0

        while True:
            state_t = torch.tensor(state, dtype=torch.float32)

            with torch.no_grad():
                probs = agent.policy(state_t).numpy()

            # Действие выбирается строго по вероятностям самой сети
            action = np.random.choice(2, p=probs)
            old_prob = probs[action]

            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            states.append(state)
            actions.append(action)
            rewards.append(reward)
            old_probs.append(old_prob)

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


# --- 4. ДЕМОНСТРАЦИЯ (С графикой) ---
def run_demonstration(agent):
    print("\nЗапуск демонстрации обученного агента в PyGame...")
    env = gym.make("CartPole-v1", render_mode="human")

    state, info = env.reset()
    total_reward = 0

    while True:
        state_t = torch.tensor(state, dtype=torch.float32)
        with torch.no_grad():
            probs = agent.policy(state_t).numpy()

        # Берем самое уверенное действие
        action = np.argmax(probs)

        state, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

        time.sleep(0.02)  # Чтобы глаз успевал следить

        if terminated or truncated:
            print(f"Демонстрация окончена. Маятник удержался {total_reward} шагов!")
            break

    env.close()


if __name__ == "__main__":
    # 1. Честно обучаем агента вслепую
    trained_agent = train_agent_honestly()

    # 2. Смотрим на результат его "мозгов" в реальном времени
    run_demonstration(trained_agent)
