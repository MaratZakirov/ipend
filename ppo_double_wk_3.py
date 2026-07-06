import time
import gymnasium as gym
import numpy as np
import torch
from torch import nn
import torch.optim as optim
from torch.distributions import Normal
import os
import pickle

GAMMA = 0.99
CLIP_RATIO = 0.2
LR = 1e-4  # УМЕНЬШИЛ! Было 3e-3
N_STEPS = 2048
K_EPOCHS = 2
BATCH_SIZE = 64
ENTROPY_COEF = 0.001  # УВЕЛИЧИЛ! Для лучшего исследования
MAX_GRAD_NORM = 0.5
VF_COEF = 0.5
TAU = 0.95

device = torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")


class PolicyNetwork(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, 64),  # УВЕЛИЧИЛ размер
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
        )
        self.mu_head = nn.Linear(64, act_dim)
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, state):
        if state.dim() == 1:
            state = state.unsqueeze(0)
        x = self.shared(state)
        mu = torch.tanh(self.mu_head(x))
        std = torch.exp(torch.clamp(self.log_std, -3.0, 1.0)).expand_as(mu)
        return mu, std

    def sample_action(self, state):
        if not torch.is_tensor(state):
            state = torch.tensor(state, dtype=torch.float32, device=device)
        if state.dim() == 1:
            state = state.unsqueeze(0)

        mu, std = self.forward(state)
        dist = Normal(mu, std)
        raw_action = dist.rsample()
        action = torch.tanh(raw_action)
        log_prob = dist.log_prob(raw_action) - torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return action.squeeze(0), raw_action.squeeze(0), log_prob.squeeze(0), entropy.squeeze(0), std.squeeze(0)

    def deterministic_action(self, state):
        state_t = torch.tensor(state, dtype=torch.float32, device=device)
        if state_t.dim() == 1:
            state_t = state_t.unsqueeze(0)
        with torch.no_grad():
            mu, _ = self.forward(state_t)
            return torch.tanh(mu).squeeze(0).cpu().numpy()


class ValueNetwork(nn.Module):
    def __init__(self, obs_dim):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, 64),  # УВЕЛИЧИЛ размер
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
        )
        self.value_head = nn.Linear(64, 1)

    def forward(self, state):
        if state.dim() == 1:
            state = state.unsqueeze(0)
        x = self.shared(state)
        value = self.value_head(x)
        return value

    def get_value(self, state):
        state_t = torch.tensor(state, dtype=torch.float32, device=device)
        if state_t.dim() == 1:
            state_t = state_t.unsqueeze(0)
        with torch.no_grad():
            value = self.forward(state_t)
            return value.squeeze(0)


class PPOAgent:
    def __init__(self, obs_dim, act_dim):
        self.policy = PolicyNetwork(obs_dim, act_dim).to(device)
        self.critic = ValueNetwork(obs_dim).to(device)
        self.optimizer_po = optim.Adam(self.policy.parameters(), lr=LR)
        self.optimizer_cr = optim.Adam(self.critic.parameters(), lr=LR * 3)  # Критик учится быстрее
        self.obs_dim = obs_dim
        self.act_dim = act_dim

        # Для хранения истории обучения
        self.history = {
            'episode_rewards': [],
            'moving_avg': [],
            'sigmas': []
        }

    def compute_gae(self, rewards, dones, values, last_value=0):
        advantages = []
        gae = 0
        values = values + [last_value]

        for i in reversed(range(len(rewards))):
            delta = rewards[i] + GAMMA * values[i + 1] * (1 - dones[i]) - values[i]
            gae = delta + GAMMA * TAU * (1 - dones[i]) * gae
            advantages.insert(0, gae)

        advantages = np.array(advantages, dtype=np.float32)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        return advantages

    def update_policy(self, states, raw_actions, old_log_probs, rewards, dones, values, last_value):
        states_t = torch.tensor(np.array(states), dtype=torch.float32, device=device)
        raw_actions_t = torch.tensor(np.array(raw_actions), dtype=torch.float32, device=device)
        old_log_probs_t = torch.tensor(np.array(old_log_probs), dtype=torch.float32, device=device)
        values_t = torch.tensor(np.array(values), dtype=torch.float32, device=device)

        advantages_t = torch.tensor(
            self.compute_gae(rewards, dones, values, last_value),
            dtype=torch.float32,
            device=device
        )

        dataset_size = states_t.size(0)

        for _ in range(K_EPOCHS):
            permutation = torch.randperm(dataset_size, device=device)
            for start in range(0, dataset_size, BATCH_SIZE):
                idx = permutation[start:start + BATCH_SIZE]

                b_states = states_t[idx]
                b_raw_actions = raw_actions_t[idx]
                b_old_log_probs = old_log_probs_t[idx]
                b_adv = advantages_t[idx]
                b_values = values_t[idx]

                mu, std = self.policy(b_states)
                new_values = self.critic(b_states)
                dist = Normal(mu, std)

                b_actions = torch.tanh(b_raw_actions)
                new_log_probs = dist.log_prob(b_raw_actions) - torch.log(1 - b_actions.pow(2) + 1e-6)
                new_log_probs = new_log_probs.sum(dim=-1)

                entropy = dist.entropy().sum(dim=-1)

                ratio = torch.exp(new_log_probs - b_old_log_probs)
                surr1 = ratio * b_adv
                surr2 = torch.clamp(ratio, 1 - CLIP_RATIO, 1 + CLIP_RATIO) * b_adv
                policy_loss = -torch.mean(torch.min(surr1, surr2))

                value_loss = nn.MSELoss()(new_values.squeeze(-1), b_values)
                entropy_loss = -ENTROPY_COEF * torch.mean(entropy)

                loss_po = policy_loss + entropy_loss

                self.optimizer_po.zero_grad()
                self.optimizer_cr.zero_grad()
                loss_po.backward()
                value_loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), MAX_GRAD_NORM)
                nn.utils.clip_grad_norm_(self.critic.parameters(), MAX_GRAD_NORM)
                self.optimizer_po.step()
                self.optimizer_cr.step()

    def save(self, filepath="ppo_checkpoint.pth"):
        """Сохраняет модель и историю обучения"""
        checkpoint = {
            'policy_state_dict': self.policy.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'optimizer_po_state_dict': self.optimizer_po.state_dict(),
            'optimizer_cr_state_dict': self.optimizer_cr.state_dict(),
            'history': self.history,
            'obs_dim': self.obs_dim,
            'act_dim': self.act_dim,
        }
        torch.save(checkpoint, filepath)
        print(f"✅ Модель сохранена в {filepath}")

    def load(self, filepath="ppo_checkpoint.pth"):
        if not os.path.exists(filepath):
            print(f"❌ Файл {filepath} не найден")
            return False

        # Явно указываем weights_only=False
        checkpoint = torch.load(filepath, map_location=device, weights_only=False)
        self.policy.load_state_dict(checkpoint['policy_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.optimizer_po.load_state_dict(checkpoint['optimizer_po_state_dict'])
        self.optimizer_cr.load_state_dict(checkpoint['optimizer_cr_state_dict'])
        self.history = checkpoint['history']
        print(f"✅ Модель загружена из {filepath}")
        return True

    def save_history(self, filepath="training_history.pkl"):
        """Сохраняет только историю обучения (без весов)"""
        with open(filepath, 'wb') as f:
            pickle.dump(self.history, f)
        print(f"✅ История сохранена в {filepath}")

    def load_history(self, filepath="training_history.pkl"):
        """Загружает историю обучения"""
        if not os.path.exists(filepath):
            print(f"❌ Файл {filepath} не найден")
            return False
        with open(filepath, 'rb') as f:
            self.history = pickle.load(f)
        print(f"✅ История загружена из {filepath}")
        return True


def train_agent(load_from=None, save_every=50):
    SUCCESS_THRESHOLD = 15000.0
    MOVING_AVG_WINDOW = 100

    env = gym.make("InvertedDoublePendulum-v5", render_mode=None)
    state, info = env.reset()
    obs_dim = np.asarray(state, dtype=np.float32).shape[0]
    act_dim = env.action_space.shape[0]

    agent = PPOAgent(obs_dim, act_dim)

    # Загрузка если есть
    start_episode = 0
    if load_from and os.path.exists(load_from):
        agent.load(load_from)
        start_episode = len(agent.history['episode_rewards'])
        print(f"Продолжаем обучение с эпизода {start_episode}")

    print(f"Начало обучения PPO. obs_dim={obs_dim}, act_dim={act_dim}")
    print(f"LR_policy={LR}, LR_critic={LR * 3}, ENTROPY_COEF={ENTROPY_COEF}\n")

    global_step = 0
    episode = start_episode

    # Восстанавливаем историю
    episode_rewards_history = agent.history.get('episode_rewards', [])

    while True:
        states, raw_actions, old_log_probs, rewards, dones, values = [], [], [], [], [], []
        episode_reward = 0.0
        episode_sigmas = []
        episode_len = 0
        last_value = 0

        # Сбор траектории
        while len(states) < N_STEPS:
            state_np = np.asarray(state, dtype=np.float32)
            state_t = torch.tensor(state_np, dtype=torch.float32, device=device)

            with torch.no_grad():
                value_t = agent.critic(state_t)
                action_t, raw_action_t, log_prob_t, _, std_t = agent.policy.sample_action(state_t)
                value = value_t.item()
                episode_sigmas.append(std_t.mean().item())

            action = action_t.cpu().numpy()
            raw_action = raw_action_t.cpu().numpy()

            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            states.append(state_np)
            raw_actions.append(raw_action)
            old_log_probs.append(log_prob_t.item())
            rewards.append(reward)
            dones.append(done)
            values.append(value)

            episode_reward += reward
            episode_len += 1
            global_step += 1
            state = next_state

            if done:
                if len(states) > 0:
                    last_state = np.asarray(state, dtype=np.float32)
                    last_state_t = torch.tensor(last_state, dtype=torch.float32, device=device)
                    with torch.no_grad():
                        last_value_t = agent.critic(last_state_t)
                        last_value = last_value_t.item()

                state, info = env.reset()

                # Сохраняем награду за эпизод
                episode_rewards_history.append(episode_reward)
                if len(episode_rewards_history) > MOVING_AVG_WINDOW:
                    episode_rewards_history.pop(0)

                # Сохраняем историю
                agent.history['episode_rewards'] = episode_rewards_history
                agent.history['sigmas'].append(np.mean(episode_sigmas) if episode_sigmas else 0)

                episode_reward = 0.0
                episode_sigmas = []
                episode_len = 0

        # Обновляем политику
        agent.update_policy(states, raw_actions, old_log_probs, rewards, dones, values, last_value)

        moving_avg_reward = float(np.mean(episode_rewards_history)) if episode_rewards_history else 0
        agent.history['moving_avg'].append(moving_avg_reward)

        current_lr_po = agent.optimizer_po.param_groups[0]["lr"]
        current_lr_cr = agent.optimizer_cr.param_groups[0]["lr"]

        if episode % 15 == 0 and episode_rewards_history:
            mean_sigma = float(np.mean(episode_sigmas)) if episode_sigmas else 0.0
            print(
                f"Эпизод: {episode:6d} | Шагов: {episode_len:<4d} | "
                f"Награда: {episode_reward:<7.1f} | "
                f"Средняя Sigma: {mean_sigma:.4f} | "
                f"Средняя награда({len(episode_rewards_history):<3d}): {moving_avg_reward:.1f} | "
                f"LR: {current_lr_po:.6f}/{current_lr_cr:.6f}"
            )

        # Сохраняем чекпоинт
        if episode % save_every == 0 and episode > 0:
            agent.save(f"ppo_checkpoint_ep{episode}.pth")
            # Сохраняем только последний чекпоинт как основной
            agent.save("ppo_checkpoint_latest.pth")

        if moving_avg_reward >= SUCCESS_THRESHOLD:
            print(f"\n🎉 [УСПЕХ] Средняя награда достигла {moving_avg_reward:.1f}")
            agent.save("ppo_success.pth")
            env.close()
            return agent

        episode += 1


def run_double_pendulum_demo(agent):
    print("\nЗапуск демонстрации обученного агента в окне MuJoCo...")
    env = gym.make("InvertedDoublePendulum-v5", render_mode="human")
    observation, info = env.reset()

    while True:
        action = agent.policy.deterministic_action(observation)
        observation, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.02)

        if terminated or truncated:
            print("Система потеряла баланс, перезапуск...")
            observation, info = env.reset()


if __name__ == "__main__":
    # Для продолжения обучения:
    #agent = train_agent(load_from="ppo_checkpoint_latest.pth")

    # Для обучения с нуля:
    agent = train_agent()

    run_double_pendulum_demo(agent)