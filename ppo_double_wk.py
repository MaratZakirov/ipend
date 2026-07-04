import time
import gymnasium as gym
import numpy as np
import torch
from torch import nn
import torch.optim as optim
from torch.distributions import Normal

ENV_NAME = "InvertedDoublePendulum-v5"
GAMMA = 0.99
CLIP_RATIO = 0.2
LR = 3e-4
N_STEPS = 2048
K_EPOCHS = 4
BATCH_SIZE = 64
ENTROPY_COEF = 0.0001
MAX_GRAD_NORM = 0.5

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class PolicyNetwork(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
        )
        self.mu_head = nn.Linear(64, act_dim)
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, state):
        if state.dim() == 1:
            state = state.unsqueeze(0)
        feat = self.shared(state)
        mu = torch.tanh(self.mu_head(feat))
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
        return action.squeeze(0), raw_action.squeeze(0), log_prob.squeeze(0), entropy.squeeze(0)

    def deterministic_action(self, state):
        state_t = torch.tensor(state, dtype=torch.float32, device=device)
        if state_t.dim() == 1:
            state_t = state_t.unsqueeze(0)
        with torch.no_grad():
            mu, _ = self.forward(state_t)
            return torch.tanh(mu).squeeze(0).cpu().numpy()


class PPOAgent:
    def __init__(self, obs_dim, act_dim):
        self.policy = PolicyNetwork(obs_dim, act_dim).to(device)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=LR)

    def compute_returns(self, rewards, dones):
        returns = []
        R = 0.0
        for r, d in zip(reversed(rewards), reversed(dones)):
            if d:
                R = 0.0
            R = r + GAMMA * R
            returns.insert(0, R)
        returns = np.array(returns, dtype=np.float32)
        return (returns - returns.mean()) / (returns.std() + 1e-8)

    def update_policy(self, states, raw_actions, old_log_probs, rewards, dones):
        states_t = torch.tensor(np.array(states), dtype=torch.float32, device=device)
        raw_actions_t = torch.tensor(np.array(raw_actions), dtype=torch.float32, device=device)
        old_log_probs_t = torch.tensor(np.array(old_log_probs), dtype=torch.float32, device=device)
        advantages_t = torch.tensor(self.compute_returns(rewards, dones), dtype=torch.float32, device=device)

        dataset_size = states_t.size(0)
        for _ in range(K_EPOCHS):
            permutation = torch.randperm(dataset_size, device=device)
            for start in range(0, dataset_size, BATCH_SIZE):
                idx = permutation[start:start + BATCH_SIZE]

                b_states = states_t[idx]
                b_raw_actions = raw_actions_t[idx]
                b_old_log_probs = old_log_probs_t[idx]
                b_adv = advantages_t[idx]

                mu, std = self.policy(b_states)
                dist = Normal(mu, std)

                b_actions = torch.tanh(b_raw_actions)
                new_log_probs = dist.log_prob(b_raw_actions) - torch.log(1 - b_actions.pow(2) + 1e-6)
                new_log_probs = new_log_probs.sum(dim=-1)

                entropy = dist.entropy().sum(dim=-1)

                ratio = torch.exp(new_log_probs - b_old_log_probs)
                surr1 = ratio * b_adv
                surr2 = torch.clamp(ratio, 1 - CLIP_RATIO, 1 + CLIP_RATIO) * b_adv
                policy_loss = -torch.mean(torch.min(surr1, surr2))
                entropy_loss = -ENTROPY_COEF * torch.mean(entropy)
                loss = policy_loss + entropy_loss

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), MAX_GRAD_NORM)
                self.optimizer.step()


def train_agent():
    SUCCESS_THRESHOLD = 15000.0
    MOVING_AVG_WINDOW = 20

    episode_rewards_history = []

    env = gym.make(ENV_NAME, render_mode=None)

    state, info = env.reset()
    obs_dim = np.asarray(state, dtype=np.float32).shape[0]
    act_dim = env.action_space.shape[0]

    agent = PPOAgent(obs_dim, act_dim)
    print(f"Начало обучения PPO. obs_dim={obs_dim}, act_dim={act_dim}. Мониторинг среднего значения Sigma включен...\n")

    global_step = 0
    episode = 0

    while True:
        states, raw_actions, old_log_probs, rewards, dones = [], [], [], [], []
        episode_reward = 0.0
        episode_sigmas = []
        episode_len = 0

        while len(states) < N_STEPS:
            state_np = np.asarray(state, dtype=np.float32)
            state_t = torch.tensor(state_np, dtype=torch.float32, device=device)

            with torch.no_grad():
                _, std_t = agent.policy(state_t)
                episode_sigmas.append(std_t.mean().item())
                action_t, raw_action_t, log_prob_t, _ = agent.policy.sample_action(state_t)

            action = action_t.cpu().numpy()
            raw_action = raw_action_t.cpu().numpy()

            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            states.append(state_np)
            raw_actions.append(raw_action)
            old_log_probs.append(log_prob_t.item())
            rewards.append(reward)
            dones.append(done)

            episode_reward += reward
            episode_len += 1
            global_step += 1
            state = next_state

            if done:
                if episode % 15 == 0:
                    mean_sigma = float(np.mean(episode_sigmas)) if episode_sigmas else 0.0
                    print(
                        f"Эпизод: {global_step:5d} | Шагов: {episode_len:<4d} | "
                        f"Награда: {episode_reward:<7.1f} | Средняя Sigma (Шум): {mean_sigma:.4f}"
                    )
                state, info = env.reset()
                episode_reward = 0.0
                episode_sigmas = []
                episode_len = 0

        agent.update_policy(states, raw_actions, old_log_probs, rewards, dones)

        episode_rewards_history.append(episode_reward)

        if len(episode_rewards_history) > MOVING_AVG_WINDOW:
            episode_rewards_history.pop(0)

        moving_avg_reward = sum(episode_rewards_history) / len(episode_rewards_history)

        if episode % 15 == 0:
            print(
                f"Эпизод: {global_step:5d} | Шагов: {episode_len:<4d} | "
                f"Награда: {episode_reward:<7.1f} | "
                f"Средняя Sigma (Шум): {mean_sigma:.4f} | "
                f"Средняя награда({len(episode_rewards_history)}): {moving_avg_reward:.1f}"
            )

        if moving_avg_reward >= SUCCESS_THRESHOLD:
            print(
                f"\n[УСПЕХ] Средняя награда за последние {len(episode_rewards_history)} эпизодов достигла {moving_avg_reward:.1f}")
            env.close()
            return agent

        episode += 1

def run_double_pendulum_demo(agent):
    print("\nЗапуск демонстрации обученного агента в окне MuJoCo...")
    env = gym.make(ENV_NAME, render_mode="human")
    observation, info = env.reset()

    while True:
        action = agent.policy.deterministic_action(observation)
        observation, reward, terminated, truncated, info = env.step(action)
        time.sleep(0.02)

        if terminated or truncated:
            print("Система потеряла баланс, перезапуск...")
            observation, info = env.reset()


if __name__ == "__main__":
    agent = train_agent()
    run_double_pendulum_demo(agent)