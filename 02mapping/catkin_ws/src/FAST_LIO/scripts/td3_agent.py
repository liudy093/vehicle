import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

def mlp(in_dim, out_dim):
    return nn.Sequential(
        nn.Linear(in_dim, 128),
        nn.ReLU(),
        nn.Linear(128, 128),
        nn.ReLU(),
        nn.Linear(128, out_dim)
    )

class TD3Agent:
    def __init__(self, state_dim=2, action_dim=1, action_limit=1.0):
        self.actor = mlp(state_dim, action_dim)
        self.actor_target = mlp(state_dim, action_dim)
        self.actor_target.load_state_dict(self.actor.state_dict())

        self.critic1 = mlp(state_dim + action_dim, 1)
        self.critic2 = mlp(state_dim + action_dim, 1)
        self.critic1_target = mlp(state_dim + action_dim, 1)
        self.critic2_target = mlp(state_dim + action_dim, 1)
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())

        self.opt_actor = optim.Adam(self.actor.parameters(), lr=1e-4)
        self.opt_critic = optim.Adam(
            list(self.critic1.parameters()) + list(self.critic2.parameters()),
            lr=1e-3
        )

        self.replay = []
        self.max_replay = 20000

        self.action_limit = action_limit
        self.gamma = 0.99
        self.tau = 0.005

    def select_action(self, s):
        s = torch.FloatTensor(s).unsqueeze(0)
        a = self.actor(s).detach().numpy()[0]
        a = np.clip(a, 0.05, 1.0)
        return a

    def store(self, s, a, r, s2):
        self.replay.append((s, a, r, s2))
        if len(self.replay) > self.max_replay:
            self.replay.pop(0)

    def update(self, batch_size=64):
        if len(self.replay) < batch_size:
            return

        batch = np.random.choice(len(self.replay), batch_size)
        states, actions, rewards, next_states = [], [], [], []

        for i in batch:
            s, a, r, s2 = self.replay[i]
            states.append(s)
            actions.append(a)
            rewards.append(r)
            next_states.append(s2)

        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions).unsqueeze(1)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)

        # target actions
        next_actions = self.actor_target(next_states)
        next_actions = torch.clamp(next_actions, 0.05, 1.0)

        target_q1 = self.critic1_target(torch.cat([next_states, next_actions], dim=1))
        target_q2 = self.critic2_target(torch.cat([next_states, next_actions], dim=1))
        target_q = rewards + self.gamma * torch.min(target_q1, target_q2)

        # update critics
        current_q1 = self.critic1(torch.cat([states, actions], dim=1))
        current_q2 = self.critic2(torch.cat([states, actions], dim=1))
        critic_loss = nn.MSELoss()(current_q1, target_q.detach()) + nn.MSELoss()(current_q2, target_q.detach())

        self.opt_critic.zero_grad()
        critic_loss.backward()
        self.opt_critic.step()

        # actor update
        actor_loss = -self.critic1(torch.cat([states, self.actor(states)], dim=1)).mean()
        self.opt_actor.zero_grad()
        actor_loss.backward()
        self.opt_actor.step()

        # soft update
        for t, s in zip(self.actor_target.parameters(), self.actor.parameters()):
            t.data.copy_(t.data * (1 - self.tau) + s.data * self.tau)

        for t, s in zip(self.critic1_target.parameters(), self.critic1.parameters()):
            t.data.copy_(t.data * (1 - self.tau) + s.data * self.tau)

        for t, s in zip(self.critic2_target.parameters(), self.critic2.parameters()):
            t.data.copy_(t.data * (1 - self.tau) + s.data * self.tau)

