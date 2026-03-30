#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np

# ============================================================
# Actor-Critic 网络
# ============================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim=2, action_dim=1, hidden_size=128):
        super(ActorCritic, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)

        self.mean = nn.Linear(hidden_size, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

        self.v_head = nn.Linear(hidden_size, 1)

    def forward(self, x):
        if not isinstance(x, torch.Tensor):
            x = torch.tensor(x, dtype=torch.float32)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mean = self.mean(x)
        std = torch.exp(self.log_std)
        value = self.v_head(x)
        return mean, std, value

    def get_action(self, state):
        state = np.array(state, dtype=np.float32)
        mean, std, _ = self.forward(state)
        dist = torch.distributions.Normal(mean, std)

        action = dist.sample()
        log_prob = dist.log_prob(action).sum()

        return action.detach().numpy(), log_prob.detach().numpy()


# ============================================================
# PPO buffer
# ============================================================
class PPOBuffer:
    def __init__(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.log_probs = []

    def clear(self):
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.dones.clear()
        self.log_probs.clear()


# ============================================================
# PPO Agent
# ============================================================
class PPOAgent:
    def __init__(self, state_dim=2, action_dim=1, lr=3e-4, gamma=0.99, eps_clip=0.2, K_epochs=4):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs

        self.buffer = PPOBuffer()
        self.policy = ActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)

    # 存储数据
    def store(self, s, a, r, d, logp):
        self.buffer.states.append(s)
        self.buffer.actions.append(a)
        self.buffer.rewards.append(r)
        self.buffer.dones.append(d)
        self.buffer.log_probs.append(logp)

    # 计算折扣回报
    def compute_returns(self, rewards, dones):
        R = 0
        returns = []
        for r, d in zip(reversed(rewards), reversed(dones)):
            R = r + self.gamma * R * (1 - d)
            returns.insert(0, R)
        return np.array(returns, dtype=np.float32)

    # PPO 更新
    def update(self):
        if len(self.buffer.states) == 0:
            return

        # 转为 numpy → torch
        states = torch.tensor(np.array(self.buffer.states), dtype=torch.float32)
        actions = torch.tensor(np.array(self.buffer.actions), dtype=torch.float32)
        old_log_probs = torch.tensor(np.array(self.buffer.log_probs), dtype=torch.float32)

        rewards = np.array(self.buffer.rewards)
        dones = np.array(self.buffer.dones)

        returns = self.compute_returns(rewards, dones)
        returns = torch.tensor(returns, dtype=torch.float32)

        with torch.no_grad():
            _, _, values = self.policy(states)
            advantages = returns - values.squeeze()

        for _ in range(self.K_epochs):
            mean, std, values = self.policy(states)
            dist = torch.distributions.Normal(mean, std)

            log_probs = dist.log_prob(actions).sum(-1)
            ratios = torch.exp(log_probs - old_log_probs)

            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages

            loss = -torch.min(surr1, surr2).mean() + \
                   F.mse_loss(values.squeeze(), returns)

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        self.buffer.clear()


# ============================================================
# ROS Node
# ============================================================
class CompressRLNode:
    def __init__(self):
        rospy.init_node("ppo_pointcloud_compress")

        self.agent = PPOAgent()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.pub_rate = rospy.Publisher("/compress_rate", Float32, queue_size=1)

        self.v = 0.0
        self.w = 0.0

        self.step = 0
        self.rate = rospy.Rate(10)

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def run(self):
        while not rospy.is_shutdown():

            # 状态 = 速度 + 角速度
            state = [self.v, self.w]

            action, logp = self.agent.policy.get_action(state)
            compress_rate = float(np.clip(action[0], 0.01, 1.0))

            self.pub_rate.publish(compress_rate)

            # -----------------------------------------------------
            # 🚀 新 reward：越激烈运动 → 压缩率应越低
            # -----------------------------------------------------
            motion_intensity = abs(self.v) + abs(self.w)
            reward = - motion_intensity * compress_rate
            # -----------------------------------------------------

            self.agent.store(state, [compress_rate], reward, False, logp)

            self.step += 1
            if self.step % 5 == 0:
                print(f"Step {self.step}")
                print(f" State: v={self.v:.2f}, w={self.w:.2f}")
                print(f" Compress rate: {compress_rate:.3f}")
                print(f" Reward: {reward:.3f}")
                print("-" * 40)

            if len(self.agent.buffer.states) >= 50:
                self.agent.update()
                print("PPO updated!")
                print("=" * 40)

            self.rate.sleep()


# ============================================================
if __name__ == "__main__":
    node = CompressRLNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

