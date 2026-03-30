#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, ByteMultiArray

class CompressionEnv:
    def __init__(self):
        rospy.init_node("compression_env")

        self.last_compressed_size = None
        self.last_action = 1.0

        rospy.Subscriber("/compressed_size", Float32, self.size_cb)

        self.pub_action = rospy.Publisher("/compress_rate", Float32, queue_size=10)

    def size_cb(self, msg):
        self.last_compressed_size = msg.data

    def step(self, action):
        """
        action ∈ [0,1] → 压缩率 r
        """
        action = np.clip(action, 0.05, 1.0)
        self.last_action = action

        # 发布压缩率
        self.pub_action.publish(Float32(action))

        # 等待 C++ 节点输出压缩结果
        rospy.sleep(0.05)

        if self.last_compressed_size is None:
            return None, 0, False

        # reward: 惩罚大文件 + 奖励高质量（高 r）
        reward = -self.last_compressed_size * 0.0001 + action * 0.5

        obs = np.array([self.last_compressed_size, action], dtype=np.float32)

        done = False
        return obs, reward, done

    def reset(self):
        self.last_compressed_size = None
        rospy.sleep(0.1)
        return np.array([0, 1], dtype=np.float32)


# 测试环境是否可运行
if __name__ == "__main__":
    env = CompressionEnv()
    rate = rospy.Rate(5)
    a = 1.0
    while not rospy.is_shutdown():
        obs, r, done = env.step(a)
        print("obs:", obs, "reward:", r)
        a = np.random.uniform(0.3, 1.0)
        rate.sleep()

