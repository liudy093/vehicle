#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PLF拓扑编队MPC控制器测试节点
功能：模拟发布传感器数据、轨迹数据、V2X数据（包括领航车和前车）
用于测试PLF (Predecessor-Leader Following) MPC控制效果
"""

import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from car_interfaces.msg import (
    FusionInterface,
    LocalPathPlanningInterface,
    HmiPlatoonInterface,
    VtxInterface,
    PidInterface
)

# ==============================================================================
# 测试场景配置
# ==============================================================================
TEST_CONFIG = {
    # 测试车辆ID (1=领航车, 2=跟随车1, 3=跟随车2)
    'test_car_id': 1,           # 当前测试的车辆ID

    # 编队模式 (0=单车, 1=编队)
    'platoon_mode': 1,

    # 轨迹参数 (直线轨迹)
    'trajectory_length': 500.0,  # 轨迹总长度 (m)
    'trajectory_points': 1000,   # 轨迹点数 (0.5m/点)
    'ref_speed': 4.0,            # 参考速度 (m/s)

    # 车辆几何参数 (与MPC控制器一致)
    'vehicle_length': 4.4,       # 车辆长度 (m)
    'desired_gap': 5.0,          # 期望净间距 (m)，车尾到车头的空隙

    # 初始状态 (净间距)
    'initial_gap': 10.0,         # 与前车初始净间距 (m)，车尾到车头的空隙
    'initial_speed': 0.0,        # 初始速度 (m/s)

    # 前车行为模式
    'front_car_scenario': 'constant',  # 'constant', 'accelerate', 'brake', 'stop_and_go'

    # 领航车行为模式 (PLF新增)
    'leader_scenario': 'constant',     # 领航车场景

    # 初始化等待时间
    'warmup_time': 3.0,                # MPC控制器预热时间(s)，此期间前车和领航车静止等待

    # 坐标原点 (与MPC控制器一致)
    'lat0': 34.5855425,
    'lon0': 113.6855351,
}

# ==============================================================================
# 车辆行为场景定义
# ==============================================================================
SCENARIOS = {
    'constant': {
        'description': '匀速行驶',
        'speed_profile': lambda t: 4.0,  # 恒定4m/s
    },
    'accelerate': {
        'description': '加速',
        'speed_profile': lambda t: min(2.0 + 0.2 * t, 6.0),  # 从2加速到6m/s
    },
    'brake': {
        'description': '减速',
        'speed_profile': lambda t: max(5.0 - 0.3 * t, 1.0),  # 从5减速到1m/s
    },
    'stop_and_go': {
        'description': '走走停停',
        'speed_profile': lambda t: 3.0 * (1 + math.sin(t * 0.5)),  # 0-6m/s周期变化
    },
    'leader_steady': {
        'description': '领航车稳定',
        'speed_profile': lambda t: 4.0,  # 领航车恒速
    },
    'leader_vary': {
        'description': '领航车变速',
        'speed_profile': lambda t: 4.0 + 1.0 * math.sin(t * 0.3),  # 3-5m/s变化
    },
}


class PLFTestNode(Node):
    """PLF拓扑编队测试数据发布节点"""

    def __init__(self):
        super().__init__('plf_test_node')
        self.get_logger().info("=== PLF编队MPC测试节点启动 ===")

        # 加载配置
        self.cfg = TEST_CONFIG
        self.front_scenario = SCENARIOS[self.cfg['front_car_scenario']]
        self.leader_scenario = SCENARIOS.get(self.cfg['leader_scenario'], SCENARIOS['constant'])
        self.get_logger().info(f"测试车辆ID: {self.cfg['test_car_id']}")
        self.get_logger().info(f"前车场景: {self.front_scenario['description']}")
        self.get_logger().info(f"领航车场景: {self.leader_scenario['description']}")

        # 时间与状态
        self.start_time = time.time()
        self.dt = 0.1  # 10Hz
        self.trajectory_publish_duration = 5.0

        # ====== 车辆几何参数 ======
        self.L = self.cfg['vehicle_length']  # 车长，简写为L
        self.desired_gap = self.cfg['desired_gap']

        # ====== 车辆状态模拟 (使用中心位置，考虑车长) ======
        # 本车状态 (位置为车辆中心)
        self.ego_position = 0.0
        self.ego_speed = self.cfg['initial_speed']

        # 前车状态 (ego_id - 1)
        # 前车中心位置 = 本车中心 + 净间距 + 车长
        # 这样 distance = front_position - ego_position (中心距)
        #      gap = distance - L (净间距)
        self.front_position = self.cfg['initial_gap'] + self.L
        self.front_speed = self.cfg['ref_speed']

        # 领航车状态 (PLF新增: car_id = 1)
        # 对于test_car_id=2: 领航车=前车，位置相同
        # 对于test_car_id=3: 领航车中心 = 本车中心 + 2*(净间距+车长)
        if self.cfg['test_car_id'] == 2:
            self.leader_position = self.front_position  # 领航车就是前车
        else:
            # 3号车: 领航车在前方 2*(gap+L) 处
            self.leader_position = 2 * (self.cfg['initial_gap'] + self.L)
        self.leader_speed = self.cfg['ref_speed']
        self.leader_accel = 0.0

        # MPC输出 (从订阅获取)
        self.mpc_acceleration = 0.0
        self.mpc_throttle = 0
        self.mpc_brake = 0

        # 生成轨迹
        self._generate_trajectory()

        # 创建发布者
        self.pub_fusion = self.create_publisher(FusionInterface, 'fusion_data', 10)
        self.pub_path = self.create_publisher(LocalPathPlanningInterface, 'local_path_planning_data', 10)
        self.pub_hmi = self.create_publisher(HmiPlatoonInterface, 'platoon_state_data', 10)
        self.pub_v2x = self.create_publisher(VtxInterface, 'v2x_to_planner_data', 10)

        # 订阅MPC输出 (用于闭环模拟)
        self.create_subscription(PidInterface, 'pid_data', self.cb_pid_output, 10)

        # 定时器
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # 发布一次轨迹
        self._publish_trajectory()

        self.get_logger().info("=== 开始发布PLF测试数据 ===")

    def _generate_trajectory(self):
        """生成直线测试轨迹"""
        self.traj_lat = []
        self.traj_lon = []
        self.traj_speed = []

        lat0, lon0 = self.cfg['lat0'], self.cfg['lon0']
        length = self.cfg['trajectory_length']
        n_points = self.cfg['trajectory_points']

        lat_per_meter = 1.0 / 111000.0

        for i in range(n_points):
            dist = i * length / (n_points - 1)
            self.traj_lat.append(lat0 + dist * lat_per_meter)
            self.traj_lon.append(lon0)
            self.traj_speed.append(self.cfg['ref_speed'])

    def _publish_trajectory(self, log=True):
        """发布轨迹数据"""
        msg = LocalPathPlanningInterface()
        msg.timestamp = time.time()
        msg.latitude = self.traj_lat
        msg.longitude = self.traj_lon
        msg.speed = self.traj_speed
        self.pub_path.publish(msg)
        if log:
            self.get_logger().info(f"轨迹已发布: {len(self.traj_lat)} 点, 总长 {self.cfg['trajectory_length']}m")

    def timer_callback(self):
        """主循环：更新状态并发布数据"""
        t = time.time() - self.start_time

        # ====== 0. 持续发布轨迹 (前几秒) ======
        if t < self.trajectory_publish_duration:
            self._publish_trajectory(log=False)

        # ====== 1. 更新领航车状态 ======
        # 预热期间领航车保持静止，避免MPC未启动时间距漂移
        if t >= self.cfg['warmup_time']:
            scenario_time = t - self.cfg['warmup_time']
            old_leader_speed = self.leader_speed
            self.leader_speed = self.leader_scenario['speed_profile'](scenario_time)
            self.leader_accel = (self.leader_speed - old_leader_speed) / self.dt
            self.leader_position += self.leader_speed * self.dt
        else:
            # 预热期间领航车静止
            self.leader_speed = 0.0
            self.leader_accel = 0.0

        # ====== 2. 更新前车状态 (仅跟随车需要) ======
        if self.cfg['test_car_id'] > 1:
            # 预热期间前车保持静止
            if t >= self.cfg['warmup_time']:
                scenario_time = t - self.cfg['warmup_time']
                self.front_speed = self.front_scenario['speed_profile'](scenario_time)
                self.front_position += self.front_speed * self.dt
            else:
                # 预热期间前车静止
                self.front_speed = 0.0

        # ====== 3. 更新本车状态 (基于MPC输出的闭环模拟) ======
        self.ego_speed += self.mpc_acceleration * self.dt
        self.ego_speed = max(0.0, min(self.ego_speed, 10.0))
        self.ego_position += self.ego_speed * self.dt

        # ====== 4. 计算间距（与MPC文件保持一致的命名）======
        # 与前车的间距
        distance_front = self.front_position - self.ego_position  # 中心距
        gap_front = distance_front - self.L                       # 净间距

        # 与领航车的间距
        distance_leader = self.leader_position - self.ego_position  # 中心距
        n_between = self.cfg['test_car_id'] - 2                    # 中间车辆数
        gap_leader = distance_leader - (n_between + 1) * self.L     # 净间距

        # ====== 5. 发布本车传感器数据 ======
        self._publish_fusion()

        # ====== 6. 发布编队状态 ======
        self._publish_hmi()

        # ====== 7. 发布V2X数据 (PLF: 同时发布领航车和前车) ======
        if self.cfg['test_car_id'] > 1:
            self._publish_v2x_front()   # 前车V2X
            self._publish_v2x_leader()  # 领航车V2X (PLF新增)

        # ====== 8. 打印状态 ======
        if int(t * 10) % 20 == 0:  # 每2秒打印一次
            # 显示预热状态
            warmup_status = "[预热中]" if t < self.cfg['warmup_time'] else ""

            if self.cfg['test_car_id'] == 1:
                self.get_logger().info(
                    f"[t={t:.1f}s]{warmup_status} 领航车: pos={self.ego_position:.1f}m, v={self.ego_speed:.2f}m/s | "
                    f"ref_v={self.cfg['ref_speed']:.1f}m/s | "
                    f"MPC: a={self.mpc_acceleration:.2f}, thr={self.mpc_throttle}%, brk={self.mpc_brake}%"
                )
            else:
                desired_gap_leader = (self.cfg['test_car_id'] - 1) * self.desired_gap
                self.get_logger().info(
                    f"[t={t:.1f}s]{warmup_status} 本车: pos={self.ego_position:.1f}m, v={self.ego_speed:.2f}m/s | "
                    f"前车: v={self.front_speed:.2f}m/s, gap={gap_front:.2f}m (目标{self.desired_gap:.1f}m) | "
                    f"领航: v={self.leader_speed:.2f}m/s, gap={gap_leader:.2f}m (目标{desired_gap_leader:.1f}m) | "
                    f"MPC: a={self.mpc_acceleration:.2f}"
                )

    def _publish_fusion(self):
        """发布本车GPS/IMU融合数据"""
        msg = FusionInterface()
        msg.timestamp = time.time()

        lat_per_meter = 1.0 / 111000.0
        msg.latitude = self.cfg['lat0'] + self.ego_position * lat_per_meter
        msg.longitude = self.cfg['lon0']

        msg.carspeed = self.ego_speed + np.random.normal(0, 0.05)
        msg.ax = self.mpc_acceleration

        self.pub_fusion.publish(msg)

    def _publish_hmi(self):
        """发布编队状态指令"""
        msg = HmiPlatoonInterface()
        msg.timestamp = time.time()
        msg.platoon_state = self.cfg['platoon_mode']
        msg.platoon_id = self.cfg['test_car_id']
        self.pub_hmi.publish(msg)

    def _publish_v2x_front(self):
        """发布前车V2X数据 (PF部分)"""
        msg = VtxInterface()
        msg.timestamp = time.time()
        msg.car_id = self.cfg['test_car_id'] - 1  # 前车ID
        msg.platoon_id = 1

        lat_per_meter = 1.0 / 111000.0
        msg.latitude = self.cfg['lat0'] + self.front_position * lat_per_meter
        msg.longitude = self.cfg['lon0']

        msg.speed = self.front_speed
        msg.acceleration = 0.0

        self.pub_v2x.publish(msg)

    def _publish_v2x_leader(self):
        """发布领航车V2X数据 (PLF新增)"""
        # 对于2号车，前车是1号车，领航车也是1号车 - 不需要重复发布
        # 对于3号车，前车是2号车，领航车是1号车 - 需要发布
        if self.cfg['test_car_id'] <= 2:
            # 2号车的前车就是领航车，已经在_publish_v2x_front中发布
            return

        msg = VtxInterface()
        msg.timestamp = time.time()
        msg.car_id = 1  # 领航车ID固定为1
        msg.platoon_id = 1

        lat_per_meter = 1.0 / 111000.0
        msg.latitude = self.cfg['lat0'] + self.leader_position * lat_per_meter
        msg.longitude = self.cfg['lon0']

        msg.speed = self.leader_speed
        msg.acceleration = self.leader_accel

        self.pub_v2x.publish(msg)

    def cb_pid_output(self, msg):
        """接收MPC控制器输出，用于闭环模拟"""
        self.mpc_acceleration = msg.acceleration
        self.mpc_throttle = msg.throttle_percentage
        self.mpc_brake = msg.braking_percentage


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PLFTestNode()
        print("\n" + "="*70)
        print("PLF (Predecessor-Leader Following) 编队MPC测试节点已启动")
        print("="*70)
        print(f"测试车辆: {TEST_CONFIG['test_car_id']}号车")
        print(f"编队模式: {'编队' if TEST_CONFIG['platoon_mode'] else '单车'}")
        print(f"前车场景: {SCENARIOS[TEST_CONFIG['front_car_scenario']]['description']}")
        print(f"领航车场景: {SCENARIOS.get(TEST_CONFIG['leader_scenario'], SCENARIOS['constant'])['description']}")
        print(f"车辆长度: {TEST_CONFIG['vehicle_length']}m, 期望净间距: {TEST_CONFIG['desired_gap']}m")
        print("="*70)
        print("\n请在另一个终端启动PLF-MPC控制器:")
        print("  python3 lmq_mpc_plf.py")
        print("\n按 Ctrl+C 停止测试\n")

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[INFO] 测试结束")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
