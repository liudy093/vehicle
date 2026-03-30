#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
编队MPC控制器测试节点
功能：模拟发布传感器数据、轨迹数据、V2X数据，用于容器环境下测试MPC控制效果
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
    'test_car_id': 2,           # 当前测试的车辆ID

    # 编队模式 (0=单车, 1=编队)
    'platoon_mode': 1,

    # 轨迹参数 (直线轨迹)
    'trajectory_length': 500.0,  # 轨迹总长度 (m)，加长以支持更长测试
    'trajectory_points': 1000,   # 轨迹点数，提高弧长投影精度 (0.5m/点)
    'ref_speed': 4.0,            # 参考速度 (m/s)

    # 车辆几何参数 (与MPC控制器一致)
    'vehicle_length': 4.4,       # 车辆长度 (m)
    'desired_gap': 5.0,          # 期望净间距 (m)，车尾到车头的空隙

    # 初始状态
    'initial_gap': 10.0,         # 初始净间距 (m)，车尾到车头的空隙
    'initial_speed': 0.0,        # 初始速度 (m/s)

    # 前车行为模式
    'front_car_scenario': 'constant',  # 'constant', 'accelerate', 'brake', 'stop_and_go'

    # 初始化等待时间
    'warmup_time': 3.0,              # MPC控制器预热时间(s)，此期间前车静止等待

    # 坐标原点 (与MPC控制器一致)
    'lat0': 34.5855425,
    'lon0': 113.6855351,
}

# ==============================================================================
# 前车行为场景定义
# ==============================================================================
SCENARIOS = {
    'constant': {
        'description': '前车匀速行驶',
        'speed_profile': lambda t: 4.0,  # 恒定4m/s
    },
    'accelerate': {
        'description': '前车加速',
        'speed_profile': lambda t: min(2.0 + 0.2 * t, 6.0),  # 从2加速到6m/s
    },
    'brake': {
        'description': '前车减速',
        'speed_profile': lambda t: max(5.0 - 0.3 * t, 1.0),  # 从5减速到1m/s
    },
    'stop_and_go': {
        'description': '前车走走停停',
        'speed_profile': lambda t: 3.0 * (1 + math.sin(t * 0.5)),  # 0-6m/s周期变化
    },
}


class PlatoonTestNode(Node):
    """编队测试数据发布节点"""

    def __init__(self):
        super().__init__('platoon_test_node')
        self.get_logger().info("=== 编队MPC测试节点启动 ===")

        # 加载配置
        self.cfg = TEST_CONFIG
        self.scenario = SCENARIOS[self.cfg['front_car_scenario']]
        self.get_logger().info(f"测试车辆ID: {self.cfg['test_car_id']}")
        self.get_logger().info(f"前车场景: {self.scenario['description']}")

        # 时间与状态
        self.start_time = time.time()
        self.dt = 0.1  # 10Hz
        self.trajectory_publish_duration = 5.0  # 前5秒持续发布轨迹

        # 车辆几何参数
        self.L = self.cfg['vehicle_length']

        # 车辆状态 (模拟)
        # 位置为车辆中心位置
        self.ego_position = 0.0      # 本车弧长位置（中心）
        self.ego_speed = self.cfg['initial_speed']
        # 前车中心位置 = 本车中心 + 净间距 + 车长
        # 这样 distance = front_position - ego_position (中心距)
        #      gap = distance - L (净间距)
        self.front_position = self.cfg['initial_gap'] + self.L  # 前车位置（中心）
        self.front_speed = self.cfg['ref_speed']

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

        self.get_logger().info("=== 开始发布测试数据 ===")

    def _generate_trajectory(self):
        """生成直线测试轨迹"""
        self.traj_lat = []
        self.traj_lon = []
        self.traj_speed = []

        # 沿纬度方向的直线轨迹
        lat0, lon0 = self.cfg['lat0'], self.cfg['lon0']
        length = self.cfg['trajectory_length']
        n_points = self.cfg['trajectory_points']

        # 1度纬度约111km
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
        # 确保MPC节点能收到轨迹数据，即使它启动较晚
        if t < self.trajectory_publish_duration:
            self._publish_trajectory(log=False)

        # ====== 1. 更新前车状态 (仅跟随车需要) ======
        if self.cfg['test_car_id'] > 1:
            # 预热期间前车保持静止，避免MPC未启动时间距漂移
            if t >= self.cfg['warmup_time']:
                # 调整时间基准，使场景从预热结束后开始
                scenario_time = t - self.cfg['warmup_time']
                self.front_speed = self.scenario['speed_profile'](scenario_time)
                self.front_position += self.front_speed * self.dt
            else:
                # 预热期间前车静止
                self.front_speed = 0.0

        # ====== 2. 更新本车状态 (基于MPC输出的闭环模拟) ======
        # 简化车辆动力学：v' = a, s' = v
        self.ego_speed += self.mpc_acceleration * self.dt
        self.ego_speed = max(0.0, min(self.ego_speed, 10.0))  # 限制速度范围
        self.ego_position += self.ego_speed * self.dt

        # ====== 3. 计算间距（与MPC文件保持一致的命名）======
        # distance: 中心距（弧长差）
        distance_front = self.front_position - self.ego_position
        # gap: 净间距（车尾到车头的空隙）= 中心距 - 车长
        gap_front = distance_front - self.L

        # ====== 4. 发布本车传感器数据 ======
        self._publish_fusion()

        # ====== 5. 发布编队状态 ======
        self._publish_hmi()

        # ====== 6. 发布前车V2X数据 ======
        if self.cfg['test_car_id'] > 1:
            self._publish_v2x()

        # ====== 7. 打印状态 ======
        if int(t * 10) % 20 == 0:  # 每2秒打印一次
            # 显示预热状态
            warmup_status = "[预热中]" if t < self.cfg['warmup_time'] else ""

            if self.cfg['test_car_id'] == 1:
                # 1号车（领航车）：只显示本车状态，无间距信息
                self.get_logger().info(
                    f"[t={t:.1f}s]{warmup_status} 领航车: pos={self.ego_position:.1f}m, v={self.ego_speed:.2f}m/s | "
                    f"ref_v={self.cfg['ref_speed']:.1f}m/s | "
                    f"MPC输出: a={self.mpc_acceleration:.2f}, thr={self.mpc_throttle}%, brk={self.mpc_brake}%"
                )
            else:
                # 2/3号车（跟随车）：显示间距信息
                self.get_logger().info(
                    f"[t={t:.1f}s]{warmup_status} 本车: pos={self.ego_position:.1f}m, v={self.ego_speed:.2f}m/s | "
                    f"前车: pos={self.front_position:.1f}m, v={self.front_speed:.2f}m/s | "
                    f"distance={distance_front:.2f}m, gap={gap_front:.2f}m (目标{self.cfg['desired_gap']:.1f}m) | "
                    f"MPC输出: a={self.mpc_acceleration:.2f}, thr={self.mpc_throttle}%, brk={self.mpc_brake}%"
                )

    def _publish_fusion(self):
        """发布本车GPS/IMU融合数据"""
        msg = FusionInterface()
        msg.timestamp = time.time()

        # 将弧长位置转换为经纬度
        lat_per_meter = 1.0 / 111000.0
        msg.latitude = self.cfg['lat0'] + self.ego_position * lat_per_meter
        msg.longitude = self.cfg['lon0']

        # 速度和加速度
        msg.carspeed = self.ego_speed + np.random.normal(0, 0.05)  # 添加小噪声
        msg.ax = self.mpc_acceleration

        self.pub_fusion.publish(msg)

    def _publish_hmi(self):
        """发布编队状态指令"""
        msg = HmiPlatoonInterface()
        msg.timestamp = time.time()
        msg.platoon_state = self.cfg['platoon_mode']
        msg.platoon_id = self.cfg['test_car_id']
        self.pub_hmi.publish(msg)

    def _publish_v2x(self):
        """发布前车V2X数据"""
        msg = VtxInterface()
        msg.timestamp = time.time()
        msg.car_id = self.cfg['test_car_id'] - 1  # 前车ID
        msg.platoon_id = 1

        # 前车位置
        lat_per_meter = 1.0 / 111000.0
        msg.latitude = self.cfg['lat0'] + self.front_position * lat_per_meter
        msg.longitude = self.cfg['lon0']

        # 前车速度
        msg.speed = self.front_speed
        msg.acceleration = 0.0

        self.pub_v2x.publish(msg)

    def cb_pid_output(self, msg):
        """接收MPC控制器输出，用于闭环模拟"""
        self.mpc_acceleration = msg.acceleration
        self.mpc_throttle = msg.throttle_percentage
        self.mpc_brake = msg.braking_percentage


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PlatoonTestNode()
        print("\n" + "="*60)
        print("编队MPC测试节点已启动")
        print("="*60)
        print(f"测试车辆: {TEST_CONFIG['test_car_id']}号车")
        print(f"编队模式: {'编队' if TEST_CONFIG['platoon_mode'] else '单车'}")
        print(f"前车场景: {SCENARIOS[TEST_CONFIG['front_car_scenario']]['description']}")
        print("="*60)
        print("\n请在另一个终端启动MPC控制器:")
        print("  python3 lmq_mpc.py")
        print("\n按 Ctrl+C 停止测试\n")

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[INFO] 测试结束")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
