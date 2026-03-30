#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Description: PLF拓扑纵向编队MPC控制器 (Predecessor-Leader Following)
# PLF拓扑：同时跟踪前车和领航车的间距与速度，满足串稳定性要求
#
# 变量命名规范:
#   - distance: 中心到中心的距离 (弧长差)
#   - gap: 净间距 (车尾到车头的空隙) = distance - vehicle_length
#   - desired_gap: 期望净间距
#
import sys
import os
import rclpy
import time
import yaml
import csv
import datetime
import numpy as np
import cvxpy as cp
from math import *
from collections import deque
from rclpy.node import Node
from easydict import EasyDict
from car_interfaces.msg import *

# 添加 sys.path 配置 (ROS 2 运行时环境需要)
ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

# ==============================================================================
# 可调参数配置区 (Tunable Parameters)
# ==============================================================================

# -------------------- 1. MPC 控制器参数 --------------------
MPC_PARAMS = {
    # 时域参数
    'dt': 0.1,                    # 控制周期 (s), 10Hz
    'N': 10,                      # 预测时域步数

    # 代价函数权重
    'W_ref_velocity': 5.0,        # 轨迹参考速度跟踪权重
    'W_accel': 3.0,               # 加速度惩罚权重 (舒适性)
    'W_jerk': 1.0,                # 加速度变化率权重 (平顺性)
    'W_front_gap': 20.0,          # 前车净间距权重
    'W_leader_gap': 15.0,         # 领航车净间距权重
    'W_leader_velocity': 10.0,    # 领航车速度跟踪权重 (PLF新增)

    # 状态约束
    'a_min': -4.0,                # 最小加速度 (m/s²), 急刹
    'a_max': 2.0,                 # 最大加速度 (m/s²)
    'v_min': 0.0,                 # 最小速度 (m/s)
    'v_max': 10.0,                # 最大速度 (m/s)

    # 求解失败时的回退P控制增益
    'fallback_kp': 0.5,           # P控制增益
}

# -------------------- 2. 编队几何参数 --------------------
PLATOON_PARAMS = {
    'desired_gap': 5.0,           # 期望净间距 (m)，车尾到车头的空隙
    'min_gap': 1.3,               # 最小安全净间距 (m), 碰撞预防硬约束
    'max_platoon_speed': 4.0,     # 编队模式限速 (m/s)
}

# -------------------- 2b. 车辆几何参数 --------------------
VEHICLE_PARAMS = {
    'vehicle_length': 4.4,        # 车辆长度 (m)，SUV约4.5m
    'gps_antenna_offset': 2.0,    # GPS天线到车头的距离 (m)，通常在车顶中部
    # 说明：
    # - GPS测量的是天线位置，不是车头/车尾
    # - 净间距(gap) = 中心距(distance) - 车长(L)
    # - distance = front_arc - ego_arc (弧长差)
    # - gap = distance - L
}

# -------------------- 2c. 车辆状态初始值 --------------------
VEHICLE_STATE = {
    'ego_id': 1,                  # 本车 ID (1-领航车, 2/3-跟随车)
    'platoon_state': 0,           # 初始编队状态 (0-单车模式, 1-编队模式)
}

# -------------------- 3. PLF拓扑参数 --------------------
PLF_PARAMS = {
    # ====== 权重分配系数 ======
    # 间距控制: alpha * 前车间距项 + (1-alpha) * 领航车间距项
    # 速度控制: beta * 前车速度项 + (1-beta) * 领航车速度项 (前车速度项暂未使用)
    'alpha': 0.7,                 # 间距权重分配 (前车占比)
    'beta': 0.3,                  # 速度权重分配 (领航车速度跟踪的启用系数)
    # alpha=0.7: 间距控制中前车占70%，领航车占30%
    # beta=0.3: 速度控制中期望速度占70%，领航车占70%

    # ====== 功能开关 ======
    'use_leader_gap': True,       # 是否启用领航车间距跟踪
    'use_leader_velocity': True,  # 是否启用领航车速度跟踪

    # ====== 间距计算说明 ======
    # 与前车的期望净间距: desired_gap (如8m)
    # 与领航车的期望净间距: (ego_id - 1) * desired_gap
    #   - 2号车: 1 * 8m = 8m (前车=领航车)
    #   - 3号车: 2 * 8m = 16m

    # ====== 串稳定性相关参数 ======
    'headway_time': 0.8,          # 时间头车 (s)，用于CTH间距策略
    'use_cth_gap': False,         # 是否使用CTH间距策略 (True: gap = gap0 + h*v)
}

# -------------------- 4. 低通滤波器参数 --------------------
FILTER_PARAMS = {
    'alpha_speed': 0.3,           # GPS速度滤波系数 (越小越平滑,延迟越大)
    'alpha_accel': 0.5,           # 加速度滤波系数
    'alpha_front_speed': 0.6,     # V2X前车速度滤波系数
    'alpha_leader_speed': 0.6,    # V2X领航车速度滤波系数
}

# -------------------- 5. 执行器映射参数 (SUV车型标定) --------------------
ACTUATOR_PARAMS = {
    'thr_base': 30.0,             # 基础油门 (%)
    'thr_gain': 12.0,             # 油门线性增益
    'thr_nonlinear': 0.1,         # 油门非线性补偿系数
    'thr_low_speed_comp': 2.0,    # 低速补偿增益
    'thr_low_speed_thresh': 3.0,  # 低速补偿阈值 (m/s)
    'brk_base': 15.0,             # 基础刹车 (%)
    'brk_gain': 20.0,             # 刹车线性增益
    'brk_nonlinear': 0.15,        # 刹车非线性补偿系数
    'brk_speed_comp': 3.0,        # 速度补偿增益
    'accel_deadzone': 0.03,       # 加速度死区 (m/s²)
    'speed_deadzone': 0.1,        # 速度死区 (m/s)
    'idle_throttle': 15.0,        # 怠速油门 (%)
    'park_brake': 20.0,           # 驻车刹车 (%)
    'thr_max': 80,                # 最大油门 (%)
    'brk_max': 100,               # 最大刹车 (%)
}

# -------------------- 6. 停车控制参数 --------------------
STOP_PARAMS = {
    'ref_speed_thresh': 0.05,     # 参考速度停车阈值 (m/s)
    'speed_stop_thresh': 0.1,     # 实际速度停稳阈值 (m/s)
    'final_brake': 30,            # 停稳后驻车刹车 (%)
    'brake_ramp_gain': 5.0,       # 刹车递增增益
    'brake_ramp_base': 20,        # 刹车递增基础值
    'brake_ramp_max': 125,        # 刹车递增最大值
    'stop_gear': 2,               # 停车档位
    'drive_gear': 3,              # 行驶档位 (D档)
}

# -------------------- 7. GPS跳点保护参数 --------------------
GPS_FILTER_PARAMS = {
    'jump_threshold': 10.0,       # GPS跳变阈值 (m)
    'smooth_new_weight': 0.3,     # 跳点平滑时新值权重
    'smooth_old_weight': 0.7,     # 跳点平滑时旧值权重
}

# -------------------- 8. 坐标系原点 (WGS84) --------------------
COORD_ORIGIN = {
    'lat0': 34.5855425,           # 原点纬度 (图书馆)
    'lon0': 113.6855351,          # 原点经度
}

# -------------------- 9. 安全兜底参数 --------------------
SAFETY_PARAMS = {
    'no_trajectory_accel': -1.0,  # 无轨迹时的安全减速度 (m/s²)
}

# -------------------- 10. 安全监控参数 --------------------
SAFETY_MONITOR = {
    'mpc_fail_count_limit': 10,    # MPC连续失败次数上限
    'mpc_fail_decel': -2.0,       # MPC连续失败时的安全减速度 (m/s²)
    'critical_gap': 1.0,          # 危险净间距阈值 (m)，小于此值触发紧急制动
    'warning_gap': 1.2,           # 警告净间距阈值 (m)，小于此值触发强制减速
    'emergency_brake': 80,        # 紧急制动刹车百分比 (%)
    'warning_decel': -3.0,        # 警告间距时的减速度 (m/s²)
}

# ==============================================================================
# 1. 工具类：一阶低通滤波器 (EWMA)
# ==============================================================================
class LowPassFilter:
    def __init__(self, alpha, name="Filter"):
        self.alpha = alpha
        self.last_value = 0.0
        self.initialized = False
        self.name = name

    def update(self, current_value):
        if not self.initialized:
            self.last_value = current_value
            self.initialized = True
            return current_value
        self.last_value = self.alpha * current_value + (1 - self.alpha) * self.last_value
        return self.last_value

# ==============================================================================
# 2. 主节点：PLF拓扑编队控制器
# ==============================================================================
class PLFPlatoonMPC(Node):
    def __init__(self, name='pid'):
        super().__init__(name)
        print("DEBUG: PLF-MPC 脚本已经进入初始化流程")
        self.get_logger().info("Initializing PLF Platoon MPC Node...")

        # -------- A. 加载可调参数 --------
        # MPC参数
        self.dt = MPC_PARAMS['dt']
        self.N = MPC_PARAMS['N']
        self.W_ref_velocity = MPC_PARAMS['W_ref_velocity']
        self.W_accel = MPC_PARAMS['W_accel']
        self.W_jerk = MPC_PARAMS['W_jerk']
        self.W_front_gap = MPC_PARAMS['W_front_gap']
        self.W_leader_gap = MPC_PARAMS['W_leader_gap']
        self.W_leader_velocity = MPC_PARAMS['W_leader_velocity']
        self.a_min = MPC_PARAMS['a_min']
        self.a_max = MPC_PARAMS['a_max']
        self.v_min = MPC_PARAMS['v_min']
        self.v_max = MPC_PARAMS['v_max']
        self.fallback_kp = MPC_PARAMS['fallback_kp']

        # 编队参数
        self.desired_gap = PLATOON_PARAMS['desired_gap']
        self.min_gap = PLATOON_PARAMS['min_gap']
        self.max_platoon_speed = PLATOON_PARAMS['max_platoon_speed']

        # PLF参数
        self.alpha = PLF_PARAMS['alpha']
        self.beta = PLF_PARAMS['beta']
        self.use_leader_gap = PLF_PARAMS['use_leader_gap']
        self.use_leader_velocity = PLF_PARAMS['use_leader_velocity']
        self.headway_time = PLF_PARAMS['headway_time']
        self.use_cth_gap = PLF_PARAMS['use_cth_gap']

        # 车辆几何参数
        self.L = VEHICLE_PARAMS['vehicle_length']  # 车长，用L表示简洁

        # 车辆状态初始值
        self.ego_id = VEHICLE_STATE['ego_id']              # 本车ID (由HMI动态更新)
        self.platoon_state = VEHICLE_STATE['platoon_state'] # 编队状态 (由HMI动态更新)

        # -------- B. 传感器滤波模块 --------
        self.speed_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_speed'], name="Speed")
        self.accel_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_accel'], name="Accel")
        self.front_speed_f = LowPassFilter(alpha=FILTER_PARAMS['alpha_front_speed'], name="FrontSpeed")
        self.leader_speed_f = LowPassFilter(alpha=FILTER_PARAMS['alpha_leader_speed'], name="LeaderSpeed")

        # -------- C. 运行状态变量 --------
        # 本车状态
        self.ego_speed = 0.0          # 本车速度 (滤波后)
        self.ego_accel = 0.0          # 本车加速度 (滤波后)
        self.ego_lat, self.ego_lon = 0.0, 0.0
        self.ego_arc = 0.0            # 本车弧长位置
        self.ref_speed = 0.0          # 轨迹参考速度

        # 前车状态
        self.front_speed = 0.0        # 前车速度
        self.front_arc = 0.0          # 前车弧长位置
        self.front_update_time = 0.0

        # 领航车状态
        self.leader_speed = 0.0       # 领航车速度
        self.leader_arc = 0.0         # 领航车弧长位置
        self.leader_accel = 0.0       # 领航车加速度
        self.leader_update_time = 0.0
        self.leader_data_valid = False

        self.braking_distance = 0.0
        self.raw_gps_speed = 0.0

        # 轨迹缓存
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_arc_map = []
        self.trajectory_available = False
        self.last_valid_arc = 0.0

        # -------- 安全监控状态变量 --------
        self.mpc_fail_count = 0
        self.mpc_status = 0
        self.safety_override = False

        # -------- D. 加载配置与初始化记录 --------
        self._load_config()
        self._init_csv_logging()

        # -------- E. 定义 ROS 2 通信 --------
        self._setup_comms()

    def _load_config(self):
        path = os.getcwd() + "/src/pid/pid/config.yaml"
        try:
            with open(path, 'r') as f:
                self.yaml_data = EasyDict(yaml.safe_load(f))
        except Exception as e:
            self.get_logger().error(f"Config load failed: {e}")

    def _setup_comms(self):
        self.pub_control = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timer = self.create_timer(self.dt, self.on_timer_callback)
        self.create_subscription(FusionInterface, 'fusion_data', self.cb_fusion, 10)
        self.create_subscription(LocalPathPlanningInterface, 'local_path_planning_data', self.cb_path, 10)
        self.create_subscription(HmiPlatoonInterface, 'platoon_state_data', self.cb_hmi, 10)
        self.create_subscription(VtxInterface, 'v2x_to_planner_data', self.cb_v2x, 10)

    # ==========================================================================
    # 3. 核心控制逻辑
    # ==========================================================================
    def on_timer_callback(self):
        msg = PidInterface()
        msg.timestamp = time.time()

        if self.ref_speed < STOP_PARAMS['ref_speed_thresh']:
            self._execute_open_loop_stop(msg)
            self.pub_control.publish(msg)
            return

        target_acceleration = 0.0
        if not self.trajectory_available:
            target_acceleration = SAFETY_PARAMS['no_trajectory_accel']
            self.mpc_status = 0
        else:
            target_acceleration = self.solve_mpc_plf()

        if self.mpc_status == 3 and self.safety_override:
            throttle = 0
            brake = SAFETY_MONITOR['emergency_brake']
        else:
            throttle, brake = self.calculate_actuator_cmds(target_acceleration)

        msg.acceleration = target_acceleration
        msg.velocity = self.ref_speed
        msg.throttle_percentage = int(np.clip(throttle, 0, ACTUATOR_PARAMS['thr_max']))
        msg.braking_percentage = int(np.clip(brake, 0, ACTUATOR_PARAMS['brk_max']))
        msg.gear = STOP_PARAMS['drive_gear']

        self.pub_control.publish(msg)
        self._write_log(target_acceleration, msg.throttle_percentage, msg.braking_percentage)

    def solve_mpc_plf(self):
        """
        PLF拓扑MPC求解器

        PLF代价函数设计 (完整版):
        ================================================================================
        J = Σ [ β * W_ref_v * (v - v_ref)²                           # 1. 轨迹参考速度跟踪
              + W_a * a²                                          # 2. 舒适性 (加速度惩罚)
              + W_j * (a[k] - a[k-1])²                            # 3. 平顺性 (加加速度惩罚)
              + α * W_fg * (gap_front - gap_des)²                 # 4. 前车净间距保持
              + (1-α) * W_lg * (gap_leader - gap_leader_des)²     # 5. 领航车净间距保持
              + (1-β) * W_lv * (v - v_leader)² ]                      # 6. 领航车速度跟踪

        变量说明:
        - gap_front: 与前车的净间距 = front_arc - ego_arc - L
        - gap_leader: 与领航车的净间距 = leader_arc - ego_arc - (中间车辆数+1)*L
        - gap_des: 期望净间距 (默认8m)
        - gap_leader_des: 与领航车期望净间距 = (ego_id-1) * gap_des
        - v_leader: 领航车速度
        - α: 间距权重分配系数 (前车/领航车)
        - β: 领航车速度跟踪系数
        ================================================================================
        """
        sm = SAFETY_MONITOR
        L = self.L  # 车长

        # ====== 计算间距 ======
        # 中心距 (distance) = 弧长差
        distance_front = self.front_arc - self.ego_arc
        distance_leader = self.leader_arc - self.ego_arc

        # 净间距 (gap) = 中心距 - 车长
        gap_front = distance_front - L

        # 与领航车的净间距需要扣除中间所有车辆的长度
        # 中间车辆数 = ego_id - 2 (例如3号车与1号车之间有1辆2号车)
        num_vehicles_between = self.ego_id - 2
        gap_leader = distance_leader - (num_vehicles_between + 1) * L

        # 期望净间距
        gap_des = self.desired_gap
        gap_leader_des = (self.ego_id - 1) * self.desired_gap

        # ====== 调试日志 ======
        self.get_logger().info(
            f"[PLF] id={self.ego_id} | "
            f"前车: dist={distance_front:.1f}m, gap={gap_front:.1f}m (目标{gap_des:.0f}m) | "
            f"领航: dist={distance_leader:.1f}m, gap={gap_leader:.1f}m (目标{gap_leader_des:.0f}m) | "
            f"v_leader={self.leader_speed:.2f}m/s"
        )

        # ====== 1号车（领航车）：仅速度跟踪 ======
        if self.ego_id == 1:
            return self._solve_leader_mpc()

        # ====== 安全检查：净间距异常检测 ======
        if self.platoon_state == 1 and self.ego_id > 1:
            if 0 < gap_front < sm['critical_gap']:
                self.mpc_status = 3
                self.safety_override = True
                self.get_logger().warn(f"[EMERGENCY] gap过小: {gap_front:.2f}m，紧急制动!")
                return self.a_min

            if 0 < gap_front < sm['warning_gap']:
                self.mpc_status = 3
                self.get_logger().warn(f"[WARNING] gap偏小: {gap_front:.2f}m，强制减速")
                return sm['warning_decel']

        # ====== PLF MPC求解 ======
        u = cp.Variable(self.N)         # 加速度
        v = cp.Variable(self.N + 1)     # 速度
        s = cp.Variable(self.N + 1)     # 弧长位置

        constraints = [
            v[0] == self.ego_speed,
            s[0] == self.ego_arc,
            u >= self.a_min, u <= self.a_max,
            v >= self.v_min, v <= self.v_max
        ]

        cost = 0
        alpha = self.alpha
        beta = self.beta

        for k in range(self.N):
            # 离散运动学模型
            constraints += [
                v[k+1] == v[k] + u[k] * self.dt,
                s[k+1] == s[k] + v[k] * self.dt
            ]

            # ===== 代价函数 1: 舒适性约束 =====
            cost += self.W_accel * cp.square(u[k])
            if k > 0:
                cost += self.W_jerk * cp.square(u[k] - u[k-1])

            # ===== 代价函数 2-6: PLF编队控制 (仅跟随车生效) =====
            if self.platoon_state == 1 and self.front_arc > self.ego_arc:

                # --- 代价3: 前车净间距保持 (权重 α) ---
                # 预测前车未来弧长位置
                pred_front_arc = self.front_arc + self.front_speed * (k+1) * self.dt
                # 预测本车未来弧长位置为s[k+1]
                # 预测净间距
                pred_gap_front = pred_front_arc - s[k+1] - L

                # 期望净间距 (支持CTH策略)
                if self.use_cth_gap:
                    pred_gap_des = self.desired_gap + self.headway_time * v[k+1]
                else:
                    pred_gap_des = self.desired_gap

                # 碰撞预防硬约束
                constraints += [pred_gap_front >= self.min_gap]

                # 前车净间距代价 (权重α)
                cost += alpha * self.W_front_gap * cp.square(pred_gap_front - pred_gap_des)

                # --- 代价4: 领航车净间距保持 (权重 1-α) ---
                if self.use_leader_gap and self.leader_data_valid and self.leader_arc > 0:
                    # 预测领航车未来弧长位置
                    pred_leader_arc = self.leader_arc + self.leader_speed * (k+1) * self.dt

                    # 预测净间距
                    n_between = self.ego_id - 2
                    pred_gap_leader = pred_leader_arc - s[k+1] - (n_between + 1) * L

                    # 期望与领航车净间距
                    pred_gap_leader_des = (self.ego_id - 1) * self.desired_gap

                    # 领航车净间距代价 (权重 1-α)
                    cost += (1 - alpha) * self.W_leader_gap * cp.square(pred_gap_leader - pred_gap_leader_des)

                # --- 代价5: 领航车速度跟踪 (权重 1-β) ---
                if self.use_leader_velocity and self.leader_data_valid and self.leader_speed > 0:
                    # 预测领航车未来速度 (假设加速度预测)
                    pred_leader_v = self.leader_speed + self.leader_accel * (k+1) * self.dt
                    pred_leader_v = max(0.0, min(pred_leader_v, self.v_max))

                    # 领航车速度跟踪代价
                    cost += (1 - beta) * self.W_leader_velocity * cp.square(v[k+1] - pred_leader_v)
                    # --- 代价6: 轨迹参考速度跟踪 (权重 β)---
                    cost += beta * self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)
                else:
                    # --- 代价6: 仅轨迹参考速度跟踪---
                    cost += self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True)
            if u.value is not None:
                self.mpc_fail_count = 0
                self.mpc_status = 1
                self.safety_override = False
                return u.value[0]
        except Exception as e:
            self.get_logger().error(f"PLF-MPC Solver Error: {e}")

        # ====== MPC求解失败处理 ======
        self.mpc_fail_count += 1
        self.mpc_status = 2

        if self.mpc_fail_count >= sm['mpc_fail_count_limit']:
            self.get_logger().error(f"[SAFETY] MPC连续失败 {self.mpc_fail_count} 次，触发安全减速!")
            self.mpc_status = 3
            return sm['mpc_fail_decel']

        return (self.ref_speed - self.ego_speed) * self.fallback_kp

    def _solve_leader_mpc(self):
        """领航车MPC：仅速度跟踪"""
        u = cp.Variable(self.N)
        v = cp.Variable(self.N + 1)

        constraints = [
            v[0] == self.ego_speed,
            u >= self.a_min, u <= self.a_max,
            v >= self.v_min, v <= self.v_max
        ]

        cost = 0
        for k in range(self.N):
            constraints += [v[k+1] == v[k] + u[k] * self.dt]
            cost += self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)
            cost += self.W_accel * cp.square(u[k])
            if k > 0:
                cost += self.W_jerk * cp.square(u[k] - u[k-1])

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True)
            if u.value is not None:
                self.mpc_status = 1
                return u.value[0]
        except Exception as e:
            self.get_logger().error(f"Leader MPC Error: {e}")

        self.mpc_status = 2
        return (self.ref_speed - self.ego_speed) * self.fallback_kp

    def calculate_actuator_cmds(self, a):
        v = self.ego_speed
        ap = ACTUATOR_PARAMS

        if abs(v) < ap['speed_deadzone'] and abs(a) < ap['accel_deadzone']:
            return 0.0, ap['park_brake']

        if a > ap['accel_deadzone']:
            low_speed_comp = max(0, ap['thr_low_speed_thresh'] - v) * ap['thr_low_speed_comp']
            thr = ap['thr_base'] + a * ap['thr_gain'] * (1 + ap['thr_nonlinear'] * a) + low_speed_comp
            return thr, 0.0
        elif a < -ap['accel_deadzone']:
            brk = ap['brk_base'] + abs(a) * ap['brk_gain'] * (1 + ap['brk_nonlinear'] * abs(a)) + v * ap['brk_speed_comp']
            return 0.0, brk

        return ap['idle_throttle'], 0.0

    def _execute_open_loop_stop(self, msg):
        sp = STOP_PARAMS
        if self.ego_speed < sp['speed_stop_thresh']:
            msg.throttle_percentage = 0
            msg.braking_percentage = sp['final_brake']
        else:
            self.braking_distance += self.ego_speed * self.dt
            brk = int(sp['brake_ramp_gain'] * self.braking_distance + sp['brake_ramp_base'])
            msg.braking_percentage = min(sp['brake_ramp_max'], brk)
        msg.gear = sp['stop_gear']

    # ==========================================================================
    # 4. 坐标转换
    # ==========================================================================
    def lla_to_enu(self, lat, lon):
        lat0, lon0 = COORD_ORIGIN['lat0'], COORD_ORIGIN['lon0']
        A, F = 6378137.0, 1/298.257223563
        E2 = F * (2 - F)
        D2R = pi / 180.0

        phi0, lam0 = lat0 * D2R, lon0 * D2R
        N0 = A / sqrt(1 - E2 * sin(phi0)**2)
        X0 = N0 * cos(phi0) * cos(lam0)
        Y0 = N0 * cos(phi0) * sin(lam0)
        Z0 = N0 * (1 - E2) * sin(phi0)

        phi, lam = lat * D2R, lon * D2R
        N = A / sqrt(1 - E2 * sin(phi)**2)
        X = N * cos(phi) * cos(lam)
        Y = N * cos(phi) * sin(lam)
        Z = N * (1 - E2) * sin(phi)

        dx, dy, dz = X-X0, Y-Y0, Z-Z0
        east = -sin(lam0) * dx + cos(lam0) * dy
        north = -sin(phi0)*cos(lam0)*dx - sin(phi0)*sin(lam0)*dy + cos(phi0)*dz
        return east, north

    def update_arc_length(self, lat, lon):
        if not self.trajectory_available or len(self.trajectory_arc_map) == 0:
            return 0.0

        e, n = self.lla_to_enu(lat, lon)
        min_dist = float('inf')
        nearest_idx = 0

        for i in range(len(self.trajectory_x)):
            dist = sqrt((e - self.trajectory_x[i])**2 + (n - self.trajectory_y[i])**2)
            if dist < min_dist:
                min_dist, nearest_idx = dist, i

        new_arc = self.trajectory_arc_map[nearest_idx]

        gp = GPS_FILTER_PARAMS
        if self.last_valid_arc > 0:
            jump = abs(new_arc - self.last_valid_arc)
            if jump > gp['jump_threshold']:
                new_arc = gp['smooth_new_weight'] * new_arc + gp['smooth_old_weight'] * self.last_valid_arc

        self.last_valid_arc = new_arc
        return new_arc

    # ==========================================================================
    # 5. 回调函数
    # ==========================================================================
    def cb_fusion(self, msg):
        self.raw_gps_speed = msg.carspeed
        self.ego_speed = self.speed_filter.update(msg.carspeed)
        self.ego_accel = self.accel_filter.update(msg.ax)
        self.ego_lat, self.ego_lon = msg.latitude, msg.longitude

        if self.trajectory_available:
            self.ego_arc = self.update_arc_length(self.ego_lat, self.ego_lon)

    def cb_v2x(self, msg):
        if self.platoon_state != 1:
            return

        # 处理领航车信息 (car_id=1)
        if msg.car_id == 1 and self.ego_id > 1:
            self.leader_speed = self.leader_speed_f.update(msg.speed)
            self.leader_accel = msg.acceleration
            if self.trajectory_available:
                self.leader_arc = self.update_arc_length(msg.latitude, msg.longitude)
            self.leader_update_time = time.time()
            self.leader_data_valid = True

        # 处理前车信息 (car_id = ego_id - 1)
        if msg.car_id == self.ego_id - 1:
            self.front_speed = self.front_speed_f.update(msg.speed)
            if self.trajectory_available:
                self.front_arc = self.update_arc_length(msg.latitude, msg.longitude)
            self.front_update_time = time.time()

    def cb_path(self, msg):
        if len(msg.latitude) > 0 and not self.trajectory_available:
            self.ref_speed = msg.speed[0]
            self.trajectory_x, self.trajectory_y = [], []
            self.trajectory_arc_map = [0.0]

            ex, nx = self.lla_to_enu(msg.latitude[0], msg.longitude[0])
            self.trajectory_x.append(ex)
            self.trajectory_y.append(nx)

            dist = 0.0
            for i in range(1, len(msg.latitude)):
                e, n = self.lla_to_enu(msg.latitude[i], msg.longitude[i])
                dist += sqrt((e-self.trajectory_x[-1])**2 + (n-self.trajectory_y[-1])**2)
                self.trajectory_x.append(e)
                self.trajectory_y.append(n)
                self.trajectory_arc_map.append(dist)

            self.trajectory_available = True

    def cb_hmi(self, msg):
        self.platoon_state = msg.platoon_state
        self.ego_id = msg.platoon_id

    # ==========================================================================
    # 6. 数据记录
    # ==========================================================================
    def _init_csv_logging(self):
        path = os.path.expanduser('/home/nvidia/AutoDrive/src/pid/data')
        os.makedirs(path, exist_ok=True)
        ts = datetime.datetime.now().strftime("%m%d_%H%M")
        self.csv_file = f"{path}/lmq_mpc_plf_car{self.ego_id}_{ts}.csv"

        with open(self.csv_file, 'w') as f:
            writer = csv.writer(f)
            # CSV列名：使用gap表示净间距，distance表示中心距
            writer.writerow([
                't_s', 'id', 'mode', 'ego_arc', 'ego_v', 'target_a', 'thr', 'brk',
                'lat', 'lon',
                'gap_front', 'gap_leader',           # 净间距
                'distance_front', 'distance_leader', # 中心距
                'ref_v', 'front_v', 'leader_v',
                'mpc_status', 'alpha', 'beta'
            ])
        self.start_time = time.time()

    def _write_log(self, a, t, b):
        L = self.L

        if self.ego_id == 1:
            gap_front = 0.0
            gap_leader = 0.0
            distance_front = 0.0
            distance_leader = 0.0
        else:
            # 中心距
            distance_front = self.front_arc - self.ego_arc
            distance_leader = self.leader_arc - self.ego_arc
            # 净间距
            gap_front = distance_front - L
            n_between = self.ego_id - 2
            gap_leader = distance_leader - (n_between + 1) * L

        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 2),
                self.ego_id, self.platoon_state,
                round(self.ego_arc, 2), round(self.ego_speed, 2),
                round(a, 3), int(t), int(b),
                round(self.ego_lat, 7), round(self.ego_lon, 7),
                round(gap_front, 2), round(gap_leader, 2),
                round(distance_front, 2), round(distance_leader, 2),
                round(self.ref_speed, 2), round(self.front_speed, 2), round(self.leader_speed, 2),
                self.mpc_status, self.alpha, self.beta
            ])


def main(args=None):
    rclpy.init(args=args)

    try:
        rosNode = PLFPlatoonMPC()
        print(">>> [SUCCESS] PLF编队控制节点已成功初始化")
        print(">>> PLF代价函数:")
        print("    J = β*W_ref_v*(v-v_ref)² + W_a*a² + W_j*jerk²")
        print("      + α*W_fg*(gap_front - gap_des)²")
        print("      + (1-α)*W_lg*(gap_leader - gap_leader_des)²")
        print("      + (1-β)*W_lv*(v - v_leader)²")
        print(f">>> 参数:")
        print(f"    - α={PLF_PARAMS['alpha']} (间距: 前车{PLF_PARAMS['alpha']*100:.0f}%/领航车{(1-PLF_PARAMS['alpha'])*100:.0f}%)")
        print(f"    - β={PLF_PARAMS['beta']} (领航车速度跟踪系数)")
        print(f"    - 车长L={VEHICLE_PARAMS['vehicle_length']}m")
        print(f"    - 期望净间距gap_des={PLATOON_PARAMS['desired_gap']}m")
        print(">>> 正在等待数据...")

        rclpy.spin(rosNode)

    except KeyboardInterrupt:
        print("\n[INFO] 用户按下 Ctrl+C，正在停止节点...")
    except Exception as e:
        print(f"\n[FATAL] 节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(">>> 正在关闭 ROS 2 资源...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
