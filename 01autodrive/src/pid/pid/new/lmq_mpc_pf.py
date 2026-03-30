#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Description: PF拓扑纵向编队MPC控制器 (Predecessor Following)
# PF拓扑：仅跟踪前车的间距与速度
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
    'desired_gap': 5,           # 期望净间距 (m)，车尾到车头的空隙 (原中心距8m - 车长4.4m = 3.6m)
    'min_gap': 1.3,               # 最小安全净间距 (m), 碰撞预防硬约束 (原中心距5m - 车长4.4m = 0.6m)
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
    'ego_id': 2,                  # 本车 ID (1-领航车, 2/3-跟随车)
    'platoon_state': 0,           # 初始编队状态 (0-单车模式, 1-编队模式)
}

# -------------------- 3. 低通滤波器参数 --------------------
FILTER_PARAMS = {
    'alpha_speed': 0.3,           # GPS速度滤波系数 (越小越平滑,延迟越大)
    'alpha_accel': 0.5,           # 加速度滤波系数
    'alpha_front_speed': 0.6,     # V2X前车速度滤波系数
}

# -------------------- 4. 执行器映射参数 (SUV车型标定) --------------------
ACTUATOR_PARAMS = {
    # 加速映射: thr = base + a * gain * (1 + nonlinear*a) + low_speed_comp
    'thr_base': 30.0,             # 基础油门 (%)
    'thr_gain': 12.0,             # 油门线性增益
    'thr_nonlinear': 0.1,         # 油门非线性补偿系数
    'thr_low_speed_comp': 2.0,    # 低速补偿增益
    'thr_low_speed_thresh': 3.0,  # 低速补偿阈值 (m/s)

    # 减速映射: brk = base + |a| * gain * (1 + nonlinear*|a|) + v*speed_comp
    'brk_base': 15.0,             # 基础刹车 (%)
    'brk_gain': 20.0,             # 刹车线性增益
    'brk_nonlinear': 0.15,        # 刹车非线性补偿系数
    'brk_speed_comp': 3.0,        # 速度补偿增益

    # 死区阈值
    'accel_deadzone': 0.05,       # 加速度死区 (m/s²)
    'speed_deadzone': 0.1,        # 速度死区 (m/s)

    # 怠速与驻车
    'idle_throttle': 15.0,        # 怠速油门 (%)
    'park_brake': 20.0,           # 驻车刹车 (%)

    # 输出限幅
    'thr_max': 80,                # 最大油门 (%)
    'brk_max': 100,               # 最大刹车 (%)
}

# -------------------- 5. 停车控制参数 --------------------
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

# -------------------- 6. GPS跳点保护参数 --------------------
GPS_FILTER_PARAMS = {
    'jump_threshold': 10.0,       # GPS跳变阈值 (m)
    'smooth_new_weight': 0.3,     # 跳点平滑时新值权重
    'smooth_old_weight': 0.7,     # 跳点平滑时旧值权重
}

# -------------------- 7. 坐标系原点 (WGS84) --------------------
COORD_ORIGIN = {
    'lat0': 34.5855425,           # 原点纬度 (图书馆)
    'lon0': 113.6855351,          # 原点经度
}

# -------------------- 8. 安全兜底参数 --------------------
SAFETY_PARAMS = {
    'no_trajectory_accel': -1.0,  # 无轨迹时的安全减速度 (m/s²)
}

# -------------------- 9. 安全监控参数 --------------------
SAFETY_MONITOR = {
    # MPC求解失败保护
    'mpc_fail_count_limit': 10,    # MPC连续失败次数上限
    'mpc_fail_decel': -2.0,       # MPC连续失败时的安全减速度 (m/s²)

    # 净间距异常保护（使用gap而非spacing）
    'critical_gap': 1.0,         # 危险净间距阈值 (m)，小于此值触发紧急制动 
    'warning_gap': 1.2,           # 警告净间距阈值 (m)，小于此值触发强制减速 
    'emergency_brake': 80,        # 紧急制动刹车百分比 (%)
    'warning_decel': -3.0,        # 警告间距时的减速度 (m/s²)
}

# ==============================================================================
# 1. 工具类：一阶低通滤波器 (EWMA - Exponentially Weighted Moving Average)
# 作用：从底层过滤 GPS 速度波动和传感器噪声，保证 MPC 输入平滑
# 公式：y[k] = α * x[k] + (1 - α) * y[k-1]
# ==============================================================================
class LowPassFilter:
    def __init__(self, alpha, name="Filter"):
        self.alpha = alpha  # 滤波系数，越小越平滑但延迟越高
        self.last_value = 0.0
        self.initialized = False
        self.name = name

    def update(self, current_value):
        if not self.initialized:
            self.last_value = current_value
            self.initialized = True
            return current_value
        # 公式：y[k] = α * x[k] + (1 - α) * y[k-1]
        self.last_value = self.alpha * current_value + (1 - self.alpha) * self.last_value
        return self.last_value

# ==============================================================================
# 2. 主节点：PF拓扑编队控制器
# ==============================================================================
class PFPlatoonMPC(Node):
    def __init__(self, name='pid'):
        super().__init__(name)
        print("DEBUG: PF-MPC 脚本已经进入初始化流程")
        self.get_logger().info("Initializing PF Platoon MPC Node...")

        # -------- A. 加载可调参数 --------
        # MPC参数
        self.dt = MPC_PARAMS['dt']
        self.N = MPC_PARAMS['N']
        self.W_ref_velocity = MPC_PARAMS['W_ref_velocity']
        self.W_accel = MPC_PARAMS['W_accel']
        self.W_jerk = MPC_PARAMS['W_jerk']
        self.W_front_gap = MPC_PARAMS['W_front_gap']
        self.a_min = MPC_PARAMS['a_min']
        self.a_max = MPC_PARAMS['a_max']
        self.v_min = MPC_PARAMS['v_min']
        self.v_max = MPC_PARAMS['v_max']
        self.fallback_kp = MPC_PARAMS['fallback_kp']

        # 编队参数
        self.desired_gap = PLATOON_PARAMS['desired_gap']
        self.min_gap = PLATOON_PARAMS['min_gap']
        self.max_platoon_speed = PLATOON_PARAMS['max_platoon_speed']

        # 车辆几何参数
        self.L = VEHICLE_PARAMS['vehicle_length']  # 车长，用L表示简洁

        # 车辆状态初始值
        self.ego_id = VEHICLE_STATE['ego_id']              # 本车ID (由HMI动态更新)
        self.platoon_state = VEHICLE_STATE['platoon_state'] # 编队状态 (由HMI动态更新)

        # -------- B. 传感器滤波模块 --------
        self.speed_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_speed'], name="Speed")
        self.accel_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_accel'], name="Accel")
        self.front_speed_f = LowPassFilter(alpha=FILTER_PARAMS['alpha_front_speed'], name="FrontSpeed")

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

        self.braking_distance = 0.0   # 停车阶段开环累计距离
        self.raw_gps_speed = 0.0      # 原始GPS速度 (未滤波)

        # 轨迹缓存
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_arc_map = []
        self.trajectory_available = False
        self.last_valid_arc = 0.0

        # -------- 安全监控状态变量 --------
        self.mpc_fail_count = 0           # MPC连续失败计数
        self.mpc_status = 0               # MPC求解状态: 0=未求解, 1=成功, 2=失败回退, 3=紧急制动
        self.safety_override = False      # 安全覆盖标志 (True时忽略MPC输出，直接紧急制动)

        # -------- D. 加载配置与初始化记录 --------
        self._load_config()
        self._init_csv_logging() # 详尽记录器

        # -------- E. 定义 ROS 2 通信 --------
        self._setup_comms()

    def _load_config(self):
        """加载本地 YAML 参数"""
        path = os.getcwd() + "/src/pid/pid/config.yaml"
        try:
            with open(path, 'r') as f:
                self.yaml_data = EasyDict(yaml.safe_load(f))
        except Exception as e:
            self.get_logger().error(f"Config load failed: {e}")

    def _setup_comms(self):
        """配置发布者、订阅者及定时器"""
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
        """主控制循环：处理停车 -> MPC 求解 -> 执行器映射"""
        msg = PidInterface()
        msg.timestamp = time.time()

        # --- A. 终端安全制动判断  ---
        if self.ref_speed < STOP_PARAMS['ref_speed_thresh']:
            self._execute_open_loop_stop(msg)
            self.pub_control.publish(msg)
            return

        # --- B. 正常 MPC 控制流程 ---
        target_acceleration = 0.0
        if not self.trajectory_available:
            # 无轨迹时的安全兜底：轻微减速
            target_acceleration = SAFETY_PARAMS['no_trajectory_accel']
            self.mpc_status = 0  # 未求解状态
        else:
            # 调用 PF-MPC 求解器计算最优加速度
            target_acceleration = self.solve_mpc_problem()

        # --- C. 执行器非线性映射 ---
        # 如果是紧急制动状态(gap过小)，直接使用紧急刹车
        if self.mpc_status == 3 and self.safety_override:
            throttle = 0
            brake = SAFETY_MONITOR['emergency_brake']
        else:
            # 正常情况：通过非线性映射将加速度转换为油门/刹车百分比
            throttle, brake = self.calculate_actuator_cmds(target_acceleration)

        # --- D. 封装控制指令并发布 ---
        msg.acceleration = target_acceleration
        msg.velocity = self.ref_speed
        msg.throttle_percentage = int(np.clip(throttle, 0, ACTUATOR_PARAMS['thr_max']))
        msg.braking_percentage = int(np.clip(brake, 0, ACTUATOR_PARAMS['brk_max']))
        msg.gear = STOP_PARAMS['drive_gear']  # 正常行驶 D 档
        
        self.pub_control.publish(msg)
        
        # 实时记录所有核心指标
        self._write_log(target_acceleration, msg.throttle_percentage, msg.braking_percentage)

    def solve_mpc_problem(self):
        """
        PF拓扑MPC求解器

        PF代价函数设计:
        ================================================================================
        J = Σ [ W_ref_v * (v - v_ref)²                    # 1. 轨迹参考速度跟踪
              + W_a * a²                                   # 2. 舒适性 (加速度惩罚)
              + W_j * (a[k] - a[k-1])²                    # 3. 平顺性 (加加速度惩罚)
              + W_fg * (gap_front - gap_des)² ]           # 4. 前车净间距保持

        变量说明:
        - gap_front: 与前车的净间距 = front_arc - ego_arc - L
        - gap_des: 期望净间距 (原中心距8m - 车长4.4m = 3.6m)
        - min_gap: 最小安全净间距 (原中心距5m - 车长4.4m = 0.6m)
        ================================================================================
        """
        sm = SAFETY_MONITOR
        L = self.L  # 车长

        # ====== 计算间距 ======
        # 中心距 (distance) = 弧长差
        distance_front = self.front_arc - self.ego_arc
        # 净间距 (gap) = 中心距 - 车长
        gap_front = distance_front - L

        # ====== 调试日志 ======
        self.get_logger().info(
            f"[PF] id={self.ego_id} | "
            f"前车: dist={distance_front:.1f}m, gap={gap_front:.1f}m (目标{self.desired_gap:.1f}m)"
        )

        # ====== 1号车（领航车）：仅速度跟踪 ======
        if self.ego_id == 1:
            return self._solve_leader_mpc()

        # ====== 安全检查：净间距异常检测 ======
        if self.platoon_state == 1 and self.ego_id > 1:
            # 使用净间距进行安全检查
            if 0 < gap_front < sm['critical_gap']:
                self.mpc_status = 3
                self.safety_override = True
                self.get_logger().warn(f"[EMERGENCY] gap过小: {gap_front:.2f}m，紧急制动!")
                return self.a_min

            if 0 < gap_front < sm['warning_gap']:
                self.mpc_status = 3
                self.get_logger().warn(f"[WARNING] gap偏小: {gap_front:.2f}m，强制减速")
                return sm['warning_decel']

        # ====== PF MPC求解 ======
        # 定义优化变量
        u = cp.Variable(self.N)         # 加速度序列 (控制输入)
        v = cp.Variable(self.N + 1)     # 速度序列 (状态变量)
        s = cp.Variable(self.N + 1)     # 弧长位置序列 (状态变量)

        # 初始状态约束和边界约束
        constraints = [
            v[0] == self.ego_speed,               # 初始速度
            s[0] == self.ego_arc,                 # 初始弧长
            u >= self.a_min, u <= self.a_max,     # 加速度物理限制
            v >= self.v_min, v <= self.v_max      # 速度限制
        ]

        # 构建代价函数
        cost = 0
        for k in range(self.N):  # 预测时域内逐步优化
            # 离散运动学模型
            constraints += [
                v[k+1] == v[k] + u[k] * self.dt,
                s[k+1] == s[k] + v[k] * self.dt
            ]

            # ===== 代价函数 1: 轨迹参考速度跟踪 =====
            cost += self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)

            # ===== 代价函数 2: 舒适性约束 =====
            cost += self.W_accel * cp.square(u[k])
            if k > 0:
                cost += self.W_jerk * cp.square(u[k] - u[k-1])

            # ===== 代价函数 3: PF编队间距控制 (仅跟随车生效) =====
            if self.platoon_state == 1 and self.front_arc > self.ego_arc:
                # 预测前车未来弧长位置 (假设前车匀速运动)
                pred_front_arc = self.front_arc + self.front_speed * (k+1) * self.dt
                # 预测本车与前车的净间距 (车尾到车头的空隙)
                pred_gap_front = pred_front_arc - s[k+1] - L

                # 碰撞预防硬约束：净间距必须大于等于最小安全净间距
                constraints += [pred_gap_front >= self.min_gap]
                # 前车净间距跟踪代价：惩罚与期望净间距的偏差
                cost += self.W_front_gap * cp.square(pred_gap_front - self.desired_gap)

        # ====== 求解优化问题 ======
        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            # 使用OSQP求解器 (开源二次规划求解器，适合实时MPC)
            prob.solve(solver=cp.OSQP, warm_start=True)
            if u.value is not None:
                # MPC求解成功
                self.mpc_fail_count = 0      # 重置失败计数
                self.mpc_status = 1          # 成功状态
                self.safety_override = False # 取消安全覆盖
                return u.value[0]            # 返回第一步的最优加速度
        except Exception as e:
            self.get_logger().error(f"PF-MPC Solver Error: {e}")

        # ====== MPC求解失败处理 ======
        self.mpc_fail_count += 1
        self.mpc_status = 2  # 失败回退状态

        # 连续失败次数过多，触发安全减速
        if self.mpc_fail_count >= sm['mpc_fail_count_limit']:
            self.get_logger().error(f"[SAFETY] MPC连续失败 {self.mpc_fail_count} 次，触发安全减速!")
            self.mpc_status = 3
            return sm['mpc_fail_decel']

        # 普通回退：使用简单P控制器
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
        """
        SUV 车型非线性油门/刹车映射公式

        将MPC输出的加速度指令映射为实际的油门/刹车百分比
        考虑了车辆动力学的非线性特性、低速补偿和速度相关的刹车力

        Args:
            a: 目标加速度 (m/s²)

        Returns:
            (throttle, brake): 油门百分比, 刹车百分比
        """
        v = self.ego_speed
        ap = ACTUATOR_PARAMS

        # 停车驻车：速度和加速度都接近0时
        if abs(v) < ap['speed_deadzone'] and abs(a) < ap['accel_deadzone']:
            return 0.0, ap['park_brake']

        # 加速映射：thr = base + a * gain * (1 + nonlinear*a) + low_speed_comp
        if a > ap['accel_deadzone']:
            # 低速时增加油门补偿，克服静摩擦力
            low_speed_comp = max(0, ap['thr_low_speed_thresh'] - v) * ap['thr_low_speed_comp']
            # 非线性项补偿大加速度时的动力损失
            thr = ap['thr_base'] + a * ap['thr_gain'] * (1 + ap['thr_nonlinear'] * a) + low_speed_comp
            return thr, 0.0
        # 减速映射：brk = base + |a| * gain * (1 + nonlinear*|a|) + v*speed_comp
        elif a < -ap['accel_deadzone']:
            # 速度补偿项：高速时增加刹车力以克服空气阻力
            brk = ap['brk_base'] + abs(a) * ap['brk_gain'] * (1 + ap['brk_nonlinear'] * abs(a)) + v * ap['brk_speed_comp']
            return 0.0, brk

        # 极小加速度波动时的怠速油门
        return ap['idle_throttle'], 0.0

    def _execute_open_loop_stop(self, msg):
        """
        开环停车控制：避免MPC在零速附近抖动

        当参考速度接近0时，使用开环控制替代MPC，避免在低速时的控制抖动
        采用渐进式刹车策略，根据累积制动距离线性增加刹车力
        """
        sp = STOP_PARAMS

        if self.ego_speed < sp['speed_stop_thresh']:
            # 已停稳：切空挡，施加驻车刹车
            msg.throttle_percentage = 0
            msg.braking_percentage = sp['final_brake']
        else:
            # 缓慢接近停车：线性增加制动压力
            self.braking_distance += self.ego_speed * self.dt
            brk = int(sp['brake_ramp_gain'] * self.braking_distance + sp['brake_ramp_base'])
            msg.braking_percentage = min(sp['brake_ramp_max'], brk)
        msg.gear = sp['stop_gear']  # 切换到停车档位

    # ==========================================================================
    # 4. 数学模型与数据转换 (LLA -> ENU -> Arc)
    # ==========================================================================
    def lla_to_enu(self, lat, lon):
        """
        高精度 WGS84 椭球体坐标投影 (LLA -> ENU)

        将GPS经纬度坐标转换为局部东北天(ENU)坐标系
        使用WGS84椭球模型，精度优于简单的平面投影

        Args:
            lat, lon: 纬度、经度 (度)

        Returns:
            (east, north): 东向、北向坐标 (m)
        """
        # 以图书馆原点为基准
        lat0, lon0 = COORD_ORIGIN['lat0'], COORD_ORIGIN['lon0']
        A, F = 6378137.0, 1/298.257223563  # WGS84椭球参数 (固定常量)
        E2 = F * (2 - F)  # 第一偏心率平方
        D2R = pi / 180.0
        
        # 计算原点直角坐标
        phi0, lam0 = lat0 * D2R, lon0 * D2R
        N0 = A / sqrt(1 - E2 * sin(phi0)**2)
        X0 = N0 * cos(phi0) * cos(lam0)
        Y0 = N0 * cos(phi0) * sin(lam0)
        Z0 = N0 * (1 - E2) * sin(phi0)
        
        # 计算当前点直角坐标
        phi, lam = lat * D2R, lon * D2R
        N = A / sqrt(1 - E2 * sin(phi)**2)
        X = N * cos(phi) * cos(lam)
        Y = N * cos(phi) * sin(lam)
        Z = N * (1 - E2) * sin(phi)
        
        dx, dy, dz = X-X0, Y-Y0, Z-Z0
        # 旋转至东北天
        east = -sin(lam0) * dx + cos(lam0) * dy
        north = -sin(phi0)*cos(lam0)*dx - sin(phi0)*sin(lam0)*dy + cos(phi0)*dz
        return east, north

    def update_arc_length(self, lat, lon):
        """
        带GPS跳点保护的弧长投影

        将GPS坐标投影到轨迹上，计算沿轨迹的累积弧长
        包含GPS跳变检测和滤波，避免定位异常导致的控制波动

        Args:
            lat, lon: GPS纬度、经度 (度)

        Returns:
            arc_length: 沿轨迹的累积弧长 (m)
        """
        # 安全检查：轨迹未加载时返回0
        if not self.trajectory_available or len(self.trajectory_arc_map) == 0:
            return 0.0

        # 转换为ENU坐标
        e, n = self.lla_to_enu(lat, lon)

        # 最近邻搜索：找到轨迹上距离当前位置最近的点
        min_dist = float('inf')
        nearest_idx = 0
        for i in range(len(self.trajectory_x)):
            dist = sqrt((e - self.trajectory_x[i])**2 + (n - self.trajectory_y[i])**2)
            if dist < min_dist:
                min_dist, nearest_idx = dist, i

        new_arc = self.trajectory_arc_map[nearest_idx]

        # GPS跳点保护：检测并过滤异常的弧长跳变
        gp = GPS_FILTER_PARAMS
        if self.last_valid_arc > 0:
            jump = abs(new_arc - self.last_valid_arc)
            if jump > gp['jump_threshold']:
                # GPS跳变超过阈值，使用加权平均平滑
                new_arc = gp['smooth_new_weight'] * new_arc + gp['smooth_old_weight'] * self.last_valid_arc

        self.last_valid_arc = new_arc
        return new_arc

    # ==========================================================================
    # 5. 回调函数处理
    # ==========================================================================
    def cb_fusion(self, msg):
        """
        传感器融合数据回调

        接收GPS位置、速度、加速度，并进行滤波处理
        将GPS位置投影到轨迹上，计算弧长坐标
        """
        self.raw_gps_speed = msg.carspeed  # 保存原始GPS速度用于记录
        # 低通滤波：平滑GPS速度和加速度噪声
        self.ego_speed = self.speed_filter.update(msg.carspeed)
        self.ego_accel = self.accel_filter.update(msg.ax)
        self.ego_lat, self.ego_lon = msg.latitude, msg.longitude

        # 轨迹可用时，计算本车在轨迹上的弧长位置
        if self.trajectory_available:
            self.ego_arc = self.update_arc_length(self.ego_lat, self.ego_lon)

    def cb_v2x(self, msg):
        """
        V2X通信数据回调

        接收前车通过V2X发送的位置和速度信息
        仅处理来自直接前车(car_id = ego_id - 1)的数据
        """
        if self.platoon_state == 1 and msg.car_id == self.ego_id - 1:
            # 低通滤波前车速度
            self.front_speed = self.front_speed_f.update(msg.speed)
            # 计算前车在轨迹上的弧长位置
            if self.trajectory_available:
                self.front_arc = self.update_arc_length(msg.latitude, msg.longitude)
            self.front_update_time = time.time()

    def cb_path(self, msg):
        """
        全局路径规划回调

        接收全局规划器发布的轨迹点序列
        构建轨迹的累积弧长映射表，用于弧长投影
        """
        if len(msg.latitude) > 0 and not self.trajectory_available:
            # 提取参考速度
            self.ref_speed = msg.speed[0]
            self.trajectory_x, self.trajectory_y = [], []
            self.trajectory_arc_map = [0.0]

            # 处理第一个轨迹点
            ex, nx = self.lla_to_enu(msg.latitude[0], msg.longitude[0])
            self.trajectory_x.append(ex)
            self.trajectory_y.append(nx)

            # 逐点计算累积弧长
            dist = 0.0
            for i in range(1, len(msg.latitude)):
                e, n = self.lla_to_enu(msg.latitude[i], msg.longitude[i])
                # 累加两点之间的欧氏距离
                dist += sqrt((e-self.trajectory_x[-1])**2 + (n-self.trajectory_y[-1])**2)
                self.trajectory_x.append(e)
                self.trajectory_y.append(n)
                self.trajectory_arc_map.append(dist)

            self.trajectory_available = True

    def cb_hmi(self, msg):
        """
        编队HMI状态回调

        接收人机交互界面发送的编队状态和本车ID
        """
        self.platoon_state = msg.platoon_state  # 0=单车, 1=编队
        self.ego_id = msg.platoon_id            # 1=领航车, 2/3=跟随车

    # ==========================================================================
    # 6. 数据持久化 (详尽 CSV 记录器)
    # ==========================================================================
    def _init_csv_logging(self):
        """
        初始化CSV日志记录器

        创建CSV文件，记录控制过程中的所有关键数据
        用于后续的性能分析、调试和论文绘图
        """
        path = os.path.expanduser('/home/nvidia/AutoDrive/src/pid/data')
        os.makedirs(path, exist_ok=True)
        ts = datetime.datetime.now().strftime("%m%d_%H%M")
        self.csv_file = f"{path}/lmq_mpc_pf_car{self.ego_id}_{ts}.csv"

        with open(self.csv_file, 'w') as f:
            writer = csv.writer(f)
            # CSV列名：使用gap表示净间距，distance表示中心距
            writer.writerow([
                't_s', 'id', 'mode', 'ego_arc', 'ego_v', 'target_a', 'thr', 'brk',
                'lat', 'lon',
                'gap_front',           # 净间距 (车尾到车头)
                'distance_front',      # 中心距 (弧长差)
                'ref_v', 'raw_v', 'mpc_status'
            ])
        self.start_time = time.time()

    def _write_log(self, a, t, b):
        """
        写入一行日志数据

        记录当前时刻的控制状态、车辆状态、间距信息等
        """
        L = self.L

        if self.ego_id == 1:
            # 领航车：无前车，间距记录为0
            gap_front = 0.0
            distance_front = 0.0
        else:
            # 跟随车：计算与前车的间距
            distance_front = self.front_arc - self.ego_arc  # 中心距 (弧长差)
            gap_front = distance_front - L                  # 净间距 (扣除车长)

        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 2),  # 相对时间 (s)
                self.ego_id, self.platoon_state,
                round(self.ego_arc, 2), round(self.ego_speed, 2),
                round(a, 3), int(t), int(b),
                round(self.ego_lat, 7), round(self.ego_lon, 7),
                round(gap_front, 2),               # 净间距 (m)
                round(distance_front, 2),          # 中心距 (m)
                round(self.ref_speed, 2),          # 参考速度 (m/s)
                round(self.raw_gps_speed, 2),      # 原始GPS速度 (m/s)
                self.mpc_status                    # MPC状态: 0=未求解, 1=成功, 2=失败, 3=紧急
            ])

def main(args=None):
    """
    PF拓扑编队控制节点主函数

    启动流程:
    1. 初始化ROS 2
    2. 创建PF-MPC控制器节点
    3. 进入事件循环，响应传感器和通信回调
    4. 异常捕获和资源清理
    """
    # 1. 初始化 ROS 2
    rclpy.init(args=args)

    try:
        # 2. 实例化PF拓扑控制器
        rosNode = PFPlatoonMPC()
        print(">>> [SUCCESS] PF编队控制节点已成功初始化")
        print(">>> PF代价函数 (Predecessor Following):")
        print("    J = W_ref_v*(v-v_ref)² + W_a*a² + W_j*jerk²")
        print("      + W_fg*(gap_front - gap_des)²")
        print(f">>> 参数:")
        print(f"    - 车长L={VEHICLE_PARAMS['vehicle_length']}m")
        print(f"    - 期望净间距gap_des={PLATOON_PARAMS['desired_gap']}m")
        print(f"    - 最小安全净间距min_gap={PLATOON_PARAMS['min_gap']}m")
        print(">>> 正在等待数据...")

        # 3. 进入自旋循环，等待回调执行
        rclpy.spin(rosNode)

    except KeyboardInterrupt:
        print("\n[INFO] 用户按下 Ctrl+C，正在停止节点...")
    except Exception as e:
        print(f"\n[FATAL] 节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 4. 彻底清理资源
        print(">>> 正在关闭 ROS 2 资源...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()