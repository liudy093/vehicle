#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Description: 集成MPC纵向控制与PID横向控制
#
# 功能说明:
#   - 纵向控制: PLF拓扑MPC编队控制 (来自 lmq_auto_mpc_plf.py)
#   - 横向控制: PID转向控制 (来自 pid.py)
#   - 同时输出: angle(横向), throttle_percentage, braking_percentage(纵向)
#横纵向控制集成，运行本代码启动start_tracker相关节点
import sys
import os
import rclpy
import time
import yaml
import csv
import datetime
import numpy as np
import cvxpy as cp
import subprocess
import signal
import atexit
from math import *
from collections import deque
from rclpy.node import Node
from easydict import EasyDict
from car_interfaces.msg import *

PI = 3.1415926535

# ==============================================================================
# 自动启动其他ROS 2节点
# ==============================================================================
class NodeLauncher:
    """自动启动和管理其他ROS 2节点"""
    def __init__(self):
        self.processes = []
        self.nodes_to_launch = [
            'car_decision',
            'local_path_planning',
            'car_control',
        ]

    def launch_nodes(self):
        """启动所有必要的节点"""
        print("=" * 60)
        print("正在启动其他必要的ROS 2节点...")
        print("=" * 60)

        for node_name in self.nodes_to_launch:
            try:
                # 使用ros2 run命令启动节点
                cmd = ['ros2', 'run', node_name, node_name]
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # 创建新的进程组
                )
                self.processes.append((node_name, process))
                print(f"✓ 已启动节点: {node_name} (PID: {process.pid})")
                time.sleep(0.5)  # 给节点一些启动时间
            except Exception as e:
                print(f"✗ 启动节点 {node_name} 失败: {e}")

        print("=" * 60)
        print(f"共启动 {len(self.processes)} 个节点")
        print("=" * 60)
        print()

    def shutdown_nodes(self):
        """关闭所有启动的节点"""
        print("\n" + "=" * 60)
        print("正在关闭所有启动的节点...")
        print("=" * 60)

        for node_name, process in self.processes:
            try:
                # 发送SIGTERM信号到进程组
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                print(f"✓ 已关闭节点: {node_name}")
            except subprocess.TimeoutExpired:
                # 如果5秒后还没关闭，强制杀死
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                print(f"✓ 已强制关闭节点: {node_name}")
            except Exception as e:
                print(f"✗ 关闭节点 {node_name} 时出错: {e}")

        print("=" * 60)
        print("所有节点已关闭")
        print("=" * 60)

# 全局节点启动器实例
node_launcher = None

# 添加 sys.path 配置
ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

# ==============================================================================
# 可调参数配置区
# ==============================================================================

# -------------------- 1. 横向PID控制参数 --------------------
LATERAL_PID_PARAMS = {
    'speed_ranges': [3.5, 5.5, 7.5],        # 速度分段阈值 (m/s)
    'pid_ephi': [1.4, 1.0, 0.5, 0.5],       # 航向角误差增益（对应4个速度段）
    'pid_ed': [0.115, 0.09, 0.05, 0.05],    # 横向距离误差增益（对应4个速度段）
    'lookahead_distances': [5, 10, 15, 20], # 前视距离 (点数)，对应4个速度段
    'control_smooth_factor': 0.3,           # 控制平滑滤波系数（0~1）
    'angle_max': 500.0,                     # 最大转向角限幅
    'angle_min': -500.0,                    # 最小转向角限幅
    'angle_scale': 20.0,                    # 转向角缩放系数
}

# -------------------- 2. MPC 控制器参数 --------------------
# 领航车MPC参数（仅用于速度跟踪）
LEADER_MPC_PARAMS = {
    'dt': 0.1,                  # 控制周期 (s)
    'N': 10,                    # 预测时域步数
    'W_ref_velocity': 8.0,     # 参考速度跟踪权重（提高以增强速度跟踪）
    'W_accel': 20.0,             # 加速度平滑权重（提高以限制加速度幅值）
    'W_jerk': 2.5,              # 加加速度平滑权重（提高以使加速度变化更平缓）
    'a_min': -4.0,              # 最小加速度 (m/s²)
    'a_max': 2.0,               # 最大加速度 (m/s²)，降低以避免过快接近导致超调
    'v_min': 0.0,               # 最小速度 (m/s)
    'v_max': 10.0,              # 最大速度 (m/s)
    'fallback_kp': 0.5,         # MPC求解失败时的比例增益
}

# 跟随车MPC参数（PLF拓扑编队控制）
FOLLOWER_MPC_PARAMS = {
    'dt': 0.1,                  # 控制周期 (s)
    'N': 10,                    # 预测时域步数
    'W_ref_velocity': 5.0,      # 参考速度跟踪权重
    'W_accel': 15.0,             # 加速度平滑权重（更高以保证舒适性）
    'W_jerk': 2.0,              # 加加速度平滑权重（更高以保证舒适性）
    'W_front_gap': 25.0,        # 前车间距跟踪权重（关键安全参数）
    'W_leader_gap': 15.0,       # 领航车间距跟踪权重
    'W_leader_velocity': 10.0,  # 领航车速度跟踪权重
    'a_min': -4.0,              # 最小加速度 (m/s²)
    'a_max': 2.5,               # 最大加速度 (m/s²)
    'v_min': 0.0,               # 最小速度 (m/s)
    'v_max': 10.0,              # 最大速度 (m/s)
    'fallback_kp': 0.5,         # MPC求解失败时的比例增益
}

# -------------------- 2. 编队几何参数 --------------------
PLATOON_PARAMS = {
    'desired_gap': 5.0,          # 期望净间距 (m)，车辆间的目标距离（不含车长）
    'min_gap': 1.3,              # 最小安全净间距 (m)，MPC约束的硬限制
    'desired_speed': 3.0,        # 期望巡航速度 (m/s)
}

VEHICLE_PARAMS = {
    'vehicle_length': 4.4,       # 车辆长度 (m)，用于计算净间距
}

VEHICLE_STATE = {
    'ego_id': 2,                 # 本车编号（1=领航车，2,3,...=跟随车）
    'platoon_state': 1,           # 编队状态（0=单车模式，1=编队模式）
}

# -------------------- 3. PLF拓扑参数 --------------------
PLF_PARAMS = {
    'alpha': 0.7,                # 前车间距权重系数（0~1），α越大越依赖前车
    'beta': 0.3,                 # 参考速度权重系数（0~1），β越大越依赖参考速度
    'use_leader_gap': True,      # 是否使用领航车间距信息
    'use_leader_velocity': True, # 是否使用领航车速度信息
    'headway_time': 0.8,         # 时距 (s)，用于CTH策略
    'use_cth_gap': False,        # 是否使用恒定时距策略（False=恒定间距）
}

# -------------------- 4. 低通滤波器参数 --------------------
FILTER_PARAMS = {
    'alpha_speed': 0.3,          # 本车速度滤波系数（0~1），越大响应越快
    'alpha_accel': 0.5,          # 本车加速度滤波系数（0~1）
    'alpha_front_speed': 0.6,    # 前车速度滤波系数（0~1）
    'alpha_leader_speed': 0.6,   # 领航车速度滤波系数（0~1）
}

# -------------------- 5. 执行器映射参数 --------------------
ACTUATOR_PARAMS = {
    # 油门映射参数
    'thr_base': 30.0,            # 油门基础值 (%)
    'thr_gain': 12.0,            # 油门增益系数 (%/(m/s²))
    'thr_nonlinear': 0.1,        # 油门非线性补偿系数
    'thr_low_speed_comp': 2.0,   # 低速油门补偿增益 (%/(m/s))
    'thr_low_speed_thresh': 3.0, # 低速补偿阈值 (m/s)

    # 刹车映射参数
    'brk_base': 10.0,            # 刹车基础值 (%)
    'brk_gain': 15.0,            # 刹车增益系数 (%/(m/s²))
    'brk_nonlinear': 0.15,       # 刹车非线性补偿系数
    'brk_speed_comp': 1.8,       # 刹车速度补偿系数 (%/(m/s))

    # 死区与限幅参数
    'speed_deadzone': 0.1,       # 速度死区 (m/s)，仅用于驻车判定
    'park_brake': 20.0,          # 驻车刹车 (%)
    'thr_max': 80,               # 最大油门限幅 (%)
    'brk_max': 100,              # 最大刹车限幅 (%)

    # 加速度死区控制开关
    'use_accel_deadzone': True,  # False=完全由MPC控制，True=使用死区
    'accel_deadzone': 0.1,       # 加速度死区 (m/s²)，仅当use_accel_deadzone=True时生效
    'idle_throttle_in_deadzone': 22.0,  # 死区内的怠速油门 (%)
}

# -------------------- 6. 停车控制参数 --------------------
STOP_PARAMS = {
    'ref_speed_thresh': 0.05,      # 参考速度停车阈值 (m/s)
    'speed_stop_thresh': 0.1,      # 实际速度停车阈值 (m/s)
    'final_brake': 30,             # 最终驻车刹车力 (%)
    'brake_ramp_gain': 5.0,        # 刹车渐进增益 (%/m)
    'brake_ramp_base': 20,         # 刹车渐进基础值 (%)
    'brake_ramp_max': 125,         # 刹车渐进最大值 (%)
    'stop_gear': 2,                # 停车档位
    'drive_gear': 3,               # 行驶档位
}

# -------------------- 7. GPS跳点保护参数 --------------------
GPS_FILTER_PARAMS = {
    'jump_threshold': 10.0,        # GPS跳点检测阈值 (m)
    'smooth_new_weight': 0.3,      # 平滑滤波新值权重
    'smooth_old_weight': 0.7,      # 平滑滤波旧值权重
}

# -------------------- 8. 坐标系原点 --------------------
COORD_ORIGIN = {
    'use_auto_origin': False,      # 是否自动使用轨迹起点作为原点
    'lat0': 34.5855425,            # 手动设置的原点纬度 (°)
    'lon0': 113.6855351,           # 手动设置的原点经度 (°)
}

# -------------------- 9. 安全兜底参数 --------------------
SAFETY_PARAMS = {
    'no_trajectory_accel': -1.0    # 无轨迹时的安全减速度 (m/s²)
}

# -------------------- 10. 安全监控参数 --------------------
SAFETY_MONITOR = {
    'mpc_fail_count_limit': 10,    # MPC连续失败次数限制
    'mpc_fail_decel': -2.0,        # MPC失败时的安全减速度 (m/s²)
    'critical_gap': 1.0,           # 临界危险间距 (m)，触发紧急制动
    'warning_gap': 1.2,            # 警告间距 (m)，触发强制减速
    'emergency_brake': 80,         # 紧急制动力 (%)
    'warning_decel': -3.0,         # 警告减速度 (m/s²)
}

# ==============================================================================
# 工具类：一阶低通滤波器
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
# 主节点：集成MPC纵向控制与PID横向控制
# ==============================================================================
class IntegratedControl(Node):
    def __init__(self, name='pid'):
        super().__init__(name)
        print("DEBUG: 集成控制脚本已经进入初始化流程")
        self.get_logger().info("Initializing Integrated Control Node (MPC Longitudinal + PID Lateral)...")

        # -------- A. 加载可调参数 --------
        # 根据车辆ID选择MPC参数（领航车或跟随车）
        self.ego_id = VEHICLE_STATE['ego_id']
        # 保存初始配置的ID，用于运行时恢复
        self.initial_ego_id = self.ego_id

        if self.ego_id == 1:
            # 领航车使用LEADER_MPC_PARAMS
            mpc_params = LEADER_MPC_PARAMS
            self.get_logger().info(">>> 加载领航车MPC参数")
        else:
            # 跟随车使用FOLLOWER_MPC_PARAMS
            mpc_params = FOLLOWER_MPC_PARAMS
            self.get_logger().info(f">>> 加载跟随车MPC参数 (车辆ID={self.ego_id})")

        # 加载MPC基础参数
        self.dt = mpc_params['dt']
        self.N = mpc_params['N']
        self.W_ref_velocity = mpc_params['W_ref_velocity']
        self.W_accel = mpc_params['W_accel']
        self.W_jerk = mpc_params['W_jerk']
        self.a_min = mpc_params['a_min']
        self.a_max = mpc_params['a_max']
        self.v_min = mpc_params['v_min']
        self.v_max = mpc_params['v_max']
        self.fallback_kp = mpc_params['fallback_kp']

        # 输出加载的参数以确认
        self.get_logger().info(f">>> MPC参数已加载: N={self.N}, W_ref_velocity={self.W_ref_velocity}, W_accel={self.W_accel}, W_jerk={self.W_jerk}")
        self.get_logger().info(f">>> 加速度约束: a_min={self.a_min}, a_max={self.a_max}")

        # 跟随车额外参数（仅跟随车使用）
        if self.ego_id > 1:
            self.W_front_gap = mpc_params['W_front_gap']
            self.W_leader_gap = mpc_params['W_leader_gap']
            self.W_leader_velocity = mpc_params['W_leader_velocity']
        else:
            # 领航车不需要这些参数，但为了代码兼容性设为0
            self.W_front_gap = 0.0
            self.W_leader_gap = 0.0
            self.W_leader_velocity = 0.0

        self.desired_gap = PLATOON_PARAMS['desired_gap']
        self.min_gap = PLATOON_PARAMS['min_gap']
        self.desired_speed = PLATOON_PARAMS['desired_speed']

        self.alpha = PLF_PARAMS['alpha']
        self.beta = PLF_PARAMS['beta']
        self.use_leader_gap = PLF_PARAMS['use_leader_gap']
        self.use_leader_velocity = PLF_PARAMS['use_leader_velocity']
        self.headway_time = PLF_PARAMS['headway_time']
        self.use_cth_gap = PLF_PARAMS['use_cth_gap']

        self.L = VEHICLE_PARAMS['vehicle_length']
        self.platoon_state = VEHICLE_STATE['platoon_state']
        self.lateral_params = LATERAL_PID_PARAMS

        # -------- B. 传感器滤波模块 --------
        self.speed_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_speed'], name="Speed")
        self.accel_filter = LowPassFilter(alpha=FILTER_PARAMS['alpha_accel'], name="Accel")
        self.front_speed_f = LowPassFilter(alpha=FILTER_PARAMS['alpha_front_speed'], name="FrontSpeed")
        self.leader_speed_f = LowPassFilter(alpha=FILTER_PARAMS['alpha_leader_speed'], name="LeaderSpeed")

        # -------- C. 运行状态变量 --------
        self.use_auto_origin = COORD_ORIGIN['use_auto_origin']
        if self.use_auto_origin:
            self.origin_lat = COORD_ORIGIN['lat0']
            self.origin_lon = COORD_ORIGIN['lon0']
            self.origin_initialized = False
        else:
            self.origin_lat = COORD_ORIGIN['lat0']
            self.origin_lon = COORD_ORIGIN['lon0']
            self.origin_initialized = True

        self.ego_speed = 0.0
        self.ego_accel = 0.0
        self.ego_lat, self.ego_lon = 0.0, 0.0
        self.ego_yaw = 0.0
        self.ego_pitch = 0.0
        self.ego_arc = 0.0
        self.ref_speed = self.desired_speed

        self.front_speed = 0.0
        self.front_arc = 0.0
        self.front_update_time = 0.0

        self.leader_speed = 0.0
        self.leader_arc = 0.0
        self.leader_accel = 0.0
        self.leader_update_time = 0.0
        self.leader_data_valid = False

        self.braking_distance = 0.0
        self.raw_gps_speed = 0.0

        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_arc_map = []
        self.trajectory_latitude = []
        self.trajectory_longitude = []
        self.trajectory_angle = []
        self.trajectory_available = False
        self.last_valid_arc = 0.0

        # 横向控制状态变量
        self.ref_latitude = 0.0
        self.ref_longitude = 0.0
        self.ref_yaw = 0.0
        self.control_value_last = 0.0
        self.GPS_state = True

        # 安全监控状态变量
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
        """设置ROS 2通信"""
        self.pub_control = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timer = self.create_timer(self.dt, self.on_timer_callback)
        self.sub_fusion = self.create_subscription(FusionInterface, 'fusion_data', self.cb_fusion, 10)
        self.sub_path = self.create_subscription(LocalPathPlanningInterface, 'local_path_planning_data', self.cb_path, 10)
        self.sub_hmi = self.create_subscription(HmiPlatoonInterface, 'platoon_state_data', self.cb_hmi, 10)
        self.sub_v2x = self.create_subscription(VtxInterface, 'v2x_to_planner_data', self.cb_v2x, 10)

    # ==========================================================================
    # 核心控制逻辑
    # ==========================================================================
    def on_timer_callback(self):
        msg = PidInterface()
        msg.timestamp = time.time()

        # 动态更新横向参考点（根据当前速度选择前视距离）
        self._update_lateral_reference()

        # 计算横向控制角度
        lateral_angle = self.calculate_lateral_control()
        msg.angle = lateral_angle

        # 停车控制
        if self.ref_speed < STOP_PARAMS['ref_speed_thresh']:
            self._execute_open_loop_stop(msg)
            self.pub_control.publish(msg)
            return

        # 计算纵向控制加速度
        target_acceleration = 0.0
        if not self.trajectory_available:
            target_acceleration = SAFETY_PARAMS['no_trajectory_accel']
            self.mpc_status = 0
        else:
            target_acceleration = self.solve_mpc_plf()

        # 安全覆盖
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

        # 无轨迹时的安全处理
        if not self.trajectory_available:
            msg.throttle_percentage = 0
            msg.braking_percentage = 30
            msg.angle = 0.0
            self.control_value_last = 0.0

        self.pub_control.publish(msg)
        self._write_log(target_acceleration, msg.throttle_percentage, msg.braking_percentage, msg.angle)

    def calculate_lateral_control(self):
        """计算横向PID控制角度 (来自 pid.py 第77-117行)"""
        if not self.trajectory_available or not self.GPS_state:
            return 0.0

        # 坐标转换
        nowPosX, nowPosY, _ = self.conversion_of_coordinates(self.ego_lat, self.ego_lon, 0)
        refPosX, refPosY, _ = self.conversion_of_coordinates(self.ref_latitude, self.ref_longitude, 0)

        # 计算航向角误差
        errorYaw = -self.ego_yaw + self.ref_yaw
        if errorYaw > PI:
            errorYaw = errorYaw - 2 * PI
        if errorYaw < -PI:
            errorYaw = errorYaw + 2 * PI

        # 计算横向距离误差
        errorDistance = ((-nowPosY + refPosY) * cos(self.ref_yaw) - (-nowPosX + refPosX) * sin(self.ref_yaw))

        # 根据速度选择PID参数
        speed_ranges = self.lateral_params['speed_ranges']
        pid_ephi_list = self.lateral_params['pid_ephi']
        pid_ed_list = self.lateral_params['pid_ed']

        if self.ego_speed < speed_ranges[0]:
            pidEphi, pidEd = pid_ephi_list[0], pid_ed_list[0]
        elif self.ego_speed < speed_ranges[1]:
            pidEphi, pidEd = pid_ephi_list[1], pid_ed_list[1]
        elif self.ego_speed < speed_ranges[2]:
            pidEphi, pidEd = pid_ephi_list[2], pid_ed_list[2]
        else:
            pidEphi, pidEd = pid_ephi_list[3], pid_ed_list[3]

        # 计算控制值
        controlValue = (pidEphi * errorYaw + pidEd * errorDistance) * 180 / PI
        controlValue = controlValue * self.lateral_params['angle_scale']

        # 平滑滤波
        control_smooth = self.lateral_params['control_smooth_factor']
        controlValue = control_smooth * controlValue + (1 - control_smooth) * self.control_value_last
        self.control_value_last = controlValue

        # 限幅
        controlValue = np.clip(controlValue, self.lateral_params['angle_min'], self.lateral_params['angle_max'])
        return controlValue

    def solve_mpc_plf(self):
        """PLF拓扑MPC求解器 (来自 lmq_auto_mpc_plf.py)"""
        sm = SAFETY_MONITOR
        L = self.L

        # 计算间距
        distance_front = self.front_arc - self.ego_arc
        distance_leader = self.leader_arc - self.ego_arc
        gap_front = distance_front - L
        num_vehicles_between = self.ego_id - 2
        gap_leader = distance_leader - (num_vehicles_between + 1) * L
        gap_des = self.desired_gap
        gap_leader_des = (self.ego_id - 1) * self.desired_gap

        # 诊断信息：检查V2X数据是否有效
        if self.ego_id > 1 and self.platoon_state == 1:
            if self.front_arc == 0.0 and self.front_speed == 0.0:
                self.get_logger().warn(f"[诊断] 前车数据缺失! front_arc={self.front_arc:.2f}, front_speed={self.front_speed:.2f}")
                self.get_logger().warn(f"[诊断] 请检查: 1)前车(ID={self.ego_id-1})是否运行 2)V2X通信是否正常 3)轨迹是否已加载")
            if self.leader_arc == 0.0 and self.leader_speed == 0.0:
                self.get_logger().warn(f"[诊断] 领航车数据缺失! leader_arc={self.leader_arc:.2f}, leader_speed={self.leader_speed:.2f}")
                self.get_logger().warn(f"[诊断] 请检查: 1)领航车(ID=1)是否运行 2)V2X通信是否正常 3)轨迹是否已加载")

        self.get_logger().info(
            f"[PLF] id={self.ego_id} | 前车: gap={gap_front:.1f}m (目标{gap_des:.0f}m) | "
            f"领航: gap={gap_leader:.1f}m (目标{gap_leader_des:.0f}m) | v_leader={self.leader_speed:.2f}m/s"
        )

        # 1号车（领航车）：仅速度跟踪
        if self.ego_id == 1:
            return self._solve_leader_mpc()

        # 安全检查
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

        # PLF MPC求解
        u = cp.Variable(self.N)
        v = cp.Variable(self.N + 1)
        s = cp.Variable(self.N + 1)

        constraints = [
            v[0] == self.ego_speed, s[0] == self.ego_arc,
            u >= self.a_min, u <= self.a_max,
            v >= self.v_min, v <= self.v_max
        ]

        cost = 0
        alpha = self.alpha
        beta = self.beta

        for k in range(self.N):
            constraints += [
                v[k+1] == v[k] + u[k] * self.dt,
                s[k+1] == s[k] + v[k] * self.dt
            ]

            cost += self.W_accel * cp.square(u[k])
            if k > 0:
                cost += self.W_jerk * cp.square(u[k] - u[k-1])

            # 修复：改为检查前车数据是否有效，而不是简单比较弧长
            # 原条件 self.front_arc > self.ego_arc 在初始时刻(都为0)会失败
            front_data_valid = (self.front_arc > 0.01 or self.front_speed > 0.01)
            if self.platoon_state == 1 and front_data_valid:
                pred_front_arc = self.front_arc + self.front_speed * (k+1) * self.dt
                pred_gap_front = pred_front_arc - s[k+1] - L

                if self.use_cth_gap:
                    pred_gap_des = self.desired_gap + self.headway_time * v[k+1]
                else:
                    pred_gap_des = self.desired_gap

                constraints += [pred_gap_front >= self.min_gap]
                cost += alpha * self.W_front_gap * cp.square(pred_gap_front - pred_gap_des)

                if self.use_leader_gap and self.leader_data_valid and self.leader_arc > 0:
                    pred_leader_arc = self.leader_arc + self.leader_speed * (k+1) * self.dt
                    n_between = self.ego_id - 2
                    pred_gap_leader = pred_leader_arc - s[k+1] - (n_between + 1) * L
                    pred_gap_leader_des = (self.ego_id - 1) * self.desired_gap
                    cost += (1 - alpha) * self.W_leader_gap * cp.square(pred_gap_leader - pred_gap_leader_des)

                if self.use_leader_velocity and self.leader_data_valid and self.leader_speed > 0:
                    pred_leader_v = self.leader_speed + self.leader_accel * (k+1) * self.dt
                    pred_leader_v = max(0.0, min(pred_leader_v, self.v_max))
                    cost += (1 - beta) * self.W_leader_velocity * cp.square(v[k+1] - pred_leader_v)
                    cost += beta * self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)
                else:
                    # 间距控制激活但无领航车速度时，跟踪参考速度
                    cost += self.W_ref_velocity * cp.square(v[k+1] - self.ref_speed)
            else:
                # 修复：间距控制未激活时的兜底逻辑，确保始终有速度跟踪目标
                # 这避免了MPC只优化加速度而没有速度目标的问题
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

        # MPC求解失败处理
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
        """
        执行器命令计算（加速度死区开关）

        功能说明：
        1. 根据目标加速度计算油门/刹车百分比
        2. 支持加速度死区开关控制
        3. 包含低速补偿和非线性补偿
        """
        v = self.ego_speed
        ap = ACTUATOR_PARAMS

        # 基础执行器映射
        if ap['use_accel_deadzone']:
            # 使用加速度死区的传统逻辑
            # 车辆静止时，只有在明确需要停车时才刹车
            if abs(v) < ap['speed_deadzone'] and abs(a) < ap['accel_deadzone'] and self.ref_speed < 0.05:
                # 仅在参考速度也为0时才驻车刹车
                base_thr, base_brk = 0.0, ap['park_brake']
            elif a > ap['accel_deadzone']:
                low_speed_comp = max(0, ap['thr_low_speed_thresh'] - v) * ap['thr_low_speed_comp']
                base_thr = ap['thr_base'] + a * ap['thr_gain'] * (1 + ap['thr_nonlinear'] * a) + low_speed_comp
                base_brk = 0.0
            elif a < -ap['accel_deadzone']:
                base_brk = ap['brk_base'] + abs(a) * ap['brk_gain'] * (1 + ap['brk_nonlinear'] * abs(a)) + v * ap['brk_speed_comp']
                base_thr = 0.0
            else:
                # 加速度在死区内（-0.1 ~ 0.1 m/s²），使用怠速油门
                base_thr, base_brk = ap['idle_throttle_in_deadzone'], 0.0
        else:
            # 完全由MPC控制（无加速度死区）
            if abs(v) < ap['speed_deadzone'] and abs(a) < 0.05:
                base_thr, base_brk = 0.0, ap['park_brake']
            elif a > 0:
                low_speed_comp = max(0, ap['thr_low_speed_thresh'] - v) * ap['thr_low_speed_comp']
                base_thr = ap['thr_base'] + a * ap['thr_gain'] * (1 + ap['thr_nonlinear'] * a) + low_speed_comp
                base_brk = 0.0
            else:
                base_brk = ap['brk_base'] + abs(a) * ap['brk_gain'] * (1 + ap['brk_nonlinear'] * abs(a)) + v * ap['brk_speed_comp']
                base_thr = 0.0

        return base_thr, base_brk

    def _execute_open_loop_stop(self, msg):
        """开环停车控制"""
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
    # 坐标转换
    # ==========================================================================
    def conversion_of_coordinates(self, conversionLatitude, conversionLongitude, conversionAltitude):
        """坐标转换函数：经纬高LLA 转 东北天ENU (来自 pid.py)"""
        wgs84_a = 6378137.0
        wgs84_f = 1/298.257223565
        wgs84_e2 = wgs84_f * (2-wgs84_f)
        D2R = PI / 180.0

        # 使用实例变量的原点
        Latitude0 = self.origin_lat
        Longitude0 = self.origin_lon

        lat0 = Latitude0 * D2R
        lon0 = Longitude0 * D2R
        alt0 = 0
        N0 = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat0)*sin(lat0))
        x0 = (N0 + alt0) * cos(lat0) * cos(lon0)
        y0 = (N0 + alt0) * cos(lat0) * sin(lon0)
        z0 = (N0*(1-wgs84_e2) + alt0) * sin(lat0)

        lat = conversionLatitude * D2R
        lon = conversionLongitude * D2R
        alt = conversionAltitude
        N = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat)*sin(lat))
        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N*(1-wgs84_e2) + alt) * sin(lat)

        dx, dy, dz = x-x0, y-y0, z-z0

        s11 = -sin(lon0)
        s12 = cos(lon0)
        s13 = 0
        s21 = -sin(lat0) * cos(lon0)
        s22 = -sin(lat0) * sin(lon0)
        s23 = cos(lat0)
        s31 = cos(lat0) * cos(lon0)
        s32 = cos(lat0) * sin(lon0)
        s33 = sin(lat0)

        posX = s11 * dx + s12 * dy + s13 * dz
        posY = s21 * dx + s22 * dy + s23 * dz
        posZ = s31 * dx + s32 * dy + s33 * dz

        return posX, posY, posZ

    def lla_to_enu(self, lat, lon):
        """GPS (WGS84) → ENU 局部坐标转换"""
        lat0, lon0 = self.origin_lat, self.origin_lon
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
        """更新弧长位置"""
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
    # 回调函数
    # ==========================================================================
    def cb_fusion(self, msg):
        """融合数据回调"""
        self.raw_gps_speed = msg.carspeed
        self.ego_speed = self.speed_filter.update(msg.carspeed)
        self.ego_accel = self.accel_filter.update(msg.ax)
        self.ego_lat, self.ego_lon = msg.latitude, msg.longitude
        self.ego_yaw = msg.yaw / 180 * PI
        self.ego_pitch = msg.pitch

        # GPS状态检查
        if abs(self.ego_lat) < 1 and abs(self.ego_lon) < 1:
            self.GPS_state = False
        else:
            self.GPS_state = True

        if self.trajectory_available:
            self.ego_arc = self.update_arc_length(self.ego_lat, self.ego_lon)

    def cb_v2x(self, msg):
        """V2X数据回调"""
        if self.platoon_state != 1:
            self.get_logger().debug(f"[V2X] 忽略V2X消息 (platoon_state={self.platoon_state})")
            return

        # 修复：使用platoon_id而不是car_id，因为V2X节点的car_id可能未正确设置
        self.get_logger().info(f"[V2X] 收到车辆{msg.platoon_id}的V2X消息: speed={msg.speed:.2f}m/s, lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

        # 处理领航车信息 (platoon_id=1)
        if msg.platoon_id == 1 and self.ego_id > 1:
            self.leader_speed = self.leader_speed_f.update(msg.speed)
            self.leader_accel = msg.acceleration
            if self.trajectory_available:
                self.leader_arc = self.update_arc_length(msg.latitude, msg.longitude)
                self.get_logger().info(f"[V2X] 领航车弧长更新: {self.leader_arc:.2f}m")
            else:
                self.get_logger().warn("[V2X] 轨迹未加载，无法计算领航车弧长")
            self.leader_update_time = time.time()
            self.leader_data_valid = True
            self.get_logger().info(f"[V2X] 领航车数据已更新: speed={self.leader_speed:.2f}m/s, arc={self.leader_arc:.2f}m")

        # 处理前车信息 (platoon_id = ego_id - 1)
        if msg.platoon_id == self.ego_id - 1:
            self.front_speed = self.front_speed_f.update(msg.speed)
            if self.trajectory_available:
                self.front_arc = self.update_arc_length(msg.latitude, msg.longitude)
                self.get_logger().info(f"[V2X] 前车弧长更新: {self.front_arc:.2f}m")
            else:
                self.get_logger().warn("[V2X] 轨迹未加载，无法计算前车弧长")
            self.front_update_time = time.time()
            self.get_logger().info(f"[V2X] 前车数据已更新: speed={self.front_speed:.2f}m/s, arc={self.front_arc:.2f}m")

    def cb_path(self, msg):
        """轨迹数据回调"""
        if len(msg.latitude) > 0:
            # 持续更新轨迹数据用于横向控制（每次都更新）
            self.trajectory_latitude = list(msg.latitude)
            self.trajectory_longitude = list(msg.longitude)
            self.trajectory_angle = list(msg.angle)

            # 仅首次接收时初始化纵向MPC所需的弧长映射
            if not self.trajectory_available:
                self.get_logger().info(f"[轨迹] 首次接收轨迹数据，共{len(msg.latitude)}个点")

                # 根据配置模式设置原点
                if self.use_auto_origin and not self.origin_initialized:
                    self.origin_lat = msg.latitude[0]
                    self.origin_lon = msg.longitude[0]
                    self.origin_initialized = True
                    self.get_logger().info(f"[ENU 坐标系] 原点已自动设置为轨迹起点: lat={self.origin_lat:.7f}, lon={self.origin_lon:.7f}")

                # 初始化轨迹存储（用于纵向MPC弧长计算）
                self.trajectory_x, self.trajectory_y = [], []
                self.trajectory_arc_map = [0.0]

                ex, nx = self.lla_to_enu(msg.latitude[0], msg.longitude[0])
                self.trajectory_x.append(ex)
                self.trajectory_y.append(nx)

                # 构建轨迹弧长映射
                dist = 0.0
                for i in range(1, len(msg.latitude)):
                    e, n = self.lla_to_enu(msg.latitude[i], msg.longitude[i])
                    dist += sqrt((e-self.trajectory_x[-1])**2 + (n-self.trajectory_y[-1])**2)
                    self.trajectory_x.append(e)
                    self.trajectory_y.append(n)
                    self.trajectory_arc_map.append(dist)

                self.trajectory_available = True
                self.get_logger().info(f"[轨迹] 加载完成: {len(msg.latitude)}点, 总长度{dist:.1f}m")

            # 更新横向控制的参考点
            self._update_lateral_reference()

    def _update_lateral_reference(self):
        """更新横向控制的参考点 (根据速度选择前视距离)"""
        if not self.trajectory_available or len(self.trajectory_latitude) == 0:
            return

        # 根据速度选择前视距离
        speed_ranges = self.lateral_params['speed_ranges']
        lookahead_distances = self.lateral_params['lookahead_distances']

        if self.ego_speed < speed_ranges[0]:
            advanceDistance = lookahead_distances[0]
        elif self.ego_speed < speed_ranges[1]:
            advanceDistance = lookahead_distances[1]
        elif self.ego_speed < speed_ranges[2]:
            advanceDistance = lookahead_distances[2]
        else:
            advanceDistance = lookahead_distances[3]

        # 确保索引不越界
        advanceDistance = min(advanceDistance, len(self.trajectory_latitude) - 1)

        self.ref_latitude = self.trajectory_latitude[advanceDistance]
        self.ref_longitude = self.trajectory_longitude[advanceDistance]
        self.ref_yaw = self.trajectory_angle[advanceDistance] / 180 * PI

    def cb_hmi(self, msg):
        """HMI数据回调"""
        self.platoon_state = msg.platoon_state

        # 判断逻辑：如果没有从ROS话题中收到合理的本车ID，使用初始化配置的ID
        if msg.platoon_id > 0:
            self.ego_id = msg.platoon_id
        else:
            # 如果HMI发送的ID无效，恢复为初始配置的ID
            self.ego_id = self.initial_ego_id
            self.get_logger().warn(f"收到无效的platoon_id: {msg.platoon_id}，已恢复为初始配置ID: {self.ego_id}")

        self.get_logger().info(f"HMI更新 - platoon_state: {self.platoon_state}, ego_id: {self.ego_id}")

    # ==========================================================================
    # 数据记录
    # ==========================================================================
    def _init_csv_logging(self):
        """初始化CSV日志"""
        path = os.path.expanduser('/home/nvidia/AutoDrive/src/pid/data')
        os.makedirs(path, exist_ok=True)
        ts = datetime.datetime.now().strftime("%m%d_%H%M")
        # 文件名格式: car{车辆ID}_{时间戳}.csv
        self.csv_file = f"{path}/car{self.ego_id}_{ts}.csv"

        with open(self.csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                't_s', 'id', 'mode', 'ego_arc', 'ego_v', 'target_a', 'thr', 'brk', 'angle',
                'lat', 'lon', 'gap_front', 'gap_leader', 'distance_front', 'distance_leader',
                'ref_v', 'front_v', 'leader_v', 'mpc_status', 'alpha', 'beta'
            ])
        self.start_time = time.time()

    def _write_log(self, a, t, b, angle):
        """写入CSV日志"""
        L = self.L

        if self.ego_id == 1:
            gap_front = 0.0
            gap_leader = 0.0
            distance_front = 0.0
            distance_leader = 0.0
        else:
            distance_front = self.front_arc - self.ego_arc
            distance_leader = self.leader_arc - self.ego_arc
            gap_front = distance_front - L
            n_between = self.ego_id - 2
            gap_leader = distance_leader - (n_between + 1) * L

        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 2),
                self.ego_id, self.platoon_state,
                round(self.ego_arc, 2), round(self.ego_speed, 2),
                round(a, 3), int(t), int(b), round(angle, 2),
                round(self.ego_lat, 7), round(self.ego_lon, 7),
                round(gap_front, 2), round(gap_leader, 2),
                round(distance_front, 2), round(distance_leader, 2),
                round(self.ref_speed, 2), round(self.front_speed, 2), round(self.leader_speed, 2),
                self.mpc_status, self.alpha, self.beta
            ])


def main(args=None):
    global node_launcher

    # 步骤1：启动其他必要的节点
    print("\n" + "=" * 60)
    print("  集成控制器一键启动程序")
    print("=" * 60)
    print()

    node_launcher = NodeLauncher()
    node_launcher.launch_nodes()

    # 注册退出时的清理函数
    atexit.register(node_launcher.shutdown_nodes)

    # 等待其他节点完全启动
    print("等待其他节点完全启动...")
    time.sleep(2)
    print()

    # 步骤2：初始化ROS 2并启动集成控制器
    rclpy.init(args=args)

    try:
        rosNode = IntegratedControl()
        print("=" * 60)
        print(">>> [SUCCESS] 集成控制节点已成功初始化")
        print("=" * 60)
        print(">>> 功能:")
        print("    - 纵向控制: PLF拓扑MPC编队控制")
        print("    - 横向控制: PID转向控制")
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
        print("=" * 60)
        print(">>> 所有节点已启动，正在等待数据...")
        print("=" * 60)
        print()

        rclpy.spin(rosNode)

    except KeyboardInterrupt:
        print("\n[INFO] 用户按下 Ctrl+C，正在停止所有节点...")
    except Exception as e:
        print(f"\n[FATAL] 节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(">>> 正在关闭 ROS 2 资源...")
        rclpy.shutdown()

        # 关闭所有启动的节点
        if node_launcher:
            node_launcher.shutdown_nodes()


if __name__ == '__main__':
    main()
