#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: y_mpc.py
# @Project: Auto-Driving System
# @CreateTime: 2025/9/15
# @Description: MPC-based longitudinal platoon control

import sys
import os
import rclpy
from rclpy.node import Node
import time
import yaml
from math import *
from car_interfaces.msg import *
from easydict import EasyDict
import numpy as np
import csv  
import datetime  
from collections import deque
import casadi as ca

PI = 3.1415926535

ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))


class PIDController:    
    """
    下层PID控制器类
    """
    def __init__(self, kp, ki, kd, dt, min_out=-1.0, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.min_out = min_out
        self.max_out = max_out

        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0 # 上一次滤波后的微分项
        self.alpha = 0.4  # 针对微分低通滤波系数
        self.dead_zone = 0.05  # 死区范围

    def update(self, error):
        # 死区处理
        if abs(error) < self.dead_zone:
            error = 0.0
        
        # 比例项
        p = self.kp * error
        # 微分项（低通滤波）
        
        """
         y_k = α * x_k + (1 - α) * y_(k-1)
            其中，y_k 是当前滤波后的值，
                 x_k 是当前输入，
                 y_(k-1) 是上一次滤波后的值，
                 α 是滤波系数（0 < α < 1）
        """
        d_raw = (error - self.prev_error) / self.dt
        self.filtered_d = self.alpha * d_raw + (1 - self.alpha) * self.filtered_d
        d = self.kd * self.filtered_d
        
        # 积分项
        # 只有当输出没有饱和时才进行积分
        potential_out = p + self.ki * (self.integral + error * self.dt) + d
        if self.min_out < potential_out < self.max_out:
            self.integral += error * self.dt

        i = self.ki * self.integral

        self.prev_error = error

        output = p + i + d
        return max(self.min_out, min(self.max_out, output))

class YMpc(Node):
    # ================================================================== #
    # ==================== 1.   初  始  化   ======================= #
    # ================================================================== #
    def __init__(self, name):
        super().__init__(name)
        
        # =============== 车辆启动系统初始化 ===============
        self.start_time = None
        self.vehicle_state = "UNINITIALIZED"  # 车辆状态管理
        self._last_acceleration = 0.0  # 用于加速度平滑
        self._last_startup_second = -1  # 启动阶段时间记录

        # define publishers
        self.pub_Y_Mpc = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timer_Y_Mpc = self.create_timer(0.1, self.mpc_callback)  # 10Hz

        # define subscribers
        # 获取参考轨迹
        self.subLocalPathPlanning = self.create_subscription(
            LocalPathPlanningInterface,
            'local_path_planning_data',
            self.sub_callback_local_path_planning,
            10)
        # 获取当前车辆状态
        self.subFusion = self.create_subscription(
            FusionInterface,
            'fusion_data',
            self.sub_callback_fusion,
            10)
        # 获取HMI界面信息
        self.subHmiPlatoonState = self.create_subscription(
            HmiPlatoonInterface,
            "platoon_state_data",
            self.sub_callback_platoon_state,
            10)
        # 获取v2x编队信息
        self.subVtxData = self.create_subscription(
            VtxInterface,
            "v2x_to_planner_data",
            self.sub_callback_vtx,
            10)

        root_path = os.getcwd() + "/src/pid/pid/"  # 根目录
        # ======================================================== #
        # 从config文件中读取超参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        # config = yaml.load(f)
        config = yaml.safe_load(f)
        self.yaml_data = EasyDict(config)
        print(self.yaml_data)


        # ==============configs============= #
        vehicles_ids = ['leader', 'follower1', 'follower2']


        # 初始化当前车辆状态
        self.nowSpeed = 0.0
        self.nowAcceleration = 0.0
        self.nowLatitude = 0.0
        self.nowLongitude = 0.0
        self.nowYaw = 0.0
        self.nowPitch = 0.0
        self.nowArcLength = 0.0  # 当前弧长位置

        # 初始化参考车辆状态 
        self.refSpeed = 0.0
        self.refAcceleration = 0.0      
        self.refLatitude = 0.0
        self.refLongitude = 0.0
        self.refYaw = 0.0
        self.refArcLength = 0.0  # 参考弧长位置

        # MPC
        self.dt = 0.1  # 控制周期
        self.N = 10  # 预测时域长度

        self.speed_ref = 2.0  # 默认期望速度

        # 初始化控制周期计数器
        self._control_cycle_count = 0
        self._spacing_log_counter = 0
  
        # 初始化MPC求解器
        self.mpc_solver = None  # 先设置为None，后续再初始化
        
        # 创建PID控制器 - 优化参数以提高响应速度
        self.declare_parameter('pid_kp', 0.6)    # 提高比例增益
        self.declare_parameter('pid_ki', 0.0)   # 适度提高积分增益
        self.declare_parameter('pid_kd', 0.05)   # 添加微分增益抑制震荡
        self.pid_kp = self.get_parameter('pid_kp').get_parameter_value().double_value
        self.pid_ki = self.get_parameter('pid_ki').get_parameter_value().double_value
        self.pid_kd = self.get_parameter('pid_kd').get_parameter_value().double_value

        # 为每辆车创建下层pid控制器 - 添加积分清零机制
        self.lower_pid = PIDController(kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd, dt=self.dt)

            # 编队状态变量
        self.platoon_state = 0          # 编队状态：0-非编队，1-编队模式
        self.ego_platoon_ID = None      # 当前车辆在编队中的ID：1-领车，2/3-跟随车
        self.front_car_state = None     # 前车信息（直接存储V2X消息）

        self.front_car_speed=0.0
        self.front_car_longitude = 0.0
        self.front_car_latitude = 0.0
        self.front_car_yaw = 0.0
        self.front_car_update_time = 0.0  # 前车信息更新时间
        
        # 编队参数
        self.max_platoon_speed = 3.0     # 编队最高速度
        self.desired_spacing = 8.0     # 期望车间距
        self.min_spacing = 5.0         # 安全间距 (m)
        self.max_spacing = 15.0        # 最大间距 (m)

        # 存储所有车辆的V2X状态信息
        self.v2x_vehicles = {}     # 存储所有收到的V2X车辆信息
        
        # 轨迹相关状态
        self.trajectory_available = False
        self.current_trajectory_position = 0.0
        self.trajectory_distances = []
        self.last_path_planning_msg = None
        self.reference_trajectory = []
        self.trajectory_update_counter = 0
        self.last_valid_position = (0.0, 0.0)
        self.distance_calc_debug = True
        
        # 状态滤波和平滑相关变量
        self.speed_filter = deque(maxlen=5)  # 速度滤波器，保存最近5个值
        self.current_velocity = 0.0  # 滤波后的当前速度
        
        # GPS状态
        self.GPS_state = False
        
        # 横向控制变量
        self.control_value_last = 0.0  # 上次横向控制值
        
        # 到达状态
        self.arrived = False
        # 转向灯状态初始化
        self.turn_signals = 0.0 
        # 最后初始化MPC求解器 (需要在所有参数设置完成后)
        print("正在初始化MPC求解器...")
        self.mpc_setup_solver()
        
        print("=== MPC控制器初始化完成 ===")
        
    # ================================================================== #
    # ==================== 2. 主控制循环 ======================= #
    # ================================================================== #
    def mpc_callback(self):
        """主控制回调函数"""
        msgYmpc = PidInterface()
        msgYmpc.timestamp = time.time()

        control_method = ""

        # 更新车辆状态和前车信息
        self.update_vehicle_state()
        self.update_front_car_state()

        self._control_cycle_count += 1

        nowPosX, nowPosY, nowPosZ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)
        refPosX, refPosY, refPosZ = self.conversion_of_coordinates(self.refLatitude, self.refLongitude, 0)

        #####################################横向控制#########################################################
        # 横向控制保持原有PID逻辑
        print("nowyaw=", self.nowYaw)
        print("refyaw=", self.refYaw)
        errorYaw = -self.nowYaw + self.refYaw
        
        # 航向角误差限制在[-pi,pi]
        if(errorYaw > PI):
            errorYaw = errorYaw - 2*PI
        if(errorYaw < -PI):
            errorYaw = errorYaw + 2*PI
        print("ephi=", errorYaw)
        
        # 计算跟踪误差
        errorDistance = ((-nowPosY+refPosY)*cos(self.refYaw)-(-nowPosX+refPosX)*sin(self.refYaw))
        print("ed=", errorDistance)
        
        # 检查异常值
        if abs(errorDistance) > 100:  # 横向偏差超过100米认为异常
            print(f"Warning: Abnormal lateral error {errorDistance:.2f}m, using last control")
            controlValue = self.control_value_last
        else:
            # 变参数pid控制
            if(self.nowSpeed < 3.5):
                pidEphi = self.yaml_data.lateral_ephi_1
                pidEd = self.yaml_data.lateral_ed_1
            elif(self.nowSpeed < 5.5):
                pidEphi = self.yaml_data.lateral_ephi_2
                pidEd = self.yaml_data.lateral_ed_2
            elif(self.nowSpeed < 7.5):
                pidEphi = self.yaml_data.lateral_ephi_3
                pidEd = self.yaml_data.lateral_ed_3  
            else:
                pidEphi = self.yaml_data.lateral_ephi_4
                pidEd = self.yaml_data.lateral_ed_4

            # controlValue为方向盘转向
            controlValue = (pidEphi*errorYaw + pidEd*errorDistance)*180/PI
            controlValue = controlValue*20.0

        # 控制平滑与限幅
        control_number = 0.3
        controlValue = control_number * controlValue + (1 - control_number) * self.control_value_last
        self.control_value_last = controlValue

        if(controlValue > 500):
            controlValue = 500.0
        if(controlValue < -500):
            controlValue = -500.0
        print("control=", controlValue)

        # 保存横向控制结果到消息
        msgYmpc.angle = controlValue
        msgYmpc.velocity = self.refSpeed
        print("refspeed=", msgYmpc.velocity)
        
        # 添加弧长位置信息
        msgYmpc.arc_position = float(self.current_trajectory_position) if hasattr(self, 'current_trajectory_position') else 0.0

        ###################################纵向编队控制####################################################
        """
        思路梳理
        1. 获取参考轨迹
            get_virtual_leader函数  

        2. 获取车辆状态和前车状态（如果有的话）
        update_vehicle_state 和 update_front_car_state函数

        3. 场景判断
            1. 场景1：智能启动阶段，在5秒能平稳启动车辆
            2. 场景2：无前车或非编队模式sub_callback_local_path_planning
                直接使用上层mpc控制器跟随虚拟leader做纵向跟踪控制，输出目标加速度
            3. 场景3：编队模式，1号车跟随虚拟领车，2号车跟1号车，3号车跟2号车
                其中1号车：直接使用上层mpc控制器跟随虚拟leader做纵向跟踪控制，输出目标加速度
                2号车和3号车：使用上层mpc控制器跟随前车做纵向跟踪控制，输出目标加速度
            4. 场景4：停车跟随，分为紧急制动与缓慢制动
        
        4. 通过当前加速度和目标加速度计算控制输出（使用下层pid控制器） 
        5. 通过话题发布控制指令

        """

        # =============== 平稳启动系统 ===============
        # 初始化启动时间记录
        if not hasattr(self, 'start_time') or self.start_time is None:
            self.start_time = time.time()
            self.vehicle_state = "INITIALIZING"  # 车辆状态：INITIALIZING, STARTING, RUNNING
            print("=== 车辆启动系统初始化 ===")
        
        # 计算启动阶段时间
        startup_time = time.time() - self.start_time
        
        # 平稳启动状态机（针对低速环境优化）
        if startup_time < 4.0:  # 前6秒为启动阶段（低速环境）
            target_acceleration, control_strategy = self.smooth_startup_control(startup_time)
            
        # =============== 正常运行阶段 ===============
        else:
            self.vehicle_state = "RUNNING"
            
            # 检查轨迹和GPS状态
            if not hasattr(self, 'trajectory_available') or not self.trajectory_available or not self.GPS_state:
                # 轨迹或GPS不可用：安全停车
                target_acceleration = -1.5 if self.nowSpeed > 0.5 else -0.5
                control_strategy = "Emergency_Stop"
                print("Warning: Trajectory or GPS unavailable, emergency stopping")
                
            else:
                # 获取虚拟leader参考轨迹
                if hasattr(self, 'get_virtual_leader'):
                    ref_arc_lengths, ref_velocities, ref_accelerations = self.get_virtual_leader()
                    
                    if ref_arc_lengths is None:
                        # 虚拟leader生成失败：回退策略
                        speed_error = self.refSpeed - self.nowSpeed
                        target_acceleration = np.clip(speed_error * 1.0, -2.0, 2.0)
                        control_strategy = "Fallback_Control"
                        print("Warning: Virtual leader generation failed, using fallback")
                        
                        # 设置默认虚拟leader值
                        msgYmpc.vleader_arc_position = float(self.current_trajectory_position) if hasattr(self, 'current_trajectory_position') else 0.0
                        msgYmpc.vleader_velocity = float(self.nowSpeed)
                        msgYmpc.vleader_acceleration = 0.0
                        
                    else:
                        # 正常MPC控制或编队控制
                        target_acceleration, control_strategy, mpc_status = self.normal_operation_control()
                        
                        # 保存虚拟leader信息（使用预测时域的第一个点）
                        msgYmpc.vleader_arc_position = float(ref_arc_lengths[0]) if ref_arc_lengths else 0.0
                        msgYmpc.vleader_velocity = float(ref_velocities[0]) if ref_velocities else 0.0
                        msgYmpc.vleader_acceleration = float(ref_accelerations[0]) if ref_accelerations else 0.0
                else:
                    # MPC系统未初始化，使用备用速度控制
                    target_acceleration, control_strategy = self.backup_speed_control()
                    
                    # 设置默认虚拟leader值
                    msgYmpc.vleader_arc_position = float(self.current_trajectory_position) if hasattr(self, 'current_trajectory_position') else 0.0
                    msgYmpc.vleader_velocity = float(self.nowSpeed)
                    msgYmpc.vleader_acceleration = 0.0
        
        # =============== 安全限制和平滑处理 ===============
        target_acceleration = self.apply_safety_limits(target_acceleration)
        
        # =============== 下层PID控制器：加速度转换为油门/刹车 ===============
        control_cmd = self.acceleration_to_throttle_brake(target_acceleration, controlValue)
        
        # 提取油门和刹车值
        throttle_value = control_cmd['throttle']
        brake_value = control_cmd['brake']
        # 确保值在 0-1 范围内（防止溢出）
        clamped_throttle = max(0.0, min(1.0, throttle_value))
        clamped_brake = max(0.0, min(1.0, brake_value))
        
        # 转换为 0-100 并取整
        msgYmpc.throttle_percentage = int(round(clamped_throttle * 100))
        msgYmpc.braking_percentage = int(round(clamped_brake * 100))
                
        # # =============== 更新消息并发布 ===============
        # msgYmpc.throttle_percentage = control_cmd['throttle']
        # msgYmpc.braking_percentage = control_cmd['brake']
        msgYmpc.acceleration = target_acceleration
        

        # throttle = 0
        # brake = 0
        #         # 特殊处理：完全停车状态
        # if self.current_velocity < 0.05 and abs(target_acceleration) < 0.1:
        #     throttle = 0
        #     brake = 40  # 保持轻微刹车
        #     control_method += "_Hold"
            
        # msgYmpc.throttle_percentage = throttle
        # msgYmpc.braking_percentage = brake


        # 数据记录
        if hasattr(self, 'save_control_data_to_csv'):
            self.save_control_data_to_csv(
                target_acceleration, 
                control_cmd['throttle'], 
                control_cmd['brake'], 
                control_strategy, 
                getattr(locals(), 'mpc_status', 'Unknown')
            )
        

        # 限制最大油门/刹车（安全）
        msgYmpc.throttle_percentage = min(100, max(0, msgYmpc.throttle_percentage))
        msgYmpc.braking_percentage = min(100, max(0, msgYmpc.braking_percentage))

        if self.arrived:
            msgYmpc.gear = 2
        else:
            msgYmpc.gear = 3
        msgYmpc.turn_signals = self.turn_signals


        # 发布控制指令
        self.pub_Y_Mpc.publish(msgYmpc)
        print("throttle = %d, brake = %d"%(msgYmpc.throttle_percentage, msgYmpc.braking_percentage))


        # 定期输出关键信息
        if self._control_cycle_count % 30 == 0:  # 每3秒输出一次
            print(f"=== Control Summary (Cycle {self._control_cycle_count}) ===")
            print(f"Vehicle State: {getattr(self, 'vehicle_state', 'UNKNOWN')}")
            print(f"Strategy: {control_strategy}, Speed: {self.nowSpeed:.2f}m/s")
            print(f"Target Accel: {target_acceleration:.3f}m/s², Throttle: {control_cmd['throttle']:.3f}")
            if hasattr(self, 'platoon_state') and self.platoon_state == 1:
                print(f"Platoon Mode: Vehicle {getattr(self, 'ego_platoon_ID', 'Unknown')}")
        

    # ================================================================== #
    # ==================== 3. 通过fusion获取当前状态 ======================= #
    # ================================================================== #

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """订阅融合数据"""
        self.nowSpeed = msgFusion.carspeed
        print("nowspeed^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        print(self.nowSpeed) 

        self.nowLatitude = msgFusion.latitude
        self.nowLongitude = msgFusion.longitude

        if abs(self.nowLatitude) < 1 and abs(self.nowLongitude) < 1:
            self.GPS_state = False
        else:
            self.GPS_state = True 

        self.nowYaw = msgFusion.yaw/180*PI
        self.nowPitch = msgFusion.pitch

    # ================================================================== #
    # ==================== 4. 通过local_path获取参考状态 ======================= #
    # ================================================================== #



    def sub_callback_local_path_planning(self, msgLocalPathPlanning:LocalPathPlanningInterface):
        """订阅局部路径规划数据"""
        if msgLocalPathPlanning.latitude is None or len(msgLocalPathPlanning.latitude) == 0:
            self.trajectory_state = False
            return
        
        self.refSpeed = msgLocalPathPlanning.speed[0]

        # 边界检查
        if(self.nowSpeed < 3.5):
            advanceDistance = self.yaml_data.advance_distance_1
        elif(self.nowSpeed < 5.5):
            advanceDistance = self.yaml_data.advance_distance_2
        elif(self.nowSpeed < 7.5):
            advanceDistance = self.yaml_data.advance_distance_3
        else:
            advanceDistance = self.yaml_data.advance_distance_4

        # 确保不越界
        advanceDistance = min(advanceDistance, len(msgLocalPathPlanning.latitude) - 1)
        
        # 设置参考点
        self.refLatitude = msgLocalPathPlanning.latitude[advanceDistance]
        self.refLongitude = msgLocalPathPlanning.longitude[advanceDistance]
        
        self.refYaw = msgLocalPathPlanning.angle[advanceDistance]/180*PI
        self.turn_signals = msgLocalPathPlanning.turn_signals

        if len(msgLocalPathPlanning.latitude) == 500:
            self.trajectory_state = True
            
            # 保存完整的参考轨迹
            self.reference_trajectory = []
            for i in range(len(msgLocalPathPlanning.latitude)):
                self.reference_trajectory.append((msgLocalPathPlanning.latitude[i], msgLocalPathPlanning.longitude[i]))
            
            # 构建轨迹距离映射
            self.build_trajectory_distance_map()
            
            if abs(msgLocalPathPlanning.latitude[1] - msgLocalPathPlanning.latitude[-1]) < 1e-8 and \
                abs(msgLocalPathPlanning.longitude[1] - msgLocalPathPlanning.longitude[-1]) < 1e-8 and \
                self.nowSpeed < 0.001:
                self.arrived = True
            else:
                self.arrived = False
        else:
            self.trajectory_state = False
            self.trajectory_available = False




    # ================================================================== #
    # ==================== 5. 虚拟leader生成 ======================= #
    # ================================================================== #
    def get_virtual_leader(self):
        """
        生成虚拟领车状态和MPC参考轨迹序列（纵向控制专用）
        编队控制策略：
        - 编队模式：1号车跟随虚拟leader，2号车跟1号车，3号车跟2号车
        - 非编队模式：所有车辆都跟随虚拟leader（单车控制）
        
        Returns:
            tuple: (ref_arc_lengths, ref_velocities, ref_accelerations) - 仅纵向控制相关
        """

        # MPC参数
        N = self.N  # 使用类变量
        dt = self.dt  # 使用类变量
        
        # 获取当前车辆的弧长位置
        current_idx, current_arc_length = self.project_point_to_trajectory(self.nowLatitude, self.nowLongitude)
        if current_arc_length is None:
            print("Warning: Failed to project current position to trajectory")
            return None, None, None
        
        # ===== 停车逻辑检查 =====
        is_stopping = False
        if self.refSpeed < 0.05:  # 参考速度接近0，触发停车
            is_stopping = True
            print(f"Stopping mode activated: refSpeed={self.refSpeed:.3f} m/s")
        
        # 生成预测时域内的参考序列（仅纵向控制）
        ref_arc_lengths = []
        ref_velocities = []
        ref_accelerations = []
        
        # ===== 停车模式：生成减速到停止的轨迹 =====
        if is_stopping:
            print("Generating stopping trajectory...")
            
            # 计算停车距离（基于当前速度）
            # 使用简化的减速模型：假设恒定减速度
            decel_rate = 1.5  # m/s² 舒适减速度
            stopping_distance = (self.nowSpeed ** 2) / (2 * decel_rate) if self.nowSpeed > 0.1 else 0.0
            stopping_time = self.nowSpeed / decel_rate if self.nowSpeed > 0.1 else 0.0
            
            print(f"Current speed: {self.nowSpeed:.2f} m/s")
            print(f"Estimated stopping distance: {stopping_distance:.2f} m")
            print(f"Estimated stopping time: {stopping_time:.2f} s")
            
            for k in range(N):
                t = k * dt  # 当前预测时间
                
                # 位置预测（匀减速运动）
                if t < stopping_time:
                    # 减速阶段
                    predicted_distance = self.nowSpeed * t - 0.5 * decel_rate * t**2
                    predicted_speed = max(0.0, self.nowSpeed - decel_rate * t)
                    predicted_accel = -decel_rate
                else:
                    # 已停止
                    predicted_distance = stopping_distance
                    predicted_speed = 0.0
                    predicted_accel = 0.0
                
                # 弧长位置
                predicted_arc_length = current_arc_length + predicted_distance
                ref_arc_lengths.append(max(0.0, predicted_arc_length))
                
                # 速度和加速度
                ref_velocities.append(predicted_speed)
                ref_accelerations.append(predicted_accel)
            
            # 停车模式统计
            print(f"Stopping trajectory generated: {N} points")
            print(f"Speed profile: {self.nowSpeed:.2f} -> {ref_velocities[-1]:.2f} m/s")
            print(f"Final arc length: {ref_arc_lengths[-1]:.2f} m")
            
            return ref_arc_lengths, ref_velocities, ref_accelerations
        
        # 确定跟随目标和控制策略（简化版本）
        follow_virtual_leader = True  # 默认跟随虚拟leader
        front_car_arc_length = None
        current_spacing = None
        
        # 编队跟随逻辑：只有2号、3号车在编队模式下且有前车数据时才跟随前车
        if (self.platoon_state == 1 and 
            self.ego_platoon_ID in [2, 3] and 
            self.front_car_state is not None):
            
            _, front_car_arc_length = self.project_point_to_trajectory(
                self.front_car_state.latitude, 
                self.front_car_state.longitude
            )
            
            if front_car_arc_length is not None:
                follow_virtual_leader = False
                current_spacing = self.calculate_distance_between_vehicles(
                    self.front_car_state.latitude, self.front_car_state.longitude,
                    self.nowLatitude, self.nowLongitude
                )
        
        # 生成预测时域的参考轨迹
        for k in range(N):
            try:
                if follow_virtual_leader:
                    # 跟随虚拟leader：基于参考轨迹和当前速度预测
                    predicted_arc_length = current_arc_length + k * dt * max(self.nowSpeed, 0.1)
                else:
                    # 跟随前车：基于前车位置和期望间距
                    target_spacing = self.desired_spacing
                    
                    # 根据当前车间距动态调整目标间距
                    if current_spacing is not None:
                        if current_spacing > self.max_spacing:
                            target_spacing = self.desired_spacing * 0.8
                        elif current_spacing < self.min_spacing:
                            target_spacing = self.desired_spacing * 1.2
                    
                    # 预测前车位置（假设前车以当前速度运动）
                    front_car_speed = getattr(self.front_car_state, 'speed', self.refSpeed)
                    predicted_front_arc = front_car_arc_length + k * dt * front_car_speed
                    predicted_arc_length = predicted_front_arc - target_spacing
                
                ref_arc_lengths.append(max(0.0, predicted_arc_length))
                
                # 查找对应的轨迹索引和参考值
                trajectory_idx = self.find_trajectory_index_by_arc_length(predicted_arc_length)
                
                if trajectory_idx is not None and trajectory_idx < len(self.last_path_planning_msg.latitude):
                    # 验证轨迹点的GPS坐标
                    traj_lat = self.last_path_planning_msg.latitude[trajectory_idx]
                    traj_lon = self.last_path_planning_msg.longitude[trajectory_idx]
                    
                    if self.validate_gps_coordinates(traj_lat, traj_lon):
                        # 速度参考生成
                        if trajectory_idx < len(self.last_path_planning_msg.speed):
                            base_ref_speed = self.last_path_planning_msg.speed[trajectory_idx]
                        else:
                            base_ref_speed = self.refSpeed
                        
                        # 根据跟随模式调整速度
                        if follow_virtual_leader:
                            # 跟随虚拟leader：使用轨迹规划速度
                            ref_speed = base_ref_speed
                            if self.platoon_state == 1:
                                # 编队模式下限制最大速度
                                ref_speed = min(ref_speed, self.max_platoon_speed)
                        else:
                            # 跟随前车：根据车间距动态调整速度
                            ref_speed = base_ref_speed
                            if self.platoon_state == 1:
                                ref_speed = min(ref_speed, self.max_platoon_speed)
                                
                                # 根据车间距微调速度
                                if current_spacing is not None:
                                    if current_spacing > self.max_spacing:
                                        # 车间距过大，提高速度追赶
                                        speed_factor = min(1.15, 1.0 + (current_spacing - self.max_spacing) / 20.0)
                                        ref_speed = min(ref_speed * speed_factor, self.max_platoon_speed)
                                    elif current_spacing < self.min_spacing:
                                        # 车间距过小，降低速度
                                        speed_factor = max(0.85, 1.0 - (self.min_spacing - current_spacing) / 10.0)
                                        ref_speed = ref_speed * speed_factor
                                    
                                    # 如果车间距严重偏离，添加额外的速度调整
                                    if current_spacing < self.min_spacing * 0.8:
                                        ref_speed = min(ref_speed, 1.0)  # 紧急减速
                        
                        ref_velocities.append(max(0.1, ref_speed))  # 确保速度不为负
                        
                        # 加速度参考
                        if (hasattr(self.last_path_planning_msg, 'acceleration') and 
                            trajectory_idx < len(self.last_path_planning_msg.acceleration)):
                            base_acceleration = self.last_path_planning_msg.acceleration[trajectory_idx]
                        else:
                            base_acceleration = 0.0
                        
                        # 在跟随模式下，根据速度变化调整加速度
                        if not follow_virtual_leader and k > 0:
                            # 计算速度变化率作为加速度参考
                            speed_diff = ref_velocities[-1] - ref_velocities[-2] if len(ref_velocities) > 1 else 0.0
                            computed_accel = speed_diff / dt
                            # 限制加速度范围
                            computed_accel = max(-2.0, min(2.0, computed_accel))
                            ref_accelerations.append(computed_accel)
                        else:
                            ref_accelerations.append(base_acceleration)
                        
                    else:
                        # GPS坐标无效，使用安全的默认值
                        safe_speed = min(self.refSpeed, self.max_platoon_speed) if self.platoon_state == 1 else self.refSpeed
                        ref_velocities.append(safe_speed)
                        ref_accelerations.append(0.0)
                else:
                    # 超出轨迹范围，使用安全的默认值
                    safe_speed = min(self.refSpeed, self.max_platoon_speed) if self.platoon_state == 1 else self.refSpeed
                    ref_velocities.append(safe_speed)
                    ref_accelerations.append(0.0)
                    
            except Exception as e:
                print(f"Error generating virtual leader reference at step {k}: {e}")
                # 使用安全的默认值
                ref_arc_lengths.append(current_arc_length + k * dt * 1.0)  # 默认1m/s前进
                ref_velocities.append(1.0)  # 默认低速
                ref_accelerations.append(0.0)
        
        # 生成结果统计
        if ref_arc_lengths:
            control_mode = "Following front car" if not follow_virtual_leader else "Following virtual leader"
            print(f"Generated virtual leader: {len(ref_arc_lengths)} points")
            print(f"Control mode: {control_mode}")
            print(f"Arc range: {ref_arc_lengths[0]:.2f} - {ref_arc_lengths[-1]:.2f}m")
            print(f"Speed range: {min(ref_velocities):.2f} - {max(ref_velocities):.2f} m/s")
            if current_spacing is not None:
                print(f"Current spacing: {current_spacing:.2f}m, Target: {self.desired_spacing:.2f}m")
        
        return ref_arc_lengths, ref_velocities, ref_accelerations
    
    
    def find_trajectory_index_by_arc_length(self, target_arc_length):
        """根据弧长位置查找对应的轨迹索引"""
        if not self.trajectory_available or not self.trajectory_distances:
            return None
        
        # 确保弧长位置在有效范围内
        if target_arc_length < 0:
            return 0
        if target_arc_length >= self.trajectory_distances[-1]:
            return len(self.trajectory_distances) - 1
        
        # 二分查找最接近的弧长位置
        left, right = 0, len(self.trajectory_distances) - 1
        while left < right:
            mid = (left + right) // 2
            if self.trajectory_distances[mid] < target_arc_length:
                left = mid + 1
            else:
                right = mid
        
        # 返回最接近的索引
        if left > 0:
            # 比较left和left-1，选择更接近的
            diff_left = abs(self.trajectory_distances[left] - target_arc_length)
            diff_prev = abs(self.trajectory_distances[left-1] - target_arc_length)
            return left-1 if diff_prev < diff_left else left
        
        return left

    def find_closest_trajectory_index(self):
        """找到当前车辆在轨迹上的最近点索引"""
        if not hasattr(self, 'last_path_planning_msg') or self.last_path_planning_msg is None:
            return 0
        
        min_dist = float('inf')
        closest_idx = 0
        
        ego_x, ego_y, _ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)
        
        for i, (lat, lon) in enumerate(zip(self.last_path_planning_msg.latitude, self.last_path_planning_msg.longitude)):
            ref_x, ref_y, _ = self.conversion_of_coordinates(lat, lon, 0)
            dist = sqrt((ego_x - ref_x)**2 + (ego_y - ref_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx


    # ================================================================== #
    # ==================== 6. 状态更新 ======================= #
    # ================================================================== #
    def update_vehicle_state(self):
        """更新和验证车辆状态信息"""
        try:
            # 速度滤波处理
            self.speed_filter.append(self.nowSpeed)
            if len(self.speed_filter) > 0:
                # 去除异常值后取平均
                speeds = list(self.speed_filter)
                speeds_filtered = [s for s in speeds if 0 <= s <= 50]  # 去除异常速度
                if speeds_filtered:
                    self.current_velocity = np.mean(speeds_filtered)
                else:
                    self.current_velocity = self.nowSpeed
            else:
                self.current_velocity = self.nowSpeed
            
            # 更新轨迹位置
            if self.trajectory_available and self.GPS_state:
                # 验证当前GPS坐标
                if self.validate_gps_coordinates(self.nowLatitude, self.nowLongitude):
                    _, current_arc_pos = self.project_point_to_trajectory(self.nowLatitude, self.nowLongitude)
                    if current_arc_pos is not None:
                        # 位置平滑处理
                        alpha = 0.8
                        if self.current_trajectory_position > 0:
                            self.current_trajectory_position = alpha * current_arc_pos + (1 - alpha) * self.current_trajectory_position
                        else:
                            self.current_trajectory_position = current_arc_pos
                        
                        # 更新当前弧长位置
                        self.nowArcLength = self.current_trajectory_position
                        
                        # 更新最后有效位置
                        self.last_valid_position = (self.nowLatitude, self.nowLongitude)
                    else:
                        print("Warning: Failed to project vehicle position to trajectory")
                else:
                    print(f"Warning: Invalid GPS coordinates: ({self.nowLatitude}, {self.nowLongitude})")
            else:
                # 轨迹不可用时，基于速度积分更新位置
                if self.current_trajectory_position > 0:
                    self.current_trajectory_position += self.current_velocity * self.dt
                
        except Exception as e:
            print(f"Error updating vehicle state: {e}")

    def update_front_car_state(self):
        """更新前车状态信息"""
        if self.platoon_state != 1 or self.front_car_state is None:
            return
        
        try:
            # 检查前车信息时效性
            current_time = time.time()
            
            # V2X数据时效性检查（优先使用类变量的更新时间）
            if hasattr(self, 'front_car_update_time'):
                if current_time - self.front_car_update_time > 2.0:  # 超过2秒认为过期
                    print("Warning: Front car data is outdated (class timer)")
                    self.front_car_state = None  # 清除过期数据
                    return
            elif hasattr(self.front_car_state, 'timestamp'):
                if current_time - self.front_car_state.timestamp > 2.0:  # 超过2秒认为过期
                    print("Warning: Front car data is outdated (message timestamp)")
                    self.front_car_state = None  # 清除过期数据
                    return
            
            # 验证前车GPS坐标
            if self.validate_gps_coordinates(self.front_car_state.latitude, self.front_car_state.longitude):
                # 更新前车状态变量
                self.front_car_speed = getattr(self.front_car_state, 'speed', 0.0)
                self.front_car_longitude = self.front_car_state.longitude
                self.front_car_latitude = self.front_car_state.latitude
                self.front_car_yaw = getattr(self.front_car_state, 'yaw', 0.0)
                
                # 更新时间戳
                if not hasattr(self, 'front_car_update_time'):
                    self.front_car_update_time = current_time
                
                # 计算并记录车间距
                current_spacing = self.calculate_distance_between_vehicles(
                    self.front_car_latitude, self.front_car_longitude,
                    self.nowLatitude, self.nowLongitude
                )
                
                if current_spacing > 0:
                    # 车间距状态判断
                    spacing_status = "NORMAL"
                    if current_spacing > self.max_spacing:
                        spacing_status = "TOO_LARGE"
                    elif current_spacing < self.min_spacing:
                        spacing_status = "TOO_SMALL"
                    
                    # 定期输出车间距信息（减少输出频率）
                    if not hasattr(self, '_spacing_log_counter'):
                        self._spacing_log_counter = 0
                    self._spacing_log_counter += 1
                    
                    if self._spacing_log_counter % 20 == 0:  # 每20次更新输出一次
                        print(f"Vehicle {self.ego_platoon_ID}: Spacing = {current_spacing:.2f}m [{spacing_status}]")
                        print(f"Target range: {self.min_spacing:.1f} - {self.max_spacing:.1f}m")
                        
            else:
                print(f"Warning: Invalid front car GPS coordinates: "
                      f"({self.front_car_state.latitude}, {self.front_car_state.longitude})")
                
        except Exception as e:
            print(f"Error updating front car state: {e}")
            # 安全措施：清除可能损坏的前车状态
            self.front_car_state = None


    # ================================================================== #
    # ==================== 6. 车辆启动 ======================= #
    # ================================================================== #


    # =============== 平稳启动系统方法 ===============
    def smooth_startup_control(self, startup_time):
        """
        平稳启动控制器（针对低速环境优化，最大速度5m/s）
        
        Args:
            startup_time: 启动阶段经过的时间 (秒)
        
        Returns:
            tuple: (target_acceleration, control_strategy)
        """
        
        # 阶段1: 初始化阶段 (0-1.5秒) - 缩短初始化时间
        if startup_time < 1.5:
            self.vehicle_state = "INITIALIZING"
            target_acceleration = 0.2  # 保持静止
            control_strategy = "Initialization"
            
            if startup_time < 1.0:
                print(f"车辆初始化中... ({startup_time:.1f}s)")
            
        # 阶段2: 缓慢启动阶段 (1.5-4秒) - 适合低速环境的启动
        elif startup_time < 4.0:
            self.vehicle_state = "STARTING"
            # 渐进式加速，从0平滑增长到目标加速度
            progress = (startup_time - 1.5) / 2.5  # 0到1的进度
            # 使用平滑的S型曲线
            smooth_factor = 3 * progress**2 - 2 * progress**3
            
            # 低速环境下，目标加速度从0增长到0.8 m/s²（更温和）
            target_acceleration = 0.8 * smooth_factor
            
            # 低速环境的速度限制
            if self.nowSpeed > 2.5:  # 启动阶段限制在2.5m/s
                target_acceleration = min(target_acceleration, 0.2)
                
            control_strategy = "Smooth_Startup_LowSpeed"
            
            # 改进：使用更精确的时间间隔输出，避免重复打印
            current_second = int(startup_time * 2) / 2  # 每0.5秒输出一次
            if current_second != getattr(self, '_last_startup_time', -1):
                print(f"低速平稳启动中... 时间: {startup_time:.1f}s, 速度: {self.nowSpeed:.1f}m/s, 加速度: {target_acceleration:.2f}m/s²")
                self._last_startup_time = current_second
                
        # 阶段3: 过渡到正常控制 (4-6秒) - 缩短过渡时间
        else:
            self.vehicle_state = "TRANSITION"
            # 逐渐过渡到参考速度控制
            progress = (startup_time - 4.0)  # 0到1的进度，1秒完成过渡
            
            # 目标速度逐渐从当前速度过渡到参考速度（限制在5m/s以内）
            if hasattr(self, 'refSpeed') and self.refSpeed > 0:
                # 确保参考速度不超过低速环境限制
                limited_ref_speed = min(self.refSpeed, 4.5)  # 参考速度限制在4.5m/s
                target_speed = self.nowSpeed + progress * (limited_ref_speed - self.nowSpeed)
            else:
                # 默认过渡到3m/s（适合低速环境）
                target_speed = self.nowSpeed + progress * (3.0 - self.nowSpeed)
                
            # 低速环境下的速度控制，使用更小的增益
            speed_error = target_speed - self.nowSpeed
            target_acceleration = np.clip(speed_error * 0.6, -0.8, 1.0)  # 更温和的控制
            
            control_strategy = "Startup_Transition_LowSpeed"
            
            if startup_time > 5.5:
                print("低速启动阶段即将完成，切换到正常运行模式...")
        
        return target_acceleration, control_strategy

    def apply_safety_limits(self, target_acceleration):
        """
        应用安全限制和平滑处理（针对低速环境优化）
        
        Args:
            target_acceleration: 目标加速度
        
        Returns:
            float: 限制后的安全加速度
        """
        
        # 低速环境的基本加速度限制（更保守）
        max_accel = 1.5   # 降低最大加速度
        max_decel = -2.5  # 降低最大减速度
        
        # 根据当前速度调整限制（针对5m/s以下的低速环境）
        if self.nowSpeed < 0.5:
            # 启动时更谨慎
            max_accel = 1.0
            max_decel = -1.5
        elif self.nowSpeed < 2.0:
            # 低速时平稳
            max_accel = 1.2
            max_decel = -2.0
        elif self.nowSpeed > 4.5:
            # 接近速度上限时更保守
            max_accel = 0.8
            max_decel = -2.5
        
        # 应用限制
        safe_acceleration = np.clip(target_acceleration, max_decel, max_accel)
        
        # 低速环境下的平滑处理：更严格的jerk限制
        if hasattr(self, '_last_acceleration'):
            max_jerk = 1.5  # m/s³，降低jerk限制使动作更平稳
            dt = 0.1  # 控制周期
            max_accel_change = max_jerk * dt
            
            accel_change = safe_acceleration - self._last_acceleration
            if abs(accel_change) > max_accel_change:
                safe_acceleration = self._last_acceleration + np.sign(accel_change) * max_accel_change
        
        # 记录当前加速度用于下次平滑
        self._last_acceleration = safe_acceleration
        
        return safe_acceleration

    def normal_operation_control(self):
        """
        正常运行阶段的控制逻辑
        编队控制策略：
        - 非编队模式或1号车：使用MPC控制器跟踪虚拟leader
        - 编队模式下的2号车：使用MPC控制器跟随1号车
        - 编队模式下的3号车：使用MPC控制器跟随2号车
        
        Args:
            ref_arc_lengths: 参考弧长数组
            ref_velocities: 参考速度数组  
            ref_accelerations: 参考加速度数组
        
        Returns:
            tuple: (target_acceleration, control_strategy, mpc_status)
        """
        
        # 检查编队状态和车辆ID
        if hasattr(self, 'platoon_state') and self.platoon_state == 1:
            vehicle_id = getattr(self, 'ego_platoon_ID', 1)
            
            if vehicle_id == 1:
                # 1号车：使用MPC控制器跟踪虚拟leader
                target_acceleration = self.mpc_solve()
                control_strategy = "MPC_Leader_VirtualTrack"
                mpc_status = "Success"
                print(f"Vehicle 1: Following virtual leader with MPC control")
                    
            elif vehicle_id in [2, 3] and self.front_car_state is not None:
                # 2号车跟1号车，3号车跟2号车：使用MPC控制器做编队跟车
                target_acceleration = self.mpc_solve()  # MPC求解器已在get_virtual_leader中处理编队逻辑
                control_strategy = f"MPC_Platoon_Vehicle{vehicle_id}"
                mpc_status = "Platoon_Following"
                print(f"Vehicle {vehicle_id}: Following front car with MPC platoon control")
                
            else:
                # 编队模式但没有前车信息：降级到虚拟leader跟踪
                target_acceleration = self.mpc_solve()
                control_strategy = "MPC_Fallback_VirtualTrack"
                mpc_status = "No_Front_Car"
                print(f"Vehicle {vehicle_id}: No front car data, fallback to virtual leader")
                
        else:
            # 非编队模式：使用MPC控制器跟踪虚拟leader
            target_acceleration = self.mpc_solve()
            control_strategy = "MPC_Solo_VirtualTrack"
            mpc_status = "Success"
            print("Non-platoon mode: Following virtual leader with MPC control")
        
        # 如果MPC求解失败，使用备用控制器
        if abs(target_acceleration) < 1e-6:  # MPC可能求解失败
            print("Warning: MPC may have failed, using backup control")
            target_acceleration, backup_strategy = self.backup_speed_control()
            control_strategy = f"{control_strategy}_Backup"
            mpc_status = "Backup_Control"
        
        return target_acceleration, control_strategy, mpc_status

    def backup_speed_control(self):
        """
        简化的备用速度控制器（低速环境优化，当MPC求解失败时使用）
        
        Returns:
            tuple: (target_acceleration, control_strategy)
        """
        
        # 获取参考速度（限制在低速环境范围内）
        if hasattr(self, 'refSpeed') and self.refSpeed > 0:
            target_speed = min(self.refSpeed, 4.5)  # 限制最大速度4.5m/s
        else:
            target_speed = 3.0  # 默认巡航速度降低到3m/s
        
        # 限制编队模式下的最大速度
        if hasattr(self, 'platoon_state') and self.platoon_state == 1:
            target_speed = min(target_speed, self.max_platoon_speed)
        
        # 低速环境优化的P控制器
        speed_error = target_speed - self.nowSpeed
        
        # 根据速度误差计算加速度（更温和的控制）
        if abs(speed_error) < 0.3:
            # 速度接近目标，使用更小增益
            target_acceleration = speed_error * 0.4
        elif speed_error > 0:
            # 需要加速（更温和）
            target_acceleration = min(speed_error * 0.8, 1.2)  # 降低最大加速度
        else:
            # 需要减速（更温和）
            target_acceleration = max(speed_error * 1.0, -1.8)  # 降低最大减速度
        
        control_strategy = "Backup_Speed_Control_LowSpeed"
        
        return target_acceleration, control_strategy



    # ================================================================== #
    # ==================== 7. 上层 mpc 控制器 ======================= #
    # ================================================================== #
    def cont_vehicle_model(self):
        """定义连续时间车辆模型（三阶积分器）"""
        x = ca.SX.sym('x')  # 弧长位置
        v = ca.SX.sym('y')  # 纵向速度
        a = ca.SX.sym('a')  # 纵向加速度
        states = ca.vertcat(x, v, a)
        states_nums = states.numel()
        
        u = ca.SX.sym('u')
        controls = ca.vertcat(u)
        controls_nums = controls.numel()
        
        tau = 0.2  # 执行器时间常数
        rhs = ca.vertcat(
            v,
            a,
            (u-a)/tau
        )
        
        cont_vehicle_model = ca.Function(
            'cont_model',
            [states, controls],
            [rhs]
        )
        
        return cont_vehicle_model, states_nums, controls_nums
    
    def get_disc_vehicle_model(self):
        """使用RK4方法对车辆模型进行离散化"""
        cont_vehicle_model, states_nums, controls_nums = self.cont_vehicle_model()
        X = ca.SX.sym('X', states_nums)
        U = ca.SX.sym('U', controls_nums)
        dt = self.dt

        # RK4离散化
        k1 = cont_vehicle_model(X, U)
        k2 = cont_vehicle_model(X + dt/2 * k1, U)
        k3 = cont_vehicle_model(X + dt/2 * k2, U)
        k4 = cont_vehicle_model(X + dt * k3, U)
        X_next = X + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        
        disc_vehicle_model = ca.Function(
            'disc_vehicle_model',
            [X, U],
            [X_next]
        )
        
        return disc_vehicle_model, states_nums, controls_nums

    def mpc_setup_solver(self):
        """设置去中心化MPC求解器"""
        try:
            # 1. 获取离散化车辆模型
            disc_vehicle_model, states_nums, controls_nums = self.get_disc_vehicle_model()
            
            # 2. 定义MPC参数        
            N = self.N
            
            # 3. 根据车辆角色和当前状态动态设置权重矩阵
            Q, R = self.get_adaptive_weights()
            
            # 4. 定义优化变量
            X = ca.SX.sym('X', states_nums, N + 1)  # 状态变量
            U = ca.SX.sym('U', controls_nums, N)    # 控制变量
            P = ca.SX.sym('P', states_nums, N + 2)  # 参数：1个初始状态 + N+1个参考状态
            
            # 5. 定义成本函数与约束
            cost = 0
            constraints = []
            
            # 提取初始状态与参考状态
            x_init = P[:, 0]
            x_ref = P[:, 1:]
            
            # 6. 初始状态约束
            constraints.append(X[:, 0] - x_init)
            
            # 7. 构建成本函数
            for k in range(N):
                # 计算状态误差成本
                if self.platoon_state == 1 and self.ego_platoon_ID != 1:
                    # 编队跟随车：使用间距误差成本
                    cost += self.calculate_platoon_cost(X[:, k], x_ref[:, k], Q)
                else:
                    # 非编队或领车：使用标准轨迹跟踪成本
                    state_error = X[:, k] - x_ref[:, k]
                    cost += state_error.T @ Q @ state_error
                
                # 控制成本
                cost += U[:, k].T @ R @ U[:, k]
                
                # 系统动力学约束
                x_next = disc_vehicle_model(X[:, k], U[:, k])
                constraints.append(X[:, k+1] - x_next)
            
            # 8. 终端成本
            Q_f = Q * 1.5  # 终端权重
            if self.platoon_state == 1 and self.ego_platoon_ID != 1:
                cost += self.calculate_platoon_cost(X[:, N], x_ref[:, N], Q_f)
            else:
                state_error = X[:, N] - x_ref[:, N]
                cost += state_error.T @ Q_f @ state_error
            
            # 9. 构建NLP问题
            opt_vars = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))
            opt_params = P
            nlp = {
                'x': opt_vars,
                'p': opt_params,
                'f': cost,
                'g': ca.vertcat(*constraints)
            }
            
            # 10. 配置求解器选项
            opts = {
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.max_iter': 50,  # 实时系统需要快速求解
                'ipopt.tol': 1e-4,
                'ipopt.acceptable_tol': 1e-3,
                'expand': True
            }
            
            # 11. 创建求解器
            solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
            
            # 12. 设置约束边界
            lbg, ubg, lbx, ubx = self.get_constraint_bounds(N, states_nums, controls_nums)
            
            # 13. 包装求解器函数
            def solver_wrapper(x0, p):
                try:
                    sol = solver(
                        x0=x0,
                        p=p,
                        lbx=lbx,
                        ubx=ubx,
                        lbg=lbg,
                        ubg=ubg
                    )
                    return sol
                except Exception as e:
                    print(f"MPC solver error: {e}")
                    return None
            
            self.mpc_solver = solver_wrapper
            print("MPC solver initialized successfully")
            
        except Exception as e:
            print(f"Error setting up MPC solver: {e}")
            self.mpc_solver = None

    def get_adaptive_weights(self):
        """获取自适应权重矩阵"""
        Q = ca.DM.zeros(3, 3)
        R = ca.DM.zeros(1, 1)
        
        if self.platoon_state != 1 or self.ego_platoon_ID == 1:
            # 非编队模式或领车：使用固定权重
            Q[0, 0] = 20.0   # 位置权重
            Q[1, 1] = 50.0   # 速度权重  
            Q[2, 2] = 10.0   # 加速度权重
            R[0, 0] = 1.0    # 控制输入权重
            
        else:
            # 编队跟随车：使用自适应权重
            # 计算当前车间距误差
            current_spacing = self.get_current_spacing()
            spacing_error = abs(current_spacing - self.desired_spacing) if current_spacing > 0 else 1.0
            
            # 基于间距误差自适应调整权重
            if spacing_error > 2.0:
                # 大误差：强化间距跟踪
                Q[0, 0] = min(200.0, 80.0 + spacing_error * 30.0)
                R[0, 0] = max(0.3, 1.0 - spacing_error * 0.2)
            elif spacing_error > 0.8:
                # 中等误差：平衡跟踪和平滑性
                Q[0, 0] = 60.0 + spacing_error * 25.0
                R[0, 0] = 0.8
            else:
                # 小误差：优先平滑性
                Q[0, 0] = 40.0
                R[0, 0] = 1.2
            
            Q[1, 1] = 30.0   # 速度权重保持固定
            Q[2, 2] = 8.0    # 加速度权重保持固定
            
        return Q, R

    def calculate_platoon_cost(self, x_current, x_ref, Q):
        """计算编队控制的成本函数"""
        # 获取前车位置
        front_car_position = self.get_front_car_position()
        
        if front_car_position is not None:
            # 使用实际前车位置计算间距误差
            ego_position = x_current[0]
            actual_distance = front_car_position - ego_position
            distance_error = actual_distance - self.desired_spacing
            
            # 间距误差成本
            spacing_cost = Q[0, 0] * distance_error**2
            
            # 速度和加速度仍使用参考轨迹
            v_error = x_current[1] - x_ref[1]
            a_error = x_current[2] - x_ref[2]
            tracking_cost = Q[1, 1] * v_error**2 + Q[2, 2] * a_error**2
            
            return spacing_cost + tracking_cost
        else:
            # 前车信息不可用，回退到标准轨迹跟踪
            state_error = x_current - x_ref
            return state_error.T @ Q @ state_error

    def get_front_car_position(self):
        """获取前车当前位置"""
        if self.platoon_state != 1 or self.front_car_state is None:
            return None
        
        # 将前车GPS坐标投影到弧长坐标
        _, front_arc_pos = self.project_point_to_trajectory(
            self.front_car_state.latitude,
            self.front_car_state.longitude
        )
        return front_arc_pos

    def get_current_spacing(self):
        """获取当前车间距"""
        front_pos = self.get_front_car_position()
        if front_pos is not None and self.current_trajectory_position > 0:
            return front_pos - self.current_trajectory_position
        return -1.0

    def get_constraint_bounds(self, N, states_nums, controls_nums):
        """设置约束边界"""
        # 计算约束数量
        num_constraints = states_nums + N * states_nums  # 初始约束 + 动力学约束
        
        # 等式约束边界（都为0）
        lbg = ca.DM.zeros(num_constraints)
        ubg = ca.DM.zeros(num_constraints)
        
        # 变量边界
        total_vars = (N + 1) * states_nums + N * controls_nums
        lbx = -ca.inf * ca.DM.ones(total_vars)
        ubx = ca.inf * ca.DM.ones(total_vars)
        
        # 状态约束
        v_min, v_max = 0.0, 5.0  # 速度范围 [m/s]
        a_min, a_max = -3.0, 1.5  # 加速度范围 [m/s²]
        
        # 控制约束（jerk约束）
        if self.platoon_state == 1 and self.ego_platoon_ID != 1:
            j_min, j_max = -1.5, 1.5  # 编队跟随车允许更大的jerk
        else:
            j_min, j_max = -2.0, 2.0  # 领车或非编队车辆相对保守
        
        # 应用状态约束
        for k in range(N + 1):
            idx = k * states_nums
            # 位置无约束
            # 速度约束
            lbx[idx + 1] = v_min
            ubx[idx + 1] = v_max
            # 加速度约束
            lbx[idx + 2] = a_min
            ubx[idx + 2] = a_max
        
        # 应用控制约束
        control_start = (N + 1) * states_nums
        for k in range(N):
            idx = control_start + k
            lbx[idx] = j_min
            ubx[idx] = j_max
        
        return lbg, ubg, lbx, ubx

    def mpc_solve(self):
        """
        求解MPC优化问题（去中心化版本）
        """
        if not hasattr(self, 'mpc_solver') or self.mpc_solver is None:
            print("Warning: MPC solver not initialized")
            return 0.0
        
        try:
            # 1. 获取当前状态
            current_states = self.get_current_mpc_states()
            if current_states is None:
                return 0.0
            
            # 2. 获取参考轨迹
            ref_traj = self.get_mpc_reference_trajectory()
            if ref_traj is None:
                return 0.0
            
            # 3. 构建初始状态向量
            x_now = ca.DM([
                float(current_states['x']),
                float(current_states['v']),
                float(current_states['a'])
            ])
            
            # 4. 构建参数向量
            p = ca.horzcat(x_now, ref_traj)
            
            # 5. 构建初始猜测
            x0 = self.generate_initial_guess(ref_traj)
            
            # 6. 求解MPC问题
            solution = self.mpc_solver(x0=x0, p=p)
            
            # 7. 提取控制结果
            if solution is not None and 'x' in solution:
                u_opt = solution['x']
                states_vars = 3 * (self.N + 1)
                u_raw = float(u_opt[states_vars])
                
                # 控制量限幅
                u = max(min(u_raw, 2.0), -2.5)
                
                # 调试信息
                if abs(u_raw) > 1.8:
                    print(f"Vehicle {self.ego_platoon_ID}: Control saturation detected: raw={u_raw:.3f}, clipped={u:.3f}")
                
                return u
            else:
                print("Warning: MPC solving failed - no solution found")
                return 0.0
                
        except Exception as e:
            print(f"Error in MPC solving: {e}")
            return 0.0

    def get_current_mpc_states(self):
        """获取当前MPC状态"""
        if not self.GPS_state or self.current_trajectory_position <= 0:
            return None
        
        return {
            'x': self.current_trajectory_position,  # 弧长位置
            'v': self.current_velocity if hasattr(self, 'current_velocity') else self.nowSpeed,
            'a': self.nowAcceleration
        }

    def get_mpc_reference_trajectory(self):
        """获取MPC参考轨迹（仅纵向控制）"""
        # 使用虚拟leader参考（仅纵向控制相关）
        ref_arc_lengths, ref_velocities, ref_accelerations = self.get_virtual_leader()
        
        if ref_arc_lengths is None:
            return None
        
        # 构建参考轨迹矩阵（仅包含纵向状态）
        ref_traj = ca.DM.zeros(3, self.N + 1)
        for k in range(min(self.N + 1, len(ref_arc_lengths))):
            ref_traj[0, k] = ref_arc_lengths[k]   # 弧长位置
            ref_traj[1, k] = ref_velocities[k]    # 速度
            ref_traj[2, k] = ref_accelerations[k] # 加速度
        
        return ref_traj

    def generate_initial_guess(self, ref_traj):
        """生成MPC初始猜测"""
        states_vars = 3 * (self.N + 1)
        total_vars = states_vars + self.N
        x0 = ca.DM.zeros(total_vars, 1)
        
        # 状态变量初始猜测（基于参考轨迹）
        for k in range(self.N + 1):
            x0[k*3:(k+1)*3] = ref_traj[:, k]
        
        # 控制变量初始猜测（小的jerk值）
        for k in range(self.N):
            x0[states_vars + k] = 0.0
        
        return x0

    # ================================================================== #
    # ==================== 8. 下层 pid 控制器 ======================= #
    # ================================================================== #

    def acceleration_to_throttle_brake(self, target_acc, steer_cmd):
        """
        将目标加速度转换为油门/刹车控制指令（基于提供的PID控制逻辑优化）
        
        Args:
            target_acc (float): 由mpc的输出获得，目标加速度 [m/s²]
            steer_cmd (float): 由横向控制器的输出获得，转向指令 [rad]
            
        Returns:
            dict: {'throttle': float, 'brake': float, 'steer': float}
        """
        # 获取当前状态
        current_acc = self.nowAcceleration if self.nowAcceleration is not None else 0.0
        current_speed = self.nowSpeed if self.nowSpeed is not None else 0.0
        
        # 检查数据有效性
        if current_speed is None or current_acc is None:
            return {'throttle': 0.0, 'brake': 0.0, 'steer': steer_cmd}
        
        # 1. 计算加速度误差
        acc_error = target_acc - current_acc
        
        # 2. PID计算控制力度 + 前馈控制
        pid_output = self.lower_pid.update(acc_error)
        feedforward = target_acc * 0.1  # 前馈增益
        output = pid_output + feedforward
        
        # 3. 映射到throttle和brake - 智能分层控制策略
        control_cmd = {'throttle': 0.0, 'brake': 0.0, 'steer': steer_cmd}
        speed_ref = self.speed_ref 
        
        # 计算速度误差，决定是否真的需要刹车
        speed_error = current_speed - speed_ref
        overspeed_threshold = 0.5  # 超速阈值：只有显著超速时才考虑刹车
        
        # 4. 分层控制策略
        if output > 0.1:
            # 明确需要加速
            control_cmd['throttle'] = min(output, 1.0)
            control_cmd['brake'] = 0.0
            
        elif output < -0.4 and speed_error > overspeed_threshold:
            # 强烈需要减速 且 确实超速较多 - 使用轻刹车
            control_cmd['throttle'] = 0.0
            # 根据超速程度调节刹车强度
            overspeed_factor = min(speed_error / 1.0, 1.0)  # 最大超速1m/s对应最大刹车
            brake_intensity = min(abs(output) * 0.2 * overspeed_factor, 0.2)
            control_cmd['brake'] = brake_intensity
            
        elif output < -0.25 and speed_error > 0.8:
            # 中等需要减速 且 超速较多 - 使用很轻的刹车
            control_cmd['throttle'] = 0.0
            brake_intensity = min(abs(output) * 0.1, 0.1)  # 很轻的刹车
            control_cmd['brake'] = brake_intensity
            
        else:
            # 其他情况 - 仅使用油门调节（更温和）
            control_cmd['brake'] = 0.0
            
            # 智能油门控制
            if speed_error > 1.0:  # 显著超速
                control_cmd['throttle'] = 0.0  # 完全断油
            elif speed_error > 0.5:  # 轻微超速
                # 线性减少油门
                control_cmd['throttle'] = max(0.0, 0.2 * (1.0 - speed_error))
            elif speed_error > 0.0:  # 微超速
                control_cmd['throttle'] = 0.15 
            else:  # 速度不足
                # 根据速度缺口给油门
                speed_deficit = -speed_error
                if speed_deficit > 0.8:
                    control_cmd['throttle'] = 0.4
                elif speed_deficit > 0.4:
                    control_cmd['throttle'] = 0.3
                elif speed_deficit > 0.2:
                    control_cmd['throttle'] = 0.25
                else:
                    control_cmd['throttle'] = 0.2
        
        # 5. 安全限制（保持原有的SUV限制）
        control_cmd['throttle'] = max(0.0, min(control_cmd['throttle'], 0.8))  # SUV油门限制
        control_cmd['brake'] = max(0.0, min(control_cmd['brake'], 0.4))        # SUV刹车限制
        
        return control_cmd



    # ================================================================== #
    # ==================== 9. HMI & V2X  ======================= #
    # ================================================================== #


    def sub_callback_platoon_state(self, msgHmiPlatoonState:HmiPlatoonInterface):
        """订阅编队状态（优化版）"""
        old_state = self.platoon_state
        old_id = self.ego_platoon_ID
        
        self.platoon_state = msgHmiPlatoonState.platoon_state
        self.ego_platoon_ID = msgHmiPlatoonState.platoon_id
        
        print("--------get_hmi_msg-------------")
        print("platoon_state =", self.platoon_state)
        print("ego_platoon_ID =", self.ego_platoon_ID)
        
        # 状态变化时重置相关参数
        if old_state != self.platoon_state or old_id != self.ego_platoon_ID:
            print(f"=== Platoon State Changed ===")
            print(f"State: {old_state} -> {self.platoon_state}")
            print(f"ID: {old_id} -> {self.ego_platoon_ID}")
            
            # 重置前车信息
            if self.platoon_state == 0:
                self.front_car_state = None
                print("Exited platoon mode, clearing front car data")
            

    def sub_callback_vtx(self, msgVtx:VtxInterface):
        """
        订阅V2X数据，更新前车信息
        编队策略：1号车跟虚拟leader，2号车跟1号车，3号车跟2号车
        """
        print("-----get vtx data---------")
        
        # 存储所有V2X车辆信息
        if hasattr(msgVtx, 'platoon_id'):
            self.v2x_vehicles[msgVtx.platoon_id] = msgVtx
            print(f"Received V2X data from vehicle ID: {msgVtx.platoon_id}")
        
        # 非编队模式：清除前车信息
        if self.platoon_state != 1:
            self.front_car_state = None 
            print("Non-platoon mode: No front car tracking")
            return
        
        # 编队模式：根据三车编队策略确定前车
        expected_front_car_id = self.get_platoon_front_car_id()
        
        if expected_front_car_id is None:
            # 1号车：没有前车，跟随虚拟leader
            self.front_car_state = None
            print(f"Vehicle {self.ego_platoon_ID}: Following virtual leader (no front car)")
        elif msgVtx.platoon_id == expected_front_car_id:
            # 收到期望前车的V2X数据
            self.front_car_state = msgVtx

            if hasattr(msgVtx, 'speed'):
                print(f"Front car speed: {msgVtx.speed:.2f} m/s")
            
            # 计算并显示当前车间距
            if self.nowLatitude != 0 and self.nowLongitude != 0:
                current_spacing = self.calculate_distance_between_vehicles(
                    msgVtx.latitude, msgVtx.longitude,
                    self.nowLatitude, self.nowLongitude
                )
                if current_spacing > 0:
                    spacing_status = "OK"
                    if current_spacing > self.max_spacing:
                        spacing_status = "TOO LARGE"
                    elif current_spacing < self.min_spacing:
                        spacing_status = "TOO SMALL"
                    print(f"Current spacing: {current_spacing:.2f}m [{spacing_status}]")
                    print(f"Target range: {self.min_spacing:.1f} - {self.max_spacing:.1f}m")
        else:
            # 收到其他车辆的V2X数据，不更新前车状态
            print(f"Vehicle {self.ego_platoon_ID}: Received data from Vehicle {msgVtx.platoon_id} "
                  f"(expected front car: {expected_front_car_id})")
        

    def get_platoon_front_car_id(self):
        """
        根据当前车辆ID确定前车ID
        编队策略：1号车跟虚拟leader，2号车跟1号车，3号车跟2号车
        """
        if self.platoon_state != 1:
            return None  # 非编队模式
        
        if self.ego_platoon_ID == 1:
            return None  # 1号车没有前车，跟随虚拟leader
        elif self.ego_platoon_ID == 2:
            return 1     # 2号车跟随1号车
        elif self.ego_platoon_ID == 3:
            return 2     # 3号车跟随2号车
        else:
            return None  # 未知车辆ID
    
    # ================================================================== #
    # ==================== 10.  scripts   ======================= #
    # ================================================================== #

    def validate_gps_coordinates(self, lat, lon):
        """验证GPS坐标的有效性"""
        # 检查坐标范围
        if abs(lat) < 1e-6 or abs(lon) < 1e-6:
            return False
        if abs(lat) > 90 or abs(lon) > 180:
            return False
            
        # 检查与上一个有效位置的距离（避免跳变）
        if self.last_valid_position[0] != 0:
            lat_diff = abs(lat - self.last_valid_position[0])
            lon_diff = abs(lon - self.last_valid_position[1])
            # 如果位置跳变超过0.001度（约111米），认为无效
            if lat_diff > 0.001 or lon_diff > 0.001:
                return False
                
        return True

    def build_trajectory_distance_map(self):
        """构建参考轨迹的累积距离映射"""
        if not self.reference_trajectory or len(self.reference_trajectory) < 2:
            self.trajectory_available = False
            print("Warning: Invalid reference trajectory")
            return
        
        self.trajectory_distances = [0.0]
        total_distance = 0.0
        
        for i in range(1, len(self.reference_trajectory)):
            lat1, lon1 = self.reference_trajectory[i-1]
            lat2, lon2 = self.reference_trajectory[i]
            
            # 验证坐标有效性
            if not (self.validate_gps_coordinates(lat1, lon1) and 
                   self.validate_gps_coordinates(lat2, lon2)):
                continue
            
            try:
                pos1_x, pos1_y, _ = self.conversion_of_coordinates(lat1, lon1, 0)
                pos2_x, pos2_y, _ = self.conversion_of_coordinates(lat2, lon2, 0)
                
                segment_distance = sqrt((pos2_x - pos1_x)**2 + (pos2_y - pos1_y)**2)
                
                # 异常段长度检测
                if segment_distance > 10.0:  # 段长度超过10米认为异常
                    print(f"Warning: Abnormal segment distance {segment_distance:.2f}m at index {i}")
                    segment_distance = 0.0
                    
                total_distance += segment_distance
                self.trajectory_distances.append(total_distance)
                
            except Exception as e:
                print(f"Error calculating segment distance: {e}")
                self.trajectory_distances.append(total_distance)
        
        if total_distance > 0:
            self.trajectory_available = True
            self.trajectory_update_counter += 1
            print(f"Trajectory distance map built: {len(self.trajectory_distances)} points, total length: {total_distance:.2f}m")
        else:
            self.trajectory_available = False
            print("Warning: Failed to build valid trajectory distance map")

    def project_point_to_trajectory(self, latitude, longitude):
        """将GPS点投影到参考轨迹上，返回最近轨迹点的索引和在轨迹上的弧长位置"""
        if not self.trajectory_available:
            return None, 0.0
            
        if not self.validate_gps_coordinates(latitude, longitude):
            print(f"Warning: Invalid GPS coordinates for projection: ({latitude}, {longitude})")
            return None, self.current_trajectory_position  # 返回上一个有效位置
        
        try:
            target_x, target_y, _ = self.conversion_of_coordinates(latitude, longitude, 0)
            
            min_distance = float('inf')
            closest_index = 0
            
            # 修复：搜索整个轨迹以找到最近点
            for i in range(len(self.reference_trajectory)):
                lat, lon = self.reference_trajectory[i]
                if not self.validate_gps_coordinates(lat, lon):
                    continue
                    
                traj_x, traj_y, _ = self.conversion_of_coordinates(lat, lon, 0)
                distance = sqrt((target_x - traj_x)**2 + (target_y - traj_y)**2)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i
            
            # 异常检测：如果最近点距离过大，使用上一个有效位置
            if min_distance > 50.0:  # 50米以上认为偏离轨迹
                print(f"Warning: Vehicle too far from trajectory ({min_distance:.2f}m)")
                return None, self.current_trajectory_position
            
            arc_length_position = self.trajectory_distances[closest_index]
            
            # 平滑处理：限制位置变化率（只在已有有效位置时进行平滑）
            if self.current_trajectory_position > 0 and abs(arc_length_position - self.current_trajectory_position) > 10.0:
                # 位置跳变过大，进行平滑
                alpha = 0.3  # 平滑系数
                arc_length_position = alpha * arc_length_position + (1 - alpha) * self.current_trajectory_position
            
            return closest_index, arc_length_position
            
        except Exception as e:
            print(f"Error in trajectory projection: {e}")
            return None, self.current_trajectory_position

    def calculate_distance_between_vehicles(self, front_lat, front_lon, ego_lat, ego_lon):
        """
        计算两车之间的距离 (前车到本车的距离，始终返回正值)
        用于三车编队控制的车间距计算
        """
        # 验证输入坐标
        if not (self.validate_gps_coordinates(front_lat, front_lon) and 
               self.validate_gps_coordinates(ego_lat, ego_lon)):
            print("Warning: Invalid coordinates for distance calculation")
            return -1.0
        
        try:
            # 始终使用欧几里得距离（更准确）
            front_x, front_y, _ = self.conversion_of_coordinates(front_lat, front_lon, 0)
            ego_x, ego_y, _ = self.conversion_of_coordinates(ego_lat, ego_lon, 0)
            distance = sqrt((front_x - ego_x)**2 + (front_y - ego_y)**2)
            
            # 如果有轨迹信息，也可以计算弧长距离进行比较
            if self.trajectory_available and self.distance_calc_debug:
                _, front_arc_pos = self.project_point_to_trajectory(front_lat, front_lon)
                _, ego_arc_pos = self.project_point_to_trajectory(ego_lat, ego_lon)
                
                if front_arc_pos is not None and ego_arc_pos is not None:
                    arc_distance = abs(front_arc_pos - ego_arc_pos)
                    print(f"Debug - Euclidean distance: {distance:.2f}m, Arc distance: {arc_distance:.2f}m")
                    # 使用两者中较小的值（更保守的估计）
                    distance = min(distance, arc_distance)
            
            # 异常值检测
            if distance > 1000.0:  # 距离超过1公里认为异常
                print(f"Warning: Abnormal vehicle distance {distance:.2f}m")
                return -1.0
            
            return distance
            
        except Exception as e:
            print(f"Error calculating vehicle distance: {e}")
            return -1.0

    # 坐标转换函数,经纬高LLA 转 东北天ENU
    def conversion_of_coordinates(self, conversionLatitude, conversionLongitude, conversionAltitude):
        """坐标转换函数,经纬高LLA 转 东北天ENU"""
        
        # WGS84 参数定义
        wgs84_a  = 6378137.0
        wgs84_f  = 1/298.257223565
        wgs84_e2 = wgs84_f * (2-wgs84_f)
        D2R      = PI / 180.0

        # 局部坐标原点经纬度
        Latitude0 = 36.65538784
        Longitude0 = 114.58287107

        lat0 = Latitude0  *D2R
        lon0 = Longitude0 *D2R
        alt0 = 0
        N0 = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat0)*sin(lat0))
        x0 = (N0 + alt0) * cos(lat0) * cos(lon0)
        y0 = (N0 + alt0) * cos(lat0) * sin(lon0)
        z0 = (N0*(1-wgs84_e2) + alt0) * sin(lat0)

        lat = conversionLatitude  *D2R
        lon = conversionLongitude *D2R
        alt = conversionAltitude
        N = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat)*sin(lat))
        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N*(1-wgs84_e2) + alt) * sin(lat)

        dx = x - x0
        dy = y - y0
        dz = z - z0

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
    
    def __del__(self):
        """析构函数：保存剩余数据"""
        try:
            # 保存CSV缓存中的剩余数据
            if hasattr(self, 'csv_buffer') and self.csv_buffer:
                with open(self.csv_filename, 'a', newline='', encoding='utf-8') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(self.csv_buffer)
                print(f"Saved remaining {len(self.csv_buffer)} data points to CSV")
            
            # 关闭调试文件
            if hasattr(self, 'saveData'):
                self.saveData.close()
                
        except Exception as e:
            print(f"Error in destructor: {e}")

def main():
    rclpy.init()
    try:
        rosNode = YMpc(name='pid')
        print("Node initialized successfully")
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Shutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
