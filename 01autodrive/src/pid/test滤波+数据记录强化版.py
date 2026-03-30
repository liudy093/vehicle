


import sys
import os
import rclpy
from rclpy.node import Node
import time
from car_interfaces.msg import (
    PidInterface,
    LocalPathPlanningInterface,
    FusionInterface,
    HmiPlatoonInterface,
    VtxInterface
)

import casadi as ca
import numpy as np
from math import *
import csv
import yaml
from easydict import EasyDict
from datetime import datetime
import bisect

PI = 3.1415926535

ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class YMpc(Node):
    def __init__(self, name):
        super().__init__(name)
    
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

        
        self.ego_platoon_ID = 1  # 默认车辆ID为1

        # 初始化当前车辆状态
        self.nowSpeed = 0.0
        self.nowAcceleration = 0.0
        self.nowLatitude = 0.0
        self.nowLongitude = 0.0
        self.nowArcLength = 0.0  # 当前弧长位置
        self.nowYaw = 0.0


        # 初始化参考车辆状态 
        self.refSpeed = 0.0
        self.refAcceleration = 0.0      
        self.refLatitude = 0.0
        self.refLongitude = 0.0
        self.refArcLength = 0.0  # 参考弧长位置
        self.refYaw = 0.0
        
        # 初始化前车状态
        self.frontSpeed = 0.0
        self.frontAcceleration = 0.0
        self.frontLatitude = 0.0
        self.frontLongitude = 0.0
        self.frontArcLength = 0.0  # 前车弧长位置
        
        # 初始化轨迹相关变量
        self.trajectoryX = []  # 轨迹X坐标列表
        self.trajectoryY = []  # 轨迹Y坐标列表
        self.trajectoryCumulativeArcLength = []  # 轨迹累积弧长列表
        self.trajectory_available = False  # 轨迹是否可用
        self.trajectory_update_counter = 0  # 轨迹更新计数器
        
        # 弧长计算相关
        self.last_valid_arc_length = 0.0  # 上一次有效的弧长位置
        self.arc_length_smooth_alpha = 0.3  # 弧长平滑系数
        

        # MPC
        self.dt = 0.1  # 时间间隔
        self.N = 10  # 预测步长
        
        # 控制参数
        self.desired_spacing = 8.0     # 期望车间距
        self.min_spacing = 5.0         # 安全间距 (m)
        self.max_spacing = 15.0        # 最大间距 (m)
        self.max_platoon_speed = 4.0   # 编队最大速度限制 (m/s)
        
        # GPS和编队状态
        self.GPS_state = False  # GPS状态
        self.platoon_state = 0  # 编队状态：0-非编队，1-编队模式
        
        # 横向控制相关
        self.control_value_last = 0.0  # 上次横向控制值
        self.turn_signals = 0.0  # 转向灯状态
        
        # 到达状态
        self.arrived = False
        
        # 制动距离（用于停车控制）
        self.braking_distance = 0.0
        
        # ============ 传感器状态滤波器参数 ============
        # 滤波器状态字典：{'last_value': float, 'initialized': bool, 'alpha': float}
        self.speed_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.3}
        self.accel_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.5}
        self.yaw_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.4}
        
        # V2X前车状态滤波器
        self.front_speed_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.6}
        self.front_accel_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.6}
        self.front_arc_filter = {'last_value': 0.0, 'initialized': False, 'alpha': 0.5}
        
        
        # ============ 数据记录相关初始化 ============
        self.data_recording_enabled = True  # 是否启用数据记录
        self.data_buffer = []  # 车辆状态数据缓冲区
        self.vleader_buffer = []  # 虚拟leader数据缓冲区
        self.buffer_size = 10  # 缓冲区大小（每10条写入一次）
        
        # 时间记录
        self.recording_start_time = None  # 记录开始时间
        self.sample_count = 0  # 采样计数器
        
        # CSV文件路径
        self.csv_filename = None
        self.vleader_csv_filename = None
        self.csv_initialized = False
        
        # 虚拟leader状态（从mpc_callback中获取）
        self.vleader_arc_position = 0.0
        self.vleader_velocity = 0.0
        self.vleader_acceleration = 0.0
        
        # 控制指令（从mpc_callback中获取）
        self.last_throttle_percentage = 0
        self.last_brake_percentage = 0
        self.last_gear = 3
        
        # 创建数据目录
        self.data_dir = os.path.expanduser('~/AutoDrive/src/pid/data')
        if not os.path.exists(self.data_dir):
            try:
                os.makedirs(self.data_dir)
                print(f"Created data directory: {self.data_dir}")
            except Exception as e:
                print(f"Warning: Could not create data directory: {e}")
                self.data_recording_enabled = False
        
        # 创建数据记录定时器（10Hz，与mpc_callback同步）
        if self.data_recording_enabled:
            self.timer_data_record = self.create_timer(0.1, self.data_record)
        
        # pid参数
        self.declare_parameter('pid_kp', 0.6)    # 提高比例增益
        self.declare_parameter('pid_ki', 0.0)   # 适度提高积分增益
        self.declare_parameter('pid_kd', 0.05)   # 添加微分增益抑制震荡
        self.pid_kp = self.get_parameter('pid_kp').get_parameter_value().double_value
        self.pid_ki = self.get_parameter('pid_ki').get_parameter_value().double_value
        self.pid_kd = self.get_parameter('pid_kd').get_parameter_value().double_value
        
        # 初始化下层PID控制器属性
        self.pid_min_out = -1.0
        self.pid_max_out = 1.0
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_filtered_d = 0.0  # 上一次滤波后的微分项
        self.pid_alpha = 0.4  # 针对微分低通滤波系数
        self.pid_dead_zone = 0.05  # 死区范围

        # 存储最近的路径规划消息
        self.last_path_planning_msg = None
        
        # 前车状态对象
        self.front_car_state = None
        
        # 初始化MPC求解器
        self.mpc_solver = None

        
    def mpc_callback(self):
        
        msgYmpc = PidInterface()
        msgYmpc.timestamp = time.time()

        nowPosX, nowPosY, _ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)
        refPosX, refPosY, _ = self.conversion_of_coordinates(self.refLatitude, self.refLongitude, 0)

        #####################################横向控制#########################################################
 
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

        # 添加弧长位置信息
        msgYmpc.arc_position = self.nowArcLength

        ###################################纵向编队控制####################################################

        # ============ 1. 停车逻辑判断（优先级最高） ============
        if self.refSpeed < 0.05:  # 参考速度接近0，说明到达终点
            # 重置制动距离（正常行驶时清零）
            if self.nowSpeed < 0.1:
                # 车速已经很低，保持停稳状态
                msgYmpc.throttle_percentage = 0
                msgYmpc.braking_percentage = 30  # 固定轻刹车保持停稳
                msgYmpc.acceleration = 0.0
                print("=== Stopped at destination ===")
            else:
                # 车速还较高，执行开环制动
                self.braking_distance = self.braking_distance + self.nowSpeed * 0.1
                msgYmpc.throttle_percentage = 0
                
                # 从yaml配置读取制动参数，如果没有则使用默认值
                dec_open_loop = self.yaml_data.get('dec_open_loop', 5.0) if hasattr(self, 'yaml_data') else 5.0
                dec_open_loop_start = self.yaml_data.get('dec_open_loop_start', 20) if hasattr(self, 'yaml_data') else 20
                
                refBraking = int(dec_open_loop * self.braking_distance + dec_open_loop_start)
                refBraking = max(0, min(125, refBraking))  # 限幅 [0, 125]
                
                msgYmpc.braking_percentage = refBraking
                msgYmpc.acceleration = -2.0  # 显示减速状态
                print(f"=== Stopping: Brake={refBraking}%, Distance={self.braking_distance:.2f}m ===")
        
        # ============ 2. 正常行驶控制 ============
        else:
            # 清零制动距离（非停车状态）
            self.braking_distance = 0.0
            
            # 检查轨迹和GPS状态
            if not hasattr(self, 'GPS_state') or not self.GPS_state or not self.trajectory_available:
                # 轨迹或GPS不可用：安全停车
                target_acceleration = -1.0 if self.nowSpeed > 0.5 else 0.0
                print("Warning: Trajectory or GPS unavailable, safety stop")
            else:
                # ============ 3. 检查前车数据时效性（编队模式） ============
                if self.ego_platoon_ID in [2, 3]:
                    if self.front_car_state is not None:
                        current_time = time.time()
                        # 检查V2X数据是否超时（2秒）
                        if hasattr(self, 'front_car_update_time'):
                            if current_time - self.front_car_update_time > 2.0:
                                print(f"Warning: Front car data outdated for vehicle {self.ego_platoon_ID}")
                                self.front_car_state = None
                                self.frontArcLength = 0.0
                    
                    # 如果是跟随车但没有前车数据，降级为单车模式
                    if self.frontArcLength <= 0:
                        self.ego_platoon_ID = 1
                        print(f"Vehicle {self.ego_platoon_ID}: No valid front car data, operating in solo mode")
                
                # ============ 4. MPC求解获取目标加速度 ============
                try:
                    target_acceleration = self.mpc_solve()

                    # 安全限幅
                    target_acceleration = max(-2.5, min(1.5, target_acceleration))

                except Exception as e:
                    print(f"Error in MPC solving: {e}")
                    # 降级到简单的速度控制
                    speed_error = self.refSpeed - self.nowSpeed
                    target_acceleration = max(-1.5, min(1.0, speed_error * 0.4))
            
            # ============ 5. 下层PID控制器：加速度转控制指令 ============
            control_cmd = self.acc_to_control_cmd(target_acceleration, controlValue)
            
            # 提取油门和刹车值并限幅
            throttle_value = max(0.0, min(1.0, control_cmd['throttle']))
            brake_value = max(0.0, min(1.0, control_cmd['brake']))
            
            # 转换为百分比（0-100）
            msgYmpc.throttle_percentage = int(round(throttle_value * 100))
            msgYmpc.braking_percentage = int(round(brake_value * 100))
            msgYmpc.acceleration = target_acceleration
        
        # 保存控制指令供数据记录使用
        self.last_throttle_percentage = msgYmpc.throttle_percentage
        self.last_brake_percentage = msgYmpc.braking_percentage
        
        # ============ 5. 设置档位和转向灯 ============
        if self.arrived:
            msgYmpc.gear = 2
        else:
            msgYmpc.gear = 3
        
        self.last_gear = msgYmpc.gear
        msgYmpc.turn_signals = self.turn_signals
        
        # ============ 6. 发布控制指令 ============
        self.pub_Y_Mpc.publish(msgYmpc)
        
        # 定期输出控制信息
        if not hasattr(self, '_control_counter'):
            self._control_counter = 0
        self._control_counter += 1
        
        if self._control_counter % 10 == 0:  # 每1秒输出一次
            print(f"=== Control Output (Vehicle {self.ego_platoon_ID}) ===")
            print(f"Speed: {self.nowSpeed:.2f} m/s, Accel: {target_acceleration:.3f} m/s²")
            print(f"Throttle: {msgYmpc.throttle_percentage}%, Brake: {msgYmpc.braking_percentage}%")
            if self.ego_platoon_ID in [2, 3] and self.frontArcLength > 0:
                spacing = self.frontArcLength - self.nowArcLength
                print(f"Front spacing: {spacing:.2f}m (Target: {self.desired_spacing}m)")
    
    
    # 获取nowarclength,nowspeed,nowacceleration
    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        订阅融合数据（带滤波）
        接收传感器数据后立即滤波，滤波值直接赋给状态变量
        """
        # ============ 1. 接收原始数据并滤波 ============
        raw_speed = msgFusion.carspeed
        raw_accel = msgFusion.ax
        raw_yaw = msgFusion.yaw / 180 * PI  # 转换为弧度
        
        # 应用低通滤波器（滤波后直接赋值）
        self.nowSpeed = self.low_pass_filter(raw_speed, self.speed_filter)
        self.nowAcceleration = self.low_pass_filter(raw_accel, self.accel_filter)
        self.nowYaw = self.angle_low_pass_filter(raw_yaw, self.yaw_filter)
        
        # ============ 2. GPS数据（通常已由融合模块处理，这里直接使用） ============
        self.nowLatitude = msgFusion.latitude
        self.nowLongitude = msgFusion.longitude
        
        # GPS状态检查
        if abs(self.nowLatitude) < 1 and abs(self.nowLongitude) < 1:
            self.GPS_state = False
        else:
            self.GPS_state = True 

        # ============ 3. 其他状态 ============
        self.nowPitch = msgFusion.pitch
        self.nowX, self.nowY, _ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)

        # 计算弧长（内部已有平滑处理）
        self.nowArcLength = self.calculate_arc_length(self.nowLatitude, self.nowLongitude)

    # 获取refarclength,refspeed,refacceleration
    def sub_callback_local_path_planning(self, msgLocalPathPlanning:LocalPathPlanningInterface):
        """订阅局部路径规划数据"""
        if msgLocalPathPlanning.latitude is None or len(msgLocalPathPlanning.latitude) == 0:
            self.trajectory_state = False
            return
        
        # 更新完整轨迹的弧长信息
        self.update_trajectory_arc_length(msgLocalPathPlanning.latitude, msgLocalPathPlanning.longitude)
        
        self.refSpeed = msgLocalPathPlanning.speed[0]
        self.refAcceleration = msgLocalPathPlanning.acceleration[0]
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
        
        # 计算参考点的弧长位置
        self.refArcLength = self.calculate_arc_length(self.refLatitude, self.refLongitude)

        if len(msgLocalPathPlanning.latitude) == 500:
            self.trajectory_state = True

            if abs(msgLocalPathPlanning.latitude[1] - msgLocalPathPlanning.latitude[-1]) < 1e-8 and \
                abs(msgLocalPathPlanning.longitude[1] - msgLocalPathPlanning.longitude[-1]) < 1e-8 and \
                self.nowSpeed < 0.001:
                self.arrived = True
            else:
                self.arrived = False
        else:
            self.trajectory_state = False
            self.trajectory_available = False

    def get_virtual_leader(self):
        """
        车辆id默认设置为1，如果在编队状态，则车辆id=self.ego_platoon_ID
         通过车辆id判断：
            如果车辆id为1；则跟随虚拟leader
            如果车辆id为2，3；则跟随前车（使用固定期望车间距）
        """
        N = self.N
        dt = self.dt
        
        # 生成预测时域内的参考序列
        ref_arc_lengths = []
        ref_velocities = []
        ref_accelerations = []
        
        # 获取当前弧长位置（使用calculate_arc_length作为投影）
        current_arc_length = self.calculate_arc_length(self.nowLatitude, self.nowLongitude)
        
        # 计算当前间距（使用弧长差作为沿轨迹间距估计）
        current_spacing = None
        if self.ego_platoon_ID in [2, 3] and self.frontArcLength is not None:
            current_spacing = self.frontArcLength - current_arc_length
        
        for k in range(N):
            try:
                if self.ego_platoon_ID == 1:
                    # 1号车：跟随虚拟leader，使用参考弧长预测
                    predicted_arc_length = self.refArcLength + (self.refSpeed * (k * dt) + 0.5 * self.refAcceleration * (k * dt)**2)
                else:
                    # 2号或3号车：跟随前车，使用固定期望车间距
                    # 前车未来弧长预测
                    front_future_arc = self.frontArcLength + (self.frontSpeed * (k * dt) + 0.5 * self.frontAcceleration * (k * dt)**2)
                    
                    # 使用固定期望车间距
                    predicted_arc_length = front_future_arc - self.desired_spacing
                
                ref_arc_lengths.append(max(0.0, predicted_arc_length))
                
                # 基于弧长查找轨迹索引并插值速度/加速度（从局部路径规划）
                trajectory_idx = self.find_trajectory_index_by_arc_length(predicted_arc_length)
                
                if trajectory_idx is not None and self.last_path_planning_msg is not None and trajectory_idx < len(self.last_path_planning_msg.latitude):
                    # 验证轨迹点
                    traj_lat = self.last_path_planning_msg.latitude[trajectory_idx]
                    traj_lon = self.last_path_planning_msg.longitude[trajectory_idx]
                    
                    if self.validate_gps_coordinates(traj_lat, traj_lon):
                        # 速度插值（保持来自路径规划）
                        if trajectory_idx < len(self.last_path_planning_msg.speed):
                            ref_speed = self.last_path_planning_msg.speed[trajectory_idx]
                        else:
                            ref_speed = self.refSpeed
                        
                        # 限制编队速度
                        ref_speed = min(ref_speed, self.max_platoon_speed)
                        
                        ref_velocities.append(max(0.1, ref_speed))  # 避免负速
                        
                        # 加速度插值（保持来自路径规划）
                        if trajectory_idx < len(self.last_path_planning_msg.acceleration):
                            ref_accel = self.last_path_planning_msg.acceleration[trajectory_idx]
                        else:
                            ref_accel = self.refAcceleration
                        
                        ref_accelerations.append(ref_accel)
                    else:
                        # 无效坐标，使用安全默认
                        safe_speed = min(self.refSpeed, self.max_platoon_speed)
                        ref_velocities.append(safe_speed)
                        ref_accelerations.append(0.0)
                else:
                    # 超出轨迹，使用安全默认
                    safe_speed = min(self.refSpeed, self.max_platoon_speed)
                    ref_velocities.append(safe_speed)
                    ref_accelerations.append(0.0)
                    
            except Exception as e:
                print(f"Error generating reference at step {k}: {e}")
                # 安全默认值
                ref_arc_lengths.append(current_arc_length + k * dt * 1.0)  # 默认低速前进
                ref_velocities.append(1.0)
                ref_accelerations.append(0.0)
        
        # 保存虚拟leader当前状态（k=0时刻）供数据记录使用
        if len(ref_arc_lengths) > 0:
            self.vleader_arc_position = ref_arc_lengths[0]
            self.vleader_velocity = ref_velocities[0]
            self.vleader_acceleration = ref_accelerations[0]
        
        return ref_arc_lengths, ref_velocities, ref_accelerations
    
    
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
        # 设置MPC求解器
        try:
            
            # 1.获取离散化车辆模型
            disc_vehicle_model, states_nums, controls_nums = self.get_disc_vehicle_model()
            
            # 2.初始化权重，定义N
            N = self.N
            Q = ca.DM.zeros(3, 3)
            R = ca.DM.zeros(1, 1)
            
            front_arc_pos = 0.0
            current_spacing = 0.0
            
            # 3. 根据车辆ID动态调整权重
            if self.ego_platoon_ID != 1:  # 二三号车
                front_arc_pos = self.frontArcLength
                current_spacing = self.frontArcLength - self.nowArcLength
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
            else:  # 一号车
                Q[0, 0] = 20.0   # 位置权重
                Q[1, 1] = 50.0   # 速度权重  
                Q[2, 2] = 10.0   # 加速度权重
                R[0, 0] = 1.0    # 控制输入权重

            # 定义优化变量
            X = ca.SX.sym('X', states_nums, N + 1)  # 状态变量 [s, v, a]
            U = ca.SX.sym('U', controls_nums, N)    # 控制变量 [jerk]
            P = ca.SX.sym('P', states_nums, N + 2)  # 参数：1个初始状态 + N+1个参考状态
            
            # 4. 定义成本函数和约束
            cost = 0
            constraints = []
            
            # 提取初始状态与参考状态
            x_init = P[:, 0]
            x_ref = P[:, 1:]

            # 初始化状态约束
            constraints.append(X[:, 0] - x_init)

            # 构建成本函数和动力学约束
            for k in range(N):
                # 系统动力学约束
                x_next = disc_vehicle_model(X[:, k], U[:, k])
                constraints.append(X[:, k + 1] - x_next)
                
                # 计算状态误差成本
                if self.ego_platoon_ID != 1:
                    # 编队跟随车：计算间距误差成本（复用缓存的前车位置）
                    ego_position = X[:, k][0]
                    actual_distance = front_arc_pos - ego_position
                    distance_error = actual_distance - self.desired_spacing
                    spacing_cost = Q[0, 0] * distance_error**2
                    
                    # 速度和加速度使用参考轨迹误差
                    v_error = X[:, k][1] - x_ref[:, k][1]
                    a_error = X[:, k][2] - x_ref[:, k][2]
                    tracking_cost = Q[1, 1] * v_error**2 + Q[2, 2] * a_error**2
                    
                    cost += spacing_cost + tracking_cost
                
                else:
                    # 非编队或领车/前车不可用：使用标准轨迹跟踪成本
                    state_error = X[:, k] - x_ref[:, k]
                    cost += state_error.T @ Q @ state_error
                
                # 控制成本
                cost += U[:, k].T @ R @ U[:, k]
                
            # 终端成本
            Q_f = Q * 1.5  # 终端权重
            if self.ego_platoon_ID != 1 :
                ego_position = X[:, N][0]
                actual_distance = front_arc_pos - ego_position
                distance_error = actual_distance - self.desired_spacing
                spacing_cost = Q_f[0, 0] * distance_error**2
                
                v_error = X[:, N][1] - x_ref[:, N][1]
                a_error = X[:, N][2] - x_ref[:, N][2]
                tracking_cost = Q_f[1, 1] * v_error**2 + Q_f[2, 2] * a_error**2
                
                cost += spacing_cost + tracking_cost
            else:
                state_error = X[:, N] - x_ref[:, N]
                cost += state_error.T @ Q_f @ state_error

            # 构建NLP问题
            opt_vars = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))
            opt_params = P
            nlp = {
                'x': opt_vars,
                'p': opt_params,
                'f': cost,
                'g': ca.vertcat(*constraints)
            }
            
            # 配置求解器
            opts = {
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.max_iter': 50,  # 实时系统需要快速求解
                'ipopt.tol': 1e-4,
                'ipopt.acceptable_tol': 1e-3,
                'expand': True
            }
            
            # 创建求解器
            solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
            
            
            # 设置约束边界
            # 计算约束数量
            num_constraints = states_nums + N * states_nums  # 初始约束 + 动力学约束
            
            # 等式约束边界（都为0）
            lbg = ca.DM.zeros(num_constraints)
            ubg = ca.DM.zeros(num_constraints)
            
            # 变量边界
            total_vars = (N + 1) * states_nums + N * controls_nums
            lbx = -ca.inf * ca.DM.ones(total_vars)
            ubx = ca.inf * ca.DM.ones(total_vars)
            
            # 状态约束（个性化调整：速度上限为5.0 m/s，符合低速车辆场景）
            v_min, v_max = 0.0, 5.0  # 速度范围 [m/s]
            a_min, a_max = -3.0, 1.5  # 加速度范围 [m/s²]
            
            # 控制约束（jerk约束，个性化：编队跟随车更宽松）
            if self.ego_platoon_ID != 1:
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
            
            # 包装求解器函数
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
    
    def mpc_solve(self):
        try:
            if not self.GPS_state or self.nowArcLength <= 0:
                print("Warning: Invalid GPS or position for MPC")
                return 0.0
            # 获取当前状态
            current_states = {
                'x': self.nowArcLength,  # 弧长位置
                'v': self.nowSpeed,      # 纵向速度
                'a': self.nowAcceleration  # 纵向加速度
            }
            # 获取参考轨迹
            ref_arc_lengths, ref_velocities, ref_accelerations = self.get_virtual_leader()
            
            if ref_arc_lengths is None:
                print("Warning: Failed to get reference trajectory")
                return 0.0
            
            # 构建参考轨迹矩阵
            ref_traj = ca.DM.zeros(3, self.N + 1)
            for k in range(min(self.N + 1, len(ref_arc_lengths))):
                ref_traj[0, k] = ref_arc_lengths[k]   # 弧长位置
                ref_traj[1, k] = ref_velocities[k]    # 速度
                ref_traj[2, k] = ref_accelerations[k] # 加速度
                
            # 构建初始状态向量
            x_now = ca.DM([
                float(current_states['x']),
                float(current_states['v']),
                float(current_states['a'])
            ])
            
            # 构建参考向量
            p = ca.horzcat(x_now, ref_traj)
            
            # 生成初始猜测
            states_vars = 3 * (self.N + 1)
            total_vars = states_vars + self.N
            x0 = ca.DM.zeros(total_vars, 1)
            
            # 状态变量初始猜测（基于参考轨迹）
            for k in range(self.N + 1):
                x0[k*3:(k+1)*3] = ref_traj[:, k]
            
            # 控制变量初始猜测（小的jerk值）
            for k in range(self.N):
                x0[states_vars + k] = 0.0
                
            # 求解MPC
            solution = self.mpc_solver(x0=x0, p=p)
            
            # 提取控制结果
            if solution is not None and 'x' in solution:
                u_opt = solution['x']
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

        
    def pid_update(self, error):
        """
        下层PID控制器
        """
        # 死区处理
        if abs(error) < self.pid_dead_zone:
            error = 0.0
        
        # 比例项
        p = self.pid_kp * error
        
        # 微分项（低通滤波）
        """
        低通滤波公式：y_k = α * x_k + (1 - α) * y_(k-1)
            其中，y_k 是当前滤波后的值，
                 x_k 是当前输入，
                 y_(k-1) 是上一次滤波后的值，
                 α 是滤波系数（0 < α < 1）
        """
        d_raw = (error - self.pid_prev_error) / self.dt
        self.pid_filtered_d = self.pid_alpha * d_raw + (1 - self.pid_alpha) * self.pid_filtered_d
        d = self.pid_kd * self.pid_filtered_d
        
        # 积分项
        # 只有当输出没有饱和时才进行积分（抗积分饱和）
        potential_out = p + self.pid_ki * (self.pid_integral + error * self.dt) + d
        if self.pid_min_out < potential_out < self.pid_max_out:
            self.pid_integral += error * self.dt

        i = self.pid_ki * self.pid_integral

        self.pid_prev_error = error

        output = p + i + d
        return max(self.pid_min_out, min(self.pid_max_out, output))
    
    def acc_to_control_cmd(self, target_acc, steer_cmd):
        
        # 获取当前状态
        current_acc = self.nowAcceleration if self.nowAcceleration is not None else 0.0
        current_speed = self.nowSpeed if self.nowSpeed is not None else 0.0
        
        # 检查数据有效性
        if current_speed is None or current_acc is None:
            return {'throttle': 0.0, 'brake': 0.0, 'steer': steer_cmd}
        
        # 1. 计算加速度误差
        acc_error = target_acc - current_acc
        
        # 2. PID计算控制力度 + 前馈控制
        pid_output = self.pid_update(acc_error)
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

        
    def find_trajectory_index_by_arc_length(self, arc_length):
        """
        根据弧长在轨迹累积弧长列表中查找最近的索引
        
        Args:
            arc_length: 目标弧长
            
        Returns:
            int: 最近的轨迹点索引，或None如果不可用
        """
        if not self.trajectory_available or len(self.trajectoryCumulativeArcLength) == 0:
            return None
        
        # 使用二分查找找到最近索引
        import bisect
        idx = bisect.bisect_left(self.trajectoryCumulativeArcLength, arc_length)
        
        # 边界处理
        if idx >= len(self.trajectoryCumulativeArcLength):
            idx = len(self.trajectoryCumulativeArcLength) - 1
        elif idx > 0:
            # 选择更近的点
            prev_dist = abs(self.trajectoryCumulativeArcLength[idx-1] - arc_length)
            curr_dist = abs(self.trajectoryCumulativeArcLength[idx] - arc_length)
            if prev_dist < curr_dist:
                idx -= 1
        
        return idx
    
    
    def calculate_arc_length(self, latitude, longitude):
        """
        通用方法：计算给定经纬度点在轨迹上的弧长位置
        使用最近点投影方法，将经纬度转换为局部坐标后在轨迹上找最近点
        
        Args:
            latitude: 纬度
            longitude: 经度
            
        Returns:
            arc_length: 该点在轨迹上的弧长位置（米）
        """
        # 检查轨迹数据是否可用
        if not self.trajectory_available or len(self.trajectoryX) == 0 or len(self.trajectoryCumulativeArcLength) == 0:
            # print("Warning: Trajectory data not available for arc length calculation")
            return self.last_valid_arc_length
        
        # 验证GPS坐标有效性
        if not self.validate_gps_coordinates(latitude, longitude):
            print(f"Warning: Invalid GPS coordinates for arc length calculation: ({latitude}, {longitude})")
            return self.last_valid_arc_length  # 返回上一个有效位置
        
        try:
            # 将经纬度转换为局部坐标系
            x, y, _ = self.conversion_of_coordinates(latitude, longitude, 0)
            
            # 找到轨迹上距离该点最近的点
            min_distance = float('inf')
            nearest_index = 0
            
            for i in range(len(self.trajectoryX)):
                distance = sqrt((self.trajectoryX[i] - x)**2 + (self.trajectoryY[i] - y)**2)
                if distance < min_distance:
                    min_distance = distance
                    nearest_index = i
            
            # 异常检测：如果最近点距离过大，车辆可能偏离轨迹
            if min_distance > 50.0:  # 50米以上认为偏离轨迹
                print(f"Warning: Point too far from trajectory ({min_distance:.2f}m), using last valid position")
                return self.last_valid_arc_length
            
            # 如果距离较大但在合理范围内，给出警告
            if min_distance > 10.0:
                print(f"Warning: Point distance from trajectory: {min_distance:.2f}m")
            
            # 获取最近点对应的累积弧长
            arc_length_position = self.trajectoryCumulativeArcLength[nearest_index]
            
            # 平滑处理：限制位置变化率（避免GPS跳点导致的弧长突变）
            if self.last_valid_arc_length > 0:
                position_change = abs(arc_length_position - self.last_valid_arc_length)
                
                # 如果位置跳变过大，进行平滑处理
                if position_change > 10.0:
                    print(f"Warning: Large arc length jump detected: {position_change:.2f}m")
                    # 使用指数平滑
                    arc_length_position = (self.arc_length_smooth_alpha * arc_length_position + 
                                          (1 - self.arc_length_smooth_alpha) * self.last_valid_arc_length)
            
            # 更新上一次有效位置
            self.last_valid_arc_length = arc_length_position
            
            return arc_length_position
            
        except Exception as e:
            print(f"Error in arc length calculation: {e}")
            return self.last_valid_arc_length
    
    def validate_gps_coordinates(self, latitude, longitude):
        """
        验证GPS坐标有效性
        
        Args:
            latitude: 纬度
            longitude: 经度
            
        Returns:
            bool: 坐标是否有效
        """
        # 检查纬度范围 [-90, 90]
        if latitude < -90 or latitude > 90:
            return False
        # 检查经度范围 [-180, 180]
        if longitude < -180 or longitude > 180:
            return False
        # 检查是否为零值（通常表示无效GPS信号）
        if abs(latitude) < 0.001 and abs(longitude) < 0.001:
            return False
        return True
    
    def update_trajectory_arc_length(self, latitudes, longitudes):
        """
        更新参考轨迹的弧长信息（优化版）
        将经纬度序列转换为局部坐标系，并计算累积弧长
        包含完善的验证和错误处理机制
        
        Args:
            latitudes: 纬度列表
            longitudes: 经度列表
        """
        # 基本验证：检查输入数据
        if not latitudes or not longitudes:
            self.trajectory_available = False
            print("Warning: Empty trajectory data")
            return
        
        if len(latitudes) != len(longitudes):
            self.trajectory_available = False
            print(f"Error: Latitude and longitude length mismatch ({len(latitudes)} vs {len(longitudes)})")
            return
        
        if len(latitudes) < 2:
            self.trajectory_available = False
            print(f"Warning: Invalid reference trajectory (only {len(latitudes)} points)")
            return
        
        # 清空之前的轨迹数据
        self.trajectoryX = []
        self.trajectoryY = []
        self.trajectoryCumulativeArcLength = []
        
        # 转换所有轨迹点到局部坐标系，并验证有效性
        valid_points = 0
        for i in range(len(latitudes)):
            lat, lon = latitudes[i], longitudes[i]
            
            # 验证GPS坐标有效性
            if not self.validate_gps_coordinates(lat, lon):
                print(f"Warning: Invalid GPS coordinates at index {i}: ({lat}, {lon})")
                # 如果是第一个点就无效，使用默认值
                if i == 0:
                    self.trajectoryX.append(0.0)
                    self.trajectoryY.append(0.0)
                else:
                    # 使用前一个有效点
                    self.trajectoryX.append(self.trajectoryX[-1])
                    self.trajectoryY.append(self.trajectoryY[-1])
                continue
            
            try:
                x, y, _ = self.conversion_of_coordinates(lat, lon, 0)
                self.trajectoryX.append(x)
                self.trajectoryY.append(y)
                valid_points += 1
            except Exception as e:
                print(f"Error converting coordinates at index {i}: {e}")
                # 使用前一个点或默认值
                if i == 0:
                    self.trajectoryX.append(0.0)
                    self.trajectoryY.append(0.0)
                else:
                    self.trajectoryX.append(self.trajectoryX[-1])
                    self.trajectoryY.append(self.trajectoryY[-1])
        
        # 检查有效点数量
        if valid_points < 2:
            self.trajectory_available = False
            print(f"Error: Insufficient valid trajectory points ({valid_points}/{len(latitudes)})")
            return
        
        # 计算累积弧长
        cumulative_length = 0.0
        self.trajectoryCumulativeArcLength.append(cumulative_length)
        total_distance = 0.0
        abnormal_segments = 0
        
        for i in range(1, len(self.trajectoryX)):
            # 计算相邻两点之间的距离
            dx = self.trajectoryX[i] - self.trajectoryX[i-1]
            dy = self.trajectoryY[i] - self.trajectoryY[i-1]
            segment_length = sqrt(dx**2 + dy**2)
            
            # 异常段长度检测（段长度超过10米认为异常）
            if segment_length > 10.0:
                abnormal_segments += 1
                print(f"Warning: Abnormal segment distance {segment_length:.2f}m at index {i}")
                # 限制异常段长度，避免影响整体轨迹
                segment_length = min(segment_length, 10.0)
            
            # 累加弧长
            cumulative_length += segment_length
            total_distance += segment_length
            self.trajectoryCumulativeArcLength.append(cumulative_length)
        
        # 最终验证
        if total_distance > 0:
            self.trajectory_available = True
            self.trajectory_update_counter += 1
            print(f"✓ Trajectory arc length updated successfully:")
            print(f"  - Points: {len(self.trajectoryX)}")
            print(f"  - Valid points: {valid_points}/{len(latitudes)}")
            print(f"  - Total length: {total_distance:.2f}m")
            print(f"  - Abnormal segments: {abnormal_segments}")
            print(f"  - Update count: {self.trajectory_update_counter}")
        else:
            self.trajectory_available = False
            print("Error: Failed to build valid trajectory arc length map (zero total distance)")

    def conversion_of_coordinates(self, conversionLatitude, conversionLongitude, conversionAltitude):
            
        #WGS84 参数定义
        wgs84_a  = 6378137.0
        wgs84_f  = 1/298.257223565
        wgs84_e2 = wgs84_f * (2-wgs84_f)
        D2R      = PI / 180.0

        #局部坐标原点经纬度

        #天南街原点
        # Latitude0 = 39.1052154
        # Longitude0 = 117.1641687

        # Longitude0 = 123.24082812
        # Latitude0 = 41.737324
        
        # 图书馆原点
        Longitude0 = 34.58554250654256
        Latitude0 = 113.68553516844531
        
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

    # ==================== 传感器状态滤波方法 ====================
    
    def low_pass_filter(self, new_value, filter_state):
        """
        一阶低通滤波器（指数加权移动平均 EWMA）
        
        Args:
            new_value: 新的传感器测量值
            filter_state: 滤波器状态字典 {'last_value', 'initialized', 'alpha'}
        
        Returns:
            filtered_value: 滤波后的值
        
        滤波公式：y[k] = α * x[k] + (1 - α) * y[k-1]
        
        截止频率（10Hz采样）：
            α=0.3 → f_c≈0.68Hz（适合GPS速度/位置）
            α=0.5 → f_c≈1.59Hz（适合加速度）
            α=0.7 → f_c≈3.72Hz（适合高频响应的传感器）
        """
        if not filter_state['initialized']:
            # 首次数据：直接使用，避免从0开始的过渡
            filter_state['last_value'] = new_value
            filter_state['initialized'] = True
            return new_value
        else:
            # 正常滤波
            alpha = filter_state['alpha']
            filtered = alpha * new_value + (1 - alpha) * filter_state['last_value']
            filter_state['last_value'] = filtered
            return filtered
    
    def angle_low_pass_filter(self, new_angle_rad, filter_state):
        """
        角度专用低通滤波器（处理 ±180° 跳变）
        
        Args:
            new_angle_rad: 新的角度值（弧度）
            filter_state: 滤波器状态字典 {'last_value', 'initialized', 'alpha'}
        
        Returns:
            filtered_angle_rad: 滤波后的角度（弧度，范围 [-π, π]）
        """
        if not filter_state['initialized']:
            filter_state['last_value'] = new_angle_rad
            filter_state['initialized'] = True
            return new_angle_rad
        
        # 计算角度差，处理周期性跳变
        angle_diff = new_angle_rad - filter_state['last_value']
        if angle_diff > PI:
            angle_diff -= 2 * PI
        elif angle_diff < -PI:
            angle_diff += 2 * PI
        
        # 滤波
        alpha = filter_state['alpha']
        filtered = filter_state['last_value'] + alpha * angle_diff
        
        # 归一化到 [-π, π]
        while filtered > PI:
            filtered -= 2 * PI
        while filtered < -PI:
            filtered += 2 * PI
        
        filter_state['last_value'] = filtered
        return filtered


    def sub_callback_platoon_state(self, msgHmiPlatoonState:HmiPlatoonInterface):
        old_id = self.ego_platoon_ID
        self.platoon_state = msgHmiPlatoonState.platoon_state
        self.ego_platoon_ID = msgHmiPlatoonState.platoon_id
        
        # 车辆ID变化时，重新初始化CSV文件
        if old_id != self.ego_platoon_ID and self.ego_platoon_ID > 0 and self.data_recording_enabled:
            # 先保存当前数据
            if self.csv_initialized:
                self.flush_data()
            
            # 重新初始化
            self.csv_initialized = False
            self.recording_start_time = None
            self.sample_count = 0
            print(f"Vehicle ID changed: {old_id} -> {self.ego_platoon_ID}, reinitializing CSV files")
        
    def sub_callback_vtx(self, msgVtx: VtxInterface):
        """
        订阅V2X数据（带滤波）
        获取前车状态信息，对V2X数据进行滤波处理后使用
        """
        
        if self.ego_platoon_ID in [2, 3]:
            # 计算预期前车的ID
            front_id = self.ego_platoon_ID - 1
            
            # 检查收到的消息是否来自前车
            if msgVtx.car_id == front_id:
                # ============ 接收原始V2X数据并滤波 ============
                raw_front_speed = msgVtx.speed
                raw_front_accel = msgVtx.acceleration
                
                # 应用低通滤波器（V2X数据可能有通信延迟/抖动）
                self.frontSpeed = self.low_pass_filter(raw_front_speed, self.front_speed_filter)
                self.frontAcceleration = self.low_pass_filter(raw_front_accel, self.front_accel_filter)
                
                # GPS数据直接使用（已由前车融合处理）
                self.frontLatitude = msgVtx.latitude
                self.frontLongitude = msgVtx.longitude
                
                # 计算前车的弧长位置（原始计算）
                raw_front_arc = self.calculate_arc_length(self.frontLatitude, self.frontLongitude)
                
                # 对前车弧长也进行滤波（避免GPS跳点）
                self.frontArcLength = self.low_pass_filter(raw_front_arc, self.front_arc_filter)
                
                # 存储前车状态对象（用于超时检测）
                self.front_car_state = msgVtx
                
                # 更新前车数据接收时间
                if not hasattr(self, 'front_car_update_time'):
                    self.front_car_update_time = time.time()
                else:
                    self.front_car_update_time = time.time()
    
    # ==================== 数据记录方法 ====================
    
    def initialize_csv_files(self):
        """初始化CSV文件"""
        if self.csv_initialized or not self.data_recording_enabled:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 车辆状态数据文件
        self.csv_filename = os.path.join(
            self.data_dir,
            f'car{self.ego_platoon_ID}_state_{timestamp}.csv'
        )
        
        # 创建并写入表头
        try:
            with open(self.csv_filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    't_s',  # 相对时间（秒）
                    'car_id',
                    'platoon_state',
                    'arc_position_m',
                    'velocity_mps',
                    'acceleration_mps2',
                    'throttle_percent',
                    'brake_percent',
                    'gear',
                    'latitude',
                    'longitude'
                ])
            
            print(f"✓ Created car state CSV: {self.csv_filename}")
        except Exception as e:
            print(f"✗ Error creating car state CSV: {e}")
            self.data_recording_enabled = False
            return
        
        # 如果是1号车，创建虚拟leader数据文件
        if self.ego_platoon_ID == 1:
            self.vleader_csv_filename = os.path.join(
                self.data_dir,
                f'VLeader_state_{timestamp}.csv'
            )
            
            try:
                with open(self.vleader_csv_filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        't_s',  # 相对时间（秒）
                        'arc_position_m',
                        'velocity_mps',
                        'acceleration_mps2'
                    ])
                
                print(f"✓ Created virtual leader CSV: {self.vleader_csv_filename}")
            except Exception as e:
                print(f"✗ Error creating virtual leader CSV: {e}")
        
        self.csv_initialized = True
        self.recording_start_time = None  # 将在首次记录时设置
        self.sample_count = 0
    
    def data_record(self):
        """
        数据记录方法（10Hz定时调用）
        记录车辆状态数据和虚拟leader数据
        """
        # 确保CSV文件已初始化
        if self.ego_platoon_ID > 0 and not self.csv_initialized:
            self.initialize_csv_files()
        
        if not self.csv_initialized or not self.data_recording_enabled:
            return
        
        # 初始化开始时间（第一次记录）
        if self.recording_start_time is None:
            self.recording_start_time = time.time()
            self.sample_count = 0
            print(f"=== Data recording started for vehicle {self.ego_platoon_ID} ===")
        
        # 计算相对时间 t（秒）
        relative_time = self.sample_count * self.dt
        self.sample_count += 1
        
        # 记录车辆状态数据
        car_data = [
            round(relative_time, 2),  # 相对时间
            self.ego_platoon_ID,
            self.platoon_state,
            round(self.nowArcLength, 3),
            round(self.nowSpeed, 3),
            round(self.nowAcceleration, 3),
            self.last_throttle_percentage,
            self.last_brake_percentage,
            self.last_gear,
            round(self.nowLatitude, 8),
            round(self.nowLongitude, 8)
        ]
        
        self.data_buffer.append(car_data)
        
        # 如果是1号车，记录虚拟leader数据
        if self.ego_platoon_ID == 1:
            vleader_data = [
                round(relative_time, 2),  # 相对时间
                round(self.vleader_arc_position, 3),
                round(self.vleader_velocity, 3),
                round(self.vleader_acceleration, 3)
            ]
            self.vleader_buffer.append(vleader_data)
        
        # 缓冲区满时写入文件
        if len(self.data_buffer) >= self.buffer_size:
            self.flush_data()
    
    def flush_data(self):
        """将缓冲区数据写入CSV文件"""
        if not self.csv_initialized or not self.data_recording_enabled:
            return
        
        # 写入车辆状态数据
        if self.data_buffer:
            try:
                with open(self.csv_filename, 'a', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerows(self.data_buffer)
                # self.get_logger().info(f"Flushed {len(self.data_buffer)} car state records")
                self.data_buffer.clear()
            except Exception as e:
                self.get_logger().error(f"Error writing car state data: {e}")
        
        # 写入虚拟leader数据
        if self.ego_platoon_ID == 1 and self.vleader_buffer:
            try:
                with open(self.vleader_csv_filename, 'a', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerows(self.vleader_buffer)
                # self.get_logger().info(f"Flushed {len(self.vleader_buffer)} virtual leader records")
                self.vleader_buffer.clear()
            except Exception as e:
                self.get_logger().error(f"Error writing virtual leader data: {e}")
    
    def save_remaining_data(self):
        """保存剩余的缓冲区数据（在节点关闭时调用）"""
        if self.data_recording_enabled and self.csv_initialized:
            self.flush_data()
            print(f"✓ Data recording completed for vehicle {self.ego_platoon_ID}")
            if self.sample_count > 0:
                total_time = (self.sample_count - 1) * self.dt
                print(f"  Total samples: {self.sample_count}, Duration: {total_time:.1f}s")
                
                
def main():
    rclpy.init()
    rosNode = None
    try:
        rosNode = YMpc(name='pid')
        rclpy.spin(rosNode)
    except KeyboardInterrupt:
        print("\n=== Shutdown requested by user ===")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 保存剩余数据
        if rosNode is not None:
            rosNode.save_remaining_data()
        print("Shutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
