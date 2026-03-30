
#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: pid.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: MPC-based longitudinal platoon control with arc-length distance calculation

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
import cvxpy as cp 
import csv  
import datetime  
from collections import deque

PI = 3.1415926535

ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class Pid(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # define publishers
        self.pubPid = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timerPid = self.create_timer(0.1, self.pub_callback_pid)
        
        # define subscribers
        self.subLocalPathPlanning = self.create_subscription(LocalPathPlanningInterface, 'local_path_planning_data', self.sub_callback_local_path_planning, 10)
        self.subFusion = self.create_subscription(FusionInterface, 'fusion_data', self.sub_callback_fusion, 10)
        self.subHmiPlatoonState = self.create_subscription(HmiPlatoonInterface, "platoon_state_data", self.sub_callback_platoon_state, 10)
        self.subVtxData = self.create_subscription(VtxInterface, "v2x_to_planner_data", self.sub_callback_vtx, 10)

        root_path = os.getcwd() + "/src/pid/pid/"  # 根目录
        # ======================================================== #
        # 从config文件中读取超参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f)
        self.yaml_data = EasyDict(config)
        print(self.yaml_data)

        self.nowSpeed = 0.0
        self.nowLatitude = 0.0
        self.nowLongitude = 0.0
        self.nowYaw = 0.0
        self.nowPitch = 0.0
        
        self.refSpeed = 0.0
        self.refLatitude = 0.0
        self.refLongitude = 0.0
        self.refYaw = 0.0

        self.control_value_last = 0
        self.turn_signals = 0.0
        self.trajectory_state = False
        self.GPS_state = False
        self.arrived = False

        # 添加滤波器和异常值检测
        self.speed_filter = deque(maxlen=5)  # 速度滤波器
        self.position_filter = deque(maxlen=3)  # 位置滤波器
        self.last_valid_position = (0.0, 0.0)  # 上一个有效位置

        # 参考轨迹存储
        self.reference_trajectory = []  # 存储完整的参考轨迹 [(lat, lon), ...]
        self.trajectory_distances = []  # 存储轨迹上累积距离 [0, d1, d2, ..., dn]
        self.trajectory_available = False
        self.trajectory_update_counter = 0  # 轨迹更新计数器

        # ========== 编队控制参数========== #
        self.dt = 0.1  # 采样时间
        self.N = 10    # MPC预测步长
        
        # 编队状态变量
        self.platoon_state = 0          # 编队状态：0-非编队，1-编队模式
        self.ego_platoon_ID = None      # 当前车辆在编队中的ID：1-领车，2/3-跟随车
        self.front_car_state = None     # 前车信息（直接存储V2X消息）

        self.front_car_speed=0.0
        self.front_car_longitude = 0.0
        self.front_car_latitude = 0.0
        self.front_car_yaw = 0.0
        self.front_car_update_time = 0.0  # 前车信息更新时间
        
        # 存储所有车辆的V2X状态信息
        self.v2x_vehicles = {}     # 存储所有收到的V2X车辆信息
        
        # 当前车辆状态
        self.current_trajectory_position = 0.0  # 当前在轨迹上的弧长位置
        self.current_velocity = 0.0    # 当前速度（滤波后）
        
        # MPC约束参数
        self.u_min = -4.0  # 最小加速度 (m/s^2)
        self.u_max = 2.0   # 最大加速度 (m/s^2)  
        self.v_min = 0.0   # 最小速度 (m/s)
        self.v_max = 10.0  # 最大速度 (m/s)
        
        # 编队参数
        self.max_platoon_speed = 4.0     # 编队最高速度
        self.safe_platoon_distance = 8.0  # 编队安全距离
        self.desired_spacing = self.safe_platoon_distance    # 期望车间距
        self.min_spacing = 5.0         # 最小安全间距 (m)
        self.max_spacing = 15.0        # 最大间距 (m)
        
        # MPC权重矩阵
        self.Q_spacing = 20.0  # 间距跟踪权重
        self.Q_velocity = 10.0  # 速度跟踪权重
        self.R_control = 3.0   # 控制输入权重
        self.R_jerk = 1.0      # 控制变化率权重
        self.Q_terminal = 30.0 # 终端代价权重

        # 控制器状态
        self.mpc_solver_initialized = False
        self.last_control_input = 0.0  # 上一时刻控制输入
        self.control_deadzone = 0.05   # 控制死区
        
        # 机器学习模型参数（加速度到油门/刹车映射）
        self.init_throttle_brake_mapping()

        # ========== CSV数据保存相关变量 ========== #
        self.init_csv_logging()
        
        self.saveData = open("./src/pid/test/controlData.txt","w")

        # 添加调试标志
        self.distance_calc_debug = True  # 开启距离计算调试

    def init_csv_logging(self):
        """初始化CSV数据保存功能"""
        # 创建数据保存目录（如果不存在）
        data_dir = "./src/pid/data/"
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        
        # 生成带时间戳的文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"{data_dir}mpc_platoon_data_{timestamp}.csv"
        
        # 创建CSV文件并写入表头
        with open(self.csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            header = [
                'timestamp', 'vehicle_id', 'platoon_state', 'ego_platoon_ID',
                'distance_to_preceding', 'current_velocity', 'mpc_acceleration',
                'preceding_velocity', 'desired_spacing', 'spacing_error', 'velocity_error',
                'throttle_percentage', 'brake_percentage', 'control_method'
            ]
            writer.writerow(header)

        # 使用缓存机制减少IO操作
        if not hasattr(self, 'csv_buffer'):
            self.csv_buffer = []
            self.csv_buffer_size = 10  # 每10条数据写入一次
        
        print(f"CSV data logging initialized: {self.csv_filename}")

    def save_mpc_data_to_csv(self, distance_to_preceding, current_velocity, mpc_acceleration, 
                            throttle_percentage, brake_percentage, control_method="MPC"):
        """保存编队控制数据到CSV文件"""
        try:
            # 计算相关误差
            if self.front_car_state is not None:
                preceding_velocity = self.front_car_state.speed
                spacing_error = distance_to_preceding - self.desired_spacing
                velocity_error = current_velocity - preceding_velocity
            else:
                preceding_velocity = 0.0
                spacing_error = 0.0
                velocity_error = current_velocity - self.refSpeed
            
            # 构造数据行
            data_row = [
                time.time(), self.ego_platoon_ID, int(self.platoon_state), self.ego_platoon_ID,
                round(distance_to_preceding, 3), round(current_velocity, 3), round(mpc_acceleration, 4),
                round(preceding_velocity, 3), round(self.desired_spacing, 1), round(spacing_error, 3),
                round(velocity_error, 3), throttle_percentage, brake_percentage, control_method
            ]
            
            self.csv_buffer.append(data_row)
            
            # 当缓存满时写入文件
            if len(self.csv_buffer) >= self.csv_buffer_size:
                with open(self.csv_filename, 'a', newline='', encoding='utf-8') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(self.csv_buffer)
                self.csv_buffer = []  # 清空缓存
                
        except Exception as e:
            print(f"Error saving data to CSV: {e}")
    
    def init_throttle_brake_mapping(self):
        """初始化加速度到油门/刹车的映射模型"""
        self.throttle_brake_available = True
        
        # 油门映射参数（非线性映射）
        self.throttle_params = {
            'base': 30.0,      # 基础油门
            'gain': 12.0,      # 加速度增益
            'speed_factor': 2.0,  # 速度补偿因子
            'deadzone': 0.05   # 死区阈值
        }
        
        # 刹车映射参数（非线性映射）
        self.brake_params = {
            'base': 15.0,      # 基础刹车
            'gain': 20.0,      # 减速度增益
            'speed_factor': 3.0,  # 速度补偿因子
            'deadzone': 0.05   # 死区阈值
        }

    def acceleration_to_throttle_brake(self, acceleration, velocity):
        """将加速度转换为油门/刹车百分比"""
        # 处理停车状态
        if abs(velocity) < 0.1 and abs(acceleration) < 0.1:
            return 0, 20  # 停车时轻微刹车
        
        # 加速控制
        if acceleration > self.throttle_params['deadzone']:
            # 非线性油门映射
            throttle = self.throttle_params['base'] + \
                      acceleration * self.throttle_params['gain'] * (1 + 0.1 * acceleration) + \
                      max(0, (3 - velocity)) * self.throttle_params['speed_factor']
            
            # 限幅
            throttle = np.clip(throttle, 0, 80)  # 限制最大油门为80%
            return int(throttle), 0
            
        # 减速控制
        elif acceleration < -self.brake_params['deadzone']:
            # 非线性刹车映射
            brake = self.brake_params['base'] + \
                   abs(acceleration) * self.brake_params['gain'] * (1 + 0.15 * abs(acceleration)) + \
                   velocity * self.brake_params['speed_factor']
            
            # 紧急制动
            if acceleration < -3.0:
                brake = min(100, brake * 1.5)
            
            # 限幅
            brake = np.clip(brake, 0, 100)
            return 0, int(brake)
            
        # 保持当前速度（小幅油门）
        else:
            if velocity < 2.0:
                throttle = 25 + (2 - velocity) * 5
            else:
                throttle = 20 + (3 - velocity) * 2
            throttle = np.clip(throttle, 15, 35)
            return int(throttle), 0

    #================================轨迹和距离计算==========================#
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
        修复：确保返回的是实际物理距离
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

    def update_front_car_state(self):
        """更新前车信息"""
        if self.front_car_state is None:
            return
        
        # 检查前车信息时效性
        current_time = time.time()
        if hasattr(self.front_car_state, 'timestamp'):
            if current_time - self.front_car_state.timestamp > 1.0:  # 超过1秒认为过期
                print("Warning: Front car data is outdated")
                return
        
        # 验证前车坐标
        if self.validate_gps_coordinates(self.front_car_state.latitude, self.front_car_state.longitude):
            self.front_car_speed = self.front_car_state.speed
            self.front_car_longitude = self.front_car_state.longitude
            self.front_car_latitude = self.front_car_state.latitude
            self.front_car_yaw = self.front_car_state.yaw
            self.front_car_update_time = current_time
        else:
            print("Warning: Invalid front car GPS coordinates")

    def get_platoon_speed_simple(self):
        """
        简单编队速度控制，用作MPC的fallback或基准
        """
        if self.front_car_state is None:
            return min(self.refSpeed, self.max_platoon_speed)
        
        front_car_speed = self.front_car_state.speed
        
        # 计算车辆间距
        distance_to_front_car = self.calculate_distance_between_vehicles(
            self.front_car_state.latitude, self.front_car_state.longitude,
            self.nowLatitude, self.nowLongitude
        )
        
        if distance_to_front_car < 0:
            print("Warning: Invalid distance calculation, using reference speed")
            return min(self.refSpeed, self.max_platoon_speed)
        
        print(f"Simple Control - Distance: {distance_to_front_car:.2f}m, Front speed: {front_car_speed:.2f}m/s")
        
        error_distance = distance_to_front_car - self.safe_platoon_distance
        
        # 改进的速度控制策略
        if distance_to_front_car < self.min_spacing:
            # 紧急制动
            ref_platoon_speed = 0.0
        elif error_distance > 2.0:
            # 加速追赶（但不超过前车速度太多）
            ref_platoon_speed = min(front_car_speed + 1.0, front_car_speed + error_distance * 0.3)
        elif error_distance < -2.0:
            # 减速保持距离
            ref_platoon_speed = max(0, front_car_speed + error_distance * 0.5)
        else:
            # 保持与前车相同速度
            ref_platoon_speed = front_car_speed
        
        # 限速
        ref_platoon_speed = np.clip(ref_platoon_speed, 0, self.max_platoon_speed)
        return ref_platoon_speed

    def mpc_platoon_control(self):
        """基于MPC的编队纵向控制"""
        try:
            # 添加数据有效性检查
            if self.current_velocity < 0 or self.current_velocity > 50:
                print(f"Warning: Invalid current velocity {self.current_velocity}")
                return self.fallback_control()
            
            # 定义优化变量
            u = cp.Variable(self.N)         # 控制输入序列 (加速度)
            s = cp.Variable(self.N + 1)     # 位置状态序列（轨迹弧长位置）
            v = cp.Variable(self.N + 1)     # 速度状态序列
            
            # 约束列表
            constraints = []
            
            # 初始状态约束
            constraints.append(s[0] == self.current_trajectory_position)
            constraints.append(v[0] == self.current_velocity)
            
            # 车辆动力学约束
            for k in range(self.N):
                constraints.append(s[k + 1] == s[k] + v[k] * self.dt + 0.5 * u[k] * self.dt**2)
                constraints.append(v[k + 1] == v[k] + u[k] * self.dt)
            
            # 输入约束
            constraints.append(u >= self.u_min)
            constraints.append(u <= self.u_max)
            
            # 速度约束
            constraints.append(v >= self.v_min)
            constraints.append(v <= self.v_max)
            
            front_car_arc_position = None
            front_speed = 0.0
           
            if self.front_car_state is not None and self.trajectory_available:
                _, front_car_arc_position = self.project_point_to_trajectory(
                    self.front_car_state.latitude, self.front_car_state.longitude
                )
                front_speed = np.clip(self.front_car_state.speed, 0, self.v_max)
                
            # 间距安全约束    
            if front_car_arc_position is not None and front_car_arc_position > self.current_trajectory_position:
                for k in range(self.N):
                    # 预测前车位置（假设前车保持当前速度）
                    predicted_front_pos = front_car_arc_position + front_speed * (k + 1) * self.dt
                    spacing = predicted_front_pos - s[k + 1]  # 前车位置 - 本车位置

                    # 动态安全距离（与速度相关）
                    dynamic_min_spacing = self.min_spacing + 0.5 * v[k + 1]
                    constraints.append(spacing >= dynamic_min_spacing)  # 最小安全间距
            
            # 构造目标函数
            cost = 0.0
            
            # 统一目标速度（修复：这里原本有错误，min函数参数问题）
            if self.platoon_state == 1 and front_car_arc_position is not None:
                target_velocity = min(front_speed, self.max_platoon_speed)
            else:
                target_velocity = min(self.refSpeed, self.max_platoon_speed)
            
            for k in range(self.N):
                # 间距跟踪（如果有前车）
                if front_car_arc_position is not None and front_car_arc_position > self.current_trajectory_position:
                    predicted_front_pos = front_car_arc_position + front_speed * (k + 1) * self.dt
                    actual_spacing = predicted_front_pos - s[k + 1]
                    
                    # 动态期望间距
                    dynamic_desired_spacing = self.desired_spacing + 0.3 * v[k + 1]
                    spacing_error = actual_spacing - dynamic_desired_spacing
                    
                    # 非对称权重（间距过小时权重更大）
                    if spacing_error < 0:
                        cost += 2.0 * self.Q_spacing * cp.square(spacing_error)
                    else:
                        cost += self.Q_spacing * cp.square(spacing_error)
                    
                    # 软约束：惩罚过大间距
                    over_spacing = cp.pos(actual_spacing - self.max_spacing)
                    cost += 0.5 * self.Q_spacing * cp.square(over_spacing)
   
                velocity_error = v[k + 1] - target_velocity
                cost += self.Q_velocity * cp.square(velocity_error)
                
                # 控制输入代价
                cost += self.R_control * cp.square(u[k])
                # 控制变化率代价（减少抖动）
                if k > 0:
                    cost += self.R_jerk * cp.square(u[k] - u[k-1])
            
            # 终端代价
            if front_car_arc_position is not None:
                terminal_front_pos = front_car_arc_position + front_speed * self.N * self.dt
                terminal_spacing = terminal_front_pos - s[self.N]
                terminal_spacing_error = terminal_spacing - self.desired_spacing
                cost += self.Q_terminal * cp.square(terminal_spacing_error)

            # 终端速度约束
            terminal_velocity_error = v[self.N] - target_velocity
            cost += self.Q_terminal * 0.5 * cp.square(terminal_velocity_error)
            
            # 求解优化问题
            problem = cp.Problem(cp.Minimize(cost), constraints)
            problem.solve(solver=cp.OSQP, verbose=False, eps_abs=1e-3, eps_rel=1e-3, max_iter=1000, warm_start=True)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                optimal_acceleration = u.value[0]
                # 输出平滑（避免控制抖动）
                alpha = 0.7  # 平滑系数
                optimal_acceleration = alpha * optimal_acceleration + (1 - alpha) * self.last_control_input
                self.last_control_input = optimal_acceleration
                # 异常值检查
                if abs(optimal_acceleration) > 10.0 or np.isnan(optimal_acceleration):
                    print(f"Warning: Abnormal MPC acceleration {optimal_acceleration:.3f}, using fallback")
                    return self.fallback_control()
                print(f"MPC Platoon Control - Optimal acceleration: {optimal_acceleration:.3f} m/s^2")
                return optimal_acceleration
            else:
                print(f"MPC optimization failed: {problem.status}")
                return self.fallback_control()
                
        except Exception as e:
            print(f"MPC error: {e}")
            import traceback
            traceback.print_exc()
            return self.fallback_control()

    def fallback_control(self):
        """当MPC失败时的fallback控制策略"""
        print("Using fallback control")
        
        # 如果在编队模式且有前车信息
        if self.platoon_state == 1 and self.front_car_state is not None:
            distance = self.calculate_distance_between_vehicles(
                self.front_car_state.latitude, self.front_car_state.longitude,
                self.nowLatitude, self.nowLongitude
            )
            
            if distance > 0:
                # 基于距离误差的P控制
                distance_error = distance - self.desired_spacing
                
                # 基于速度误差的P控制
                speed_error = self.front_car_state.speed - self.current_velocity
                
                # 综合控制
                acceleration = 0.3 * distance_error + 0.5 * speed_error
                
                # 限幅
                acceleration = np.clip(acceleration, self.u_min, self.u_max)
                
                print(f"Fallback - Distance: {distance:.2f}m, Acceleration: {acceleration:.3f} m/s²")
                return acceleration
        
        # 默认：跟踪参考速度
        target_speed = min(self.refSpeed, self.max_platoon_speed)
        speed_error = target_speed - self.current_velocity
        acceleration = np.clip(speed_error * 1.0, self.u_min, self.u_max)
        
        print(f"Fallback (default) - Target: {target_speed:.2f} m/s, Acceleration: {acceleration:.3f} m/s²")
        return acceleration

    def update_vehicle_state(self):
        """更新车辆状态"""
        # 速度滤波
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
            _, current_arc_pos = self.project_point_to_trajectory(self.nowLatitude, self.nowLongitude)
            if current_arc_pos is not None:
                # 位置平滑
                alpha = 0.8
                self.current_trajectory_position = alpha * current_arc_pos + (1 - alpha) * self.current_trajectory_position
        else:
            # 基于速度积分更新位置
            self.current_trajectory_position += self.current_velocity * self.dt
        
        # 更新有效位置记录
        if self.validate_gps_coordinates(self.nowLatitude, self.nowLongitude):
            self.last_valid_position = (self.nowLatitude, self.nowLongitude)

    def pub_callback_pid(self):
        """主控制回调函数"""
        msgPid = PidInterface()
        msgPid.timestamp = time.time()

        # 更新车辆状态和前车信息
        self.update_vehicle_state()
        self.update_front_car_state()

        #####################################横向控制#########################################################
        # 横向控制保持原有PID逻辑
        nowPosX, nowPosY, nowPosZ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)
        refPosX, refPosY, refPosz = self.conversion_of_coordinates(self.refLatitude, self.refLongitude, 0)

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

        msgPid.angle = controlValue
        msgPid.velocity = self.refSpeed
        print("refspeed=", msgPid.velocity)

        ###################################纵向编队控制####################################################
        # 初始化变量避免未定义错误
        mpc_acceleration = 0.0
        control_method = "Normal"
        distance_to_preceding = -1.0
        
        # 计算与前车的距离
        if self.front_car_state is not None:
            distance_to_preceding = self.calculate_distance_between_vehicles(
                self.front_car_state.latitude, self.front_car_state.longitude,
                self.nowLatitude, self.nowLongitude
            )
            # 如果距离计算失败，使用默认安全距离
            if distance_to_preceding < 0:
                distance_to_preceding = 20.0  # 默认安全距离
                print("Warning: Using default safe distance 20.0m")

        # 根据不同场景选择控制策略
        # 场景1：无前车或非编队模式
        if self.platoon_state != 1 or self.front_car_state is None:
            target_speed = min(self.refSpeed, self.max_platoon_speed)
            speed_error = target_speed - self.current_velocity
            mpc_acceleration = np.clip(speed_error * 1.2, self.u_min, self.u_max)
            control_method = "Cruise_NoFront"
            
        # 场景2：前车静止（停车跟随）
        elif self.front_car_state.speed < 0.1:
            if distance_to_preceding < self.min_spacing:
                # 紧急制动
                mpc_acceleration = -3.0
                control_method = "Emergency_Stop"
            elif distance_to_preceding < self.desired_spacing:
                # 缓慢制动
                mpc_acceleration = -1.5
                control_method = "Gentle_Stop"
            elif self.current_velocity < 0.1:
                # 已停车
                mpc_acceleration = 0.0
                control_method = "Stopped"
            else:
                # 减速接近
                mpc_acceleration = -0.5 * self.current_velocity
                control_method = "Approaching_Stop"
        
        # 场景3：编队行驶
        else:
            if self.ego_platoon_ID == 1:
                # 领车：跟踪参考速度
                target_speed = min(self.refSpeed, self.max_platoon_speed)
                speed_error = target_speed - self.current_velocity
                mpc_acceleration = np.clip(speed_error * 1.5, self.u_min, self.u_max)
                control_method = "Leader_Control"
            else:
                # 跟随车：使用MPC
                mpc_acceleration = self.mpc_platoon_control()
                control_method = "MPC_Following"
            
        # 将加速度转换为油门/刹车百分比
        throttle, brake = self.acceleration_to_throttle_brake(mpc_acceleration, self.nowSpeed)
        
        # 特殊处理：完全停车状态
        if self.current_velocity < 0.05 and abs(mpc_acceleration) < 0.1:
            throttle = 0
            brake = 40  # 保持轻微刹车
            control_method += "_Hold"
            
        msgPid.throttle_percentage = throttle
        msgPid.braking_percentage = brake
            
        # 输出调试信息
        print(f"=== MPC_Platoon Debug ===")
        print(f"Platoon State: {self.platoon_state}, Vehicle ID: {self.ego_platoon_ID}")
        print(f"Current State - Arc Position: {self.current_trajectory_position:.2f}m, Velocity: {self.current_velocity:.2f}m/s")
        if self.front_car_state is not None:
            print(f"Front Car (Platoon ID: {self.front_car_state.platoon_id}) - Distance: {distance_to_preceding:.2f}m, Speed: {self.front_car_state.speed:.2f}m/s")
        print(f"Reference Speed: {self.refSpeed:.2f}m/s")
        print(f"Control Acceleration: {mpc_acceleration:.3f}m/s^2")
        print(f"Control Output - Throttle: {throttle}%, Brake: {brake}%")

        # 保存数据到CSV
        self.save_mpc_data_to_csv(
            distance_to_preceding, self.current_velocity, mpc_acceleration,
            msgPid.throttle_percentage, msgPid.braking_percentage, control_method
        )
        
        # 限制最大油门/刹车（安全）
        msgPid.throttle_percentage = min(100, max(0, msgPid.throttle_percentage))
        msgPid.braking_percentage = min(100, max(0, msgPid.braking_percentage))
      
        if self.arrived:
            msgPid.gear = 2
        else:
            msgPid.gear = 3
        msgPid.turn_signals = self.turn_signals

        self.pubPid.publish(msgPid)
        print("throttle = %d, brake = %d"%(msgPid.throttle_percentage, msgPid.braking_percentage))

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
        
        if len(msgLocalPathPlanning.angle) > advanceDistance:
            self.refYaw = msgLocalPathPlanning.angle[advanceDistance] / 180 * PI
        else:
            self.refYaw = msgLocalPathPlanning.angle[-1] / 180 * PI

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

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """订阅融合数据"""
        self.nowSpeed = msgFusion.carspeed
        print("nowspeed^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^6")
        print(self.nowSpeed) 

        self.nowLatitude = msgFusion.latitude
        self.nowLongitude = msgFusion.longitude

        if abs(self.nowLatitude) < 1 and abs(self.nowLongitude) < 1:
            self.GPS_state = False
        else:
            self.GPS_state = True 

        self.nowYaw = msgFusion.yaw/180*PI
        self.nowPitch = msgFusion.pitch

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
            
            # 重置MPC控制器
            self.last_control_input = 0.0

    def sub_callback_vtx(self, msgVtx:VtxInterface):
        """
        订阅V2X数据，更新前车信息
        修复：正确处理编队ID逻辑
        """
        print("-----get vtx data---------")
        
        # 存储所有V2X车辆信息
        if hasattr(msgVtx, 'platoon_id'):
            self.v2x_vehicles[msgVtx.platoon_id] = msgVtx
            print(f"Received V2X data from vehicle ID: {msgVtx.platoon_id}")
        
        # 当前不在编队情况
        if self.platoon_state != 1:
            self.ego_platoon_ID = None
            self.front_car_state = None 
            return
        
        # 当前在编队情况，根据自身ID确定前车
        if self.ego_platoon_ID == 2:
            # 车辆2的前车是车辆1
            if msgVtx.platoon_id == 1:
                self.front_car_state = msgVtx
                print(f"Vehicle 2: Updated front car (ID 1) - Lat: {msgVtx.latitude:.6f}, Lon: {msgVtx.longitude:.6f}")
        elif self.ego_platoon_ID == 3:
            # 车辆3的前车是车辆2
            if msgVtx.platoon_id == 2:
                self.front_car_state = msgVtx
                print(f"Vehicle 3: Updated front car (ID 2) - Lat: {msgVtx.latitude:.6f}, Lon: {msgVtx.longitude:.6f}")
        elif self.ego_platoon_ID == 1:
            # 领车没有前车
            self.front_car_state = None
            print("Vehicle 1 (Leader): No front car")

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
        rosNode = Pid(name='pid')
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
