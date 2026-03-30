#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: Zou Zhenshan, Liu Xinlong, He Yongqi
# @File: parking_path_planning.py
# @Project: Auto-Driving System
# @CreateTime: 2024/11/06
# @Description: ***

import sys
import os
import time
import copy
import json
import numpy as np
import cv2
import array
import rclpy
from rclpy.node import Node
import math
import threading
from threading import Thread
from car_interfaces.msg import *

ros_node_name = "parking_path_planning"
sys.path.append(os.getcwd() + "/src/utils")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
from tjitools import gps_to_enu, enu_to_gps
import heapq
from scipy.spatial import cKDTree
import parking_path_planning.reeds_shepp_path_planning as rs
import parking_path_planning.hybrid_a_star_tools as hatools
from parking_path_planning.hybrid_a_star_tools import XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION, SearchNode, Config 
from parking_path_planning.dynamic_programming_heuristic import calc_distance_heuristic
from parking_path_planning.car import BUBBLE_R, WB, W, LB, LF
from parking_path_planning.smooth import path_optimization

class GpsData:
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

class ParkingPathPlanning(Node):
    def __init__(self):
        super().__init__(ros_node_name)
        
        # degine params
        # self.declare_parameter('parking_type', 0)
        # self.declare_parameter('x_goal', 0)
        # self.declare_parameter('y_goal', 0)
        # self.declare_parameter('theta_goal', 0)
        
        self.declare_parameter('parking_type', 1)
        self.declare_parameter('x_goal', 117.16628608)
        self.declare_parameter('y_goal', 39.10526649)
        self.declare_parameter('theta_goal', 181.94)
        
        
        self.parking_type = self.get_parameter('parking_type').get_parameter_value().integer_value
        self.x_goal = self.get_parameter('x_goal').get_parameter_value().double_value
        self.y_goal = self.get_parameter('y_goal').get_parameter_value().double_value
        self.theta_goal = self.get_parameter('theta_goal').get_parameter_value().double_value
        self.theta_goal = rs.pi_2_pi(self.theta_goal/180*math.pi) # [-pi, pi]
        # 
        goal_data_ori = np.array([self.x_goal, self.y_goal, 0])
        print("original_refpoint =", goal_data_ori)
        goal_data_conv = gps_to_enu(goal_data_ori)
        self.x_goal = round(goal_data_conv[0],2)
        self.y_goal = round(goal_data_conv[1],2)
        print("transformed_refpoint_x =", self.x_goal)
        print("transformed_refpoint_y =", self.y_goal)        
        # define publishers

        # AutoCar version
        # self.pubGlobalPathPlanning = self.create_publisher(GlobalPathPlanningInterface, "global_path_planning_data", 1)
        # self.timerGlobalPathPlanning = self.create_timer(0.1, self.pub_callback_global_path_planning)

        self.pubLocalPathPlanning = self.create_publisher(LocalPathPlanningInterface, "local_path_planning_data", 1)
        self.timerLocallPathPlanning = self.create_timer(0.1, self.pub_callback_local_path_planning)
        # self.pubParkingObstacle = self.create_publisher(ParkingObstacleInterface, "parking_obstacle_data", 1)
        # self.timerParkingObstacle = self.create_timer(1, self.pub_callback_parking_obstacle_data)

        # define subscribers
        self.subSelfPose_subscriber = self.create_subscription(GpsInterface, "gps_data", self.pos_callback, 10)
        
        self.gps_data = None
        self.msgGlobalPathPlanning = None
        self.msgLocalPathPlanning = None
        self.msgParkingObstacle = None
        self.max_dis = 50.0  # 泊车距离目标点的最远距离
        self.is_planning = False
        self.is_finished = False
        # 车辆参数
        self.lf = LF     # 前轮轴距
        self.lr = LB     # 后轮轴距
        self.wc = W      # 车宽度
        self.Lg = 6.0    # 车库长度
        self.Wg = 2.5    # 车库宽度
        self.Wr = 6    # 道路宽度
        self.road_dis = 10    # 延长的车道长度

    def parking_judgment(self):
        """
        判断泊车是否满足条件

        Returns:
            bool: 是否能够进行泊车
        """
        if (self.gps_data == None):
            self.get_logger().warning("waiting for gps data!")
            return False
        else:
            dx = self.gps_data.x - self.x_goal
            dy = self.gps_data.y - self.y_goal
            dis = math.sqrt(pow(dx, 2) + pow(dy, 2))
            if (dis > self.max_dis):
                return False
            else:
                return True
    def parking_boundary(self, goal, parking_type): 
        """
        生成泊车边界
        Args:
            goal (tuple): 目标点坐标
            parking_type (int): 泊车类型, 0为倒库, 1为右侧开口泊车, 2为左侧开口泊车
        """
        m, a, b, c, d = self.calculate_rectangle_vertices(goal, parking_type) # 计算四个顶点的坐标
        ox, oy = [], []
        if parking_type == 0:
            points_start = [a, b, c]
            points_end = [b, c, d]
            for i in range(len(points_start)):
                ox1, oy1 = self.sample_points_between(points_start[i], points_end[i], XY_GRID_RESOLUTION)
                ox += ox1
                oy += oy1
            ox1, oy1 = self.road_boundary(m, b, c, a, d, XY_GRID_RESOLUTION, self.road_dis, parking_type)
            ox += ox1
            oy += oy1
        elif parking_type == 1:
            points_start = [b, c, d]
            points_end = [c, d, a]
            for i in range(len(points_start)):
                ox1, oy1 = self.sample_points_between(points_start[i], points_end[i], XY_GRID_RESOLUTION)
                ox += ox1
                oy += oy1
            ox1, oy1 = self.road_boundary(m, c, d, b, a, XY_GRID_RESOLUTION, self.road_dis, parking_type)
            ox += ox1
            oy += oy1
        elif parking_type == 2:
            points_start = [d, a, b]
            points_end = [a, b, c]
            for i in range(len(points_start)):
                ox1, oy1 = self.sample_points_between(points_start[i], points_end[i], XY_GRID_RESOLUTION)
                ox += ox1
                oy += oy1
            ox1, oy1 = self.road_boundary(m, a, b, d, c, XY_GRID_RESOLUTION, self.road_dis, parking_type)
            ox += ox1
            oy += oy1    
        return ox, oy

    def calculate_rectangle_vertices(self, goal, parking_type):
        Lg = self.Lg
        Wg = self.Wg
        x, y, theta = goal
        dx = (self.lf - self.lr) / (self.lf + self.lr) * Lg / 2
        
        # 计算矩形中心点
        x_c = x + dx * math.cos(theta)
        y_c = y + dx * math.sin(theta)
        
        # 
        # 计算矩形的四个顶点
        a = (x_c + Lg / 2 * math.cos(theta) - Wg / 2 * math.sin(theta),
            y_c + Lg / 2 * math.sin(theta) + Wg / 2 * math.cos(theta))
        b = (x_c - Lg / 2 * math.cos(theta) - Wg / 2 * math.sin(theta),
            y_c - Lg / 2 * math.sin(theta) + Wg / 2 * math.cos(theta))
        c = (x_c - Lg / 2 * math.cos(theta) + Wg / 2 * math.sin(theta),
            y_c - Lg / 2 * math.sin(theta) - Wg / 2 * math.cos(theta))
        d = (x_c + Lg / 2 * math.cos(theta) + Wg / 2 * math.sin(theta),
            y_c + Lg / 2 * math.sin(theta) - Wg / 2 * math.cos(theta))

        
        # 计算拓展道路边界中心点
        if (parking_type == 0):
            dx = Lg / 2 + self.Wr
            m = (x_c + dx * math.cos(theta), y_c + dx * math.sin(theta))
        elif (parking_type == 1):
            dx = Wg / 2 + self.Wr
            m = (x_c - dx * math.sin(theta), y_c + dx * math.cos(theta))
        elif (parking_type == 2):
            dx = -Wg / 2 - self.Wr
            m = (x_c - dx * math.sin(theta), y_c + dx * math.cos(theta))

        return m, a, b, c, d
    
    def road_boundary(self, p0, p1, p2, p3, p4, resolution, road_dis, parking_type):
        """
        生成车库相关的道路边界。
        参数:
        p1 (tuple): 第一个点的坐标 (x1, y1)
        p2 (tuple): 第二个点的坐标 (x2, y2)
        p3 (tuple): 第一个点的坐标 (x3, y3)
        p4 (tuple): 第二个点的坐标 (x4, y4)
        resolution (double): 采样点的分辨率
        dis (double): 道路边界的长度
        返回:
        ox, oy: 采样点的横纵坐标
        """
        x0, y0 = p0
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        ox, oy = [], []
        # 计算两点之间的距离
        dis = math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

        # 计算并归一化方向向量
        dx = (x2 - x1) / dis
        dy = (y2 - y1) / dis

        d = 0
        while (d < road_dis):
            ox.extend([x3 - d * dx, x4 + d * dx])
            oy.extend([y3 - d * dy, y4 + d * dy])
            d += resolution
        d = 0
        if parking_type == 0:
            road_dis += self.Wg / 2
        else:
            road_dis += self.Lg / 2
        while (d < road_dis):
            ox.extend([x0 - d * dx, x0 + d * dx])
            oy.extend([y0 - d * dy, y0 + d * dy])
            d += resolution
        
        return ox, oy
    def sample_points_between(self, p1, p2, resolution):
        """
        在两个点之间均匀采样生成序列点。
        参数:
        p1 (tuple): 第一个点的坐标 (x1, y1)
        p2 (tuple): 第二个点的坐标 (x2, y2)
        resolution (double): 采样点的分辨率
        返回:
        ox, oy: 采样点的横纵坐标
        """
        x1, y1 = p1
        x2, y2 = p2
        ox, oy = [], []
        # 计算两点之间的距离
        dis = math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

        # 计算并归一化方向向量
        dx = (x2 - x1) / dis
        dy = (y2 - y1) / dis

        d = 0
        while (d < dis):
            x_new = x1 + d * dx
            y_new = y1 + d * dy
            d += resolution
            
            ox.append(x_new)
            oy.append(y_new)
        
        return ox, oy
        
    def hybrid_a_star_planning(self):
        """
        start: start node
        goal: goal node
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        XY_GRID_RESOLUTION: grid resolution [m]
        YAW_GRID_RESOLUTION: yaw angle resolution [rad]
        """
        self.is_planning = True
        start = [self.gps_data.x, self.gps_data.y, rs.pi_2_pi(self.gps_data.yaw)]
        goal = [self.x_goal, self.y_goal, rs.pi_2_pi(self.theta_goal)]
        ox, oy = self.parking_boundary(goal, self.parking_type)
        ## 函数测试
        # 改变goal目标点的横纵坐标以及航向角，主要是航向角
        # 同时改变泊车类型
        # 把ox，oy还有目标点用matplot函数画图，与test.py类似，观察泊车边界生成是否错误
        # self.msgParkingObstacle = ParkingObstacleInterface()
        # self.msgParkingObstacle.x = ox
        # self.msgParkingObstacle.y = oy
        # 初始化
        
        self.get_logger().info("hybrid a star planning begin!")
        # self.get_logger().info("start: ", start)
        # self.get_logger().info("goal: ", goal)
        tox, toy = ox[:], oy[:]
        hox, hoy = ox[:], oy[:]
        hox = [ihox / XY_GRID_RESOLUTION for ihox in hox]
        hoy = [ihoy / XY_GRID_RESOLUTION for ihoy in hoy]
        obstacle_kd_tree = cKDTree(np.vstack((tox, toy)).T)
        obstacle_kd_tree_h_dp = cKDTree(np.vstack((hox, hoy)).T)
        config = Config(tox, toy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
        start_node = SearchNode(round(start[0] / XY_GRID_RESOLUTION),
                      round(start[1] / XY_GRID_RESOLUTION),
                      round(start[2] / YAW_GRID_RESOLUTION), True,
                      [start[0]], [start[1]], [start[2]], [True], cost=0)
        goal_node = SearchNode(round(goal[0] / XY_GRID_RESOLUTION),
                     round(goal[1] / XY_GRID_RESOLUTION),
                     round(goal[2] / YAW_GRID_RESOLUTION), True,
                     [goal[0]], [goal[1]], [goal[2]], [True])
        openList, closedList = {}, {}
        h_dp = calc_distance_heuristic(
            goal_node.x_list[-1], goal_node.y_list[-1],
            ox, oy, XY_GRID_RESOLUTION, BUBBLE_R, obstacle_kd_tree_h_dp)
        pq = []
        openList[hatools.calc_index(start_node, config)] = start_node
        heapq.heappush(pq, (hatools.calc_cost(start_node, h_dp, config),
                            hatools.calc_index(start_node, config)))
        final_path = None
        # 初始化

        # 搜索过程
        while True:
            if not openList:
                self.get_logger().info("Error: Cannot find path, No open set!")

            cost, c_id = heapq.heappop(pq)
            if c_id in openList:
                current = openList.pop(c_id)
                closedList[c_id] = current
            else:
                continue

            is_updated, final_path = hatools.update_node_with_analytic_expansion(
                current, goal_node, config, ox, oy, obstacle_kd_tree)

            if is_updated:
                self.get_logger().info("Path found!")
                break

            for neighbor in hatools.get_neighbors(current, goal_node, config, ox, oy,
                                        obstacle_kd_tree):
                neighbor_index = hatools.calc_index(neighbor, config)
                if neighbor_index in closedList:
                    continue
                if neighbor_index not in openList \
                        or openList[neighbor_index].cost > neighbor.cost:
                    heapq.heappush(
                        pq, (hatools.calc_cost(neighbor, h_dp, config),
                            neighbor_index))
                    openList[neighbor_index] = neighbor

        path = hatools.get_final_path(closedList, final_path)
        # 搜索过程

        # 发布消息
        # AutoCar verison
        # self.msgGlobalPathPlanning = GlobalPathPlanningInterface()
        # for i in range(len(path.x_list)):
        #     self.msgGlobalPathPlanning.routedata.append(path.x_list[i])
        #     self.msgGlobalPathPlanning.routedata.append(path.y_list[i])

        self.msgLocalPathPlanning = LocalPathPlanningInterface()
        for i in range(len(path.x_list)):
            enu_data_ori = np.array(
                        [path.x_list[i], path.y_list[i], 0.0]
            )
            enu_data_conv = enu_to_gps(enu_data_ori)
            self.msgLocalPathPlanning.longitude.append(enu_data_conv[0])
            self.msgLocalPathPlanning.latitude.append(enu_data_conv[1])
            
            
            self.msgLocalPathPlanning.angle.append((path.yaw_list[i]+math.pi)/2*360)
            if i == len(path.x_list)-1:
                self.msgLocalPathPlanning.speed.append(0.0)
            elif path.direction_list[i] != path.direction_list[i+1]:# or i == len(path.x_list)-1:
                self.msgLocalPathPlanning.speed.append(0.0)
            else:
                if path.direction_list[i] == True:
                    self.msgLocalPathPlanning.speed.append(1.0)
                if path.direction_list[i] == False:
                    self.msgLocalPathPlanning.speed.append(-1.0)

        # enu_data_ori = np.array([
        #                 [path.x_list[0], path.y_list[0], 0], 
        #                 [path.x_list[-1], path.y_list[-1], 0]
        #                 ])
        # enu_data_conv = enu_to_gps(enu_data_ori)
        # self.msgLocalPathPlanning.startpoint = enu_data_conv[0, :2]
        # self.msgLocalPathPlanning.endpoint = enu_data_conv[1, :2]
        # 发布消息

        self.is_finished = True
        self.get_logger().info("hybrid a star planning finished!")
        
    # def pub_callback_global_path_planning(self): # AutoCar version
    def pub_callback_local_path_planning(self):
        """
        回调函数，发布 parking_path_planning_data.
        """
        now_ts = time.time()
        #self.parking_judgment() and 
        if (not self.is_planning and (self.gps_data is not None)):  # 距离满足，并且没有进行过泊车规划
            thread = threading.Thread(target=self.hybrid_a_star_planning)
            thread.setDaemon(True)
            thread.start()
        if (not self.is_finished): # 等待混合A*规划结束
            self.get_logger().info("waiting for hybrid a star planning!")
        else:
            self.get_logger().info("publish topic local_path_planning_data!")

            # AutoCar version
            # self.msgGlobalPathPlanning.process_time = time.time() - now_ts
            # self.pubGlobalPathPlanning.publish(self.msgGlobalPathPlanning)

            self.msgLocalPathPlanning.timestamp = time.time()
            self.msgLocalPathPlanning.process_time = time.time() - now_ts
            self.pubLocalPathPlanning.publish(self.msgLocalPathPlanning)
    
    # def pub_callback_parking_obstacle_data(self):
    #     """
    #     回调函数，发布 parking_obstacle_data.
    #     """
    #     if self.msgParkingObstacle is not None:
    #         self.msgParkingObstacle.timestamp = time.time()
    #         self.pubParkingObstacle.publish(self.msgParkingObstacle)

    
    def pos_callback(self, msg):
        """
        存入车辆信息
        """
        self.gps_data = GpsData(0,0,0,0)
        gps_data_ori = np.array([msg.longitude, msg.latitude, 0.0])
        gps_data_ori = np.array([117.16642035, 39.10524115, 0.0])
        
        print("original_nowpoint =", gps_data_ori)
        
        gps_data_conv = gps_to_enu(gps_data_ori)
        self.gps_data.x = round(gps_data_conv[0],2)
        self.gps_data.y = round(gps_data_conv[1],2)
        self.gps_data.z = round(gps_data_conv[2],2)
        print("transformed_nowpoint_x =", self.gps_data.x)
        print("transformed_nowpoint_y =", self.gps_data.y)  
        self.gps_data.yaw = rs.pi_2_pi(msg.yaw/180*math.pi) #yaw [-pi, pi]
        # self.gps_data.yaw = rs.pi_2_pi(181.029/180*math.pi) #yaw [-pi, pi]

        # AutoCar version
        # self.gps_data = GpsInterface()
        # self.gps_data.x = msg.x
        # self.gps_data.y = msg.y
        # self.gps_data.yaw = rs.pi_2_pi(msg.yaw/180*math.pi) #yaw [-pi, pi]
        
def main():
    rclpy.init()
    rosNode = ParkingPathPlanning()
    rclpy.spin(rosNode)
    rclpy.shutdown()
