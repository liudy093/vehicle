"""
Author: chen fang
CreateTime: 2024/11/12 下午9:35
File: parking_planning.py
Description: 
"""

import sys
import os
import time
import copy
import yaml
from easydict import EasyDict
import numpy as np
import array
import math
import json
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "parking_planning"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))
from rs import rs_func

import tjitools


class ParkingPlanning(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # 定义发布器
        self.pubLocalPathPlanning = self.create_publisher(LocalPathPlanningInterface, "local_path_planning_data", 10)
        self.timerLocalPathPlanning = self.create_timer(0.1, self.pub_callback_local_path_planning)

        # 定义接收器
        self.subGPS = self.create_subscription(GpsInterface, 'gps_data', self.sub_callback_gps, 10)
        self.gps = None

        # 从config文件中读取超参数
        root_path = os.getcwd() + "/src/parking_planning/parking_planning/"  # 根目录
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f, Loader=yaml.FullLoader)
        self.yaml_data = EasyDict(config)

        # 读取全局地图
        self.glbPath = self.read_map(root_path + self.yaml_data.map_path)  # [N,5]

        # 停车点信息
        self.parking_point_x = self.yaml_data.parking_point_x
        self.parking_point_y = self.yaml_data.parking_point_y
        self.parking_point_yaw = self.yaml_data.parking_point_yaw  # 角度（单位：度）
        self.rs_track_cls = self.yaml_data.rs_track_cls  # rs路径的类型(0 or 1)
        self.rs_track_step = self.yaml_data.rs_track_step  # rs路径的步长（单位：米）
        self.parking_point_index = -1
        self.parking_track_xy = None  # 泊车轨迹
        self.parking_track_lonlat = None  # 泊车轨迹
        self.parking_speed_1 = self.yaml_data.parking_speed_1  # 靠近泊车点的速度
        self.parking_speed_2 = self.yaml_data.parking_speed_2  # 倒车的速度

        # 泊车状态
        self.stage = 0
        self.waitstep = 30  # 倒车前等待3秒

        # 可视化
        if self.yaml_data.vis:
            plt.ion()  # 开启交互模式
            self.fig, self.ax = plt.subplots()  # 创建图表
            self.ax.set_xlim(-30, 30)  # 设置X轴范围
            self.ax.set_ylim(-30, 30)  # 设置Y轴范围
            self.scatter_plot = None  # 创建一个空的散点图对象
            self.start_point = None
            self.end_point = None

    def pub_callback_local_path_planning(self):
        print('========================================')
        msgLocalPathPlanning = LocalPathPlanningInterface()
        now_ts = time.time()

        if self.gps is None:
            return

        # 车辆当前的坐标和航向
        car_lon = self.gps.longitude
        car_lat = self.gps.latitude
        car_yaw = self.gps.yaw
        car_x = self.gps.x
        car_y = self.gps.y

        # 确定车辆当前在全局地图的哪个点
        x_ = self.glbPath[:, 3] - car_x
        y_ = self.glbPath[:, 4] - car_y
        err_ = x_ ** 2 + y_ ** 2
        car_idx = err_.argmin()
        print("Car_Point_Index:", car_idx)

        # 确定停车点在哪个点
        if self.parking_point_index == -1:
            x_ = self.glbPath[:, 3] - self.parking_point_x
            y_ = self.glbPath[:, 4] - self.parking_point_y
            err_ = x_ ** 2 + y_ ** 2
            self.parking_point_index = err_.argmin()
        else:
            print("Parking_Point_Index:", self.parking_point_index)

        # --------------- 靠近轨迹点  --------------#
        if self.stage == 0:
            print('--------------- 靠近轨迹点  --------------')
            # 如果超过停车点20m，开始泊车
            if self.parking_point_index != -1 and \
                    (car_idx - self.parking_point_index) > int(self.yaml_data.forward_dis / 0.5):
                self.stage = 1
            # 发布轨迹
            msgLocalPathPlanning = self.get_msgLocalPathPlanning(msgLocalPathPlanning,
                                                                 car_idx,
                                                                 self.glbPath,
                                                                 self.parking_speed_1,
                                                                 car_lon,
                                                                 car_lat)

        # --------------- 倒车前等待  --------------#
        if self.stage == 1:
            print('--------------- 倒车前等待  --------------')
            if self.waitstep < 0:
                self.stage = 2
            else:
                self.waitstep = self.waitstep - 1
                print('waitstep: ', self.waitstep)
                
            # 规划轨迹（轨迹只规划一次）
            if self.parking_track_xy is None:
                paremeters = np.zeros(8).astype(np.float32)
                paremeters[0] = car_x  # 起点x坐标
                paremeters[1] = car_y  # 起点y坐标
                paremeters[2] = car_yaw  # 起点航向角（角度）
                paremeters[3] = self.parking_point_x  # 终点x坐标
                paremeters[4] = self.parking_point_y  # 终点y坐标
                paremeters[5] = self.parking_point_yaw  # 终点航向角（角度）
                paremeters[6] = self.rs_track_step  # 轨迹步长
                paremeters[7] = self.rs_track_cls  # 轨迹类型
                self.parking_track_xy = np.array(rs_func.get_rs_track(paremeters))  # [N,3]
                self.parking_track_lonlat = tjitools.enu_to_gps(self.parking_track_xy)

            # 发布轨迹
            msgLocalPathPlanning = self.get_msgLocalPathPlanning(msgLocalPathPlanning,
                                                                 car_idx,
                                                                 self.glbPath,
                                                                 0.0,
                                                                 car_lon,
                                                                 car_lat)

        # --------------- 开始泊车  --------------#
        if self.stage == 2:
            print('--------------- 开始泊车  --------------')
            # 跟轨迹倒车
            x_ = self.parking_track_xy[:, 0] - car_x
            y_ = self.parking_track_xy[:, 1] - car_y
            err_ = x_ ** 2 + y_ ** 2
            now_idx = err_.argmin()

            # 结束泊车
            err_dis = (self.parking_point_x-car_x)**2 + (self.parking_point_y-car_y)**2
            err_yaw = (self.parking_point_yaw-car_yaw)
            if err_yaw > 180:
                err_yaw -= 360
            if err_yaw < -180:
                err_yaw += 360
                
            
            print("dis:",err_dis)
            print("yaw:",err_yaw)
            if err_dis<0.4*0.4 and abs(err_yaw) < 10:
                self.stage = 3

            # 发布轨迹
            msgLocalPathPlanning = self.get_msgLocalPathPlanning(msgLocalPathPlanning,
                                                                 now_idx,
                                                                 self.parking_track_lonlat,
                                                                 self.parking_speed_2,
                                                                 car_lon,
                                                                 car_lat)

        # --------------- 泊车结束,停车  --------------#
        if self.stage == 3:
            print('--------------- 停车  --------------')
            # 发布轨迹
            msgLocalPathPlanning = self.get_msgLocalPathPlanning(msgLocalPathPlanning,
                                                                 len(self.parking_track_lonlat - 4),
                                                                 self.parking_track_lonlat,
                                                                 0.0,
                                                                 car_lon,
                                                                 car_lat)

        # -------------------------------#
        # 可视化相关
        if self.yaml_data.vis:
            # 绘制前向轨迹
            if self.stage == 0:
                lon_ = np.array(msgLocalPathPlanning.longitude)
                lat_ = np.array(msgLocalPathPlanning.latitude)
                yaw_ = np.zeros_like(lon_)
                xy = np.column_stack((lon_, lat_, yaw_))
                xy = tjitools.gps_to_enu(xy)
                x_values = xy[:, 0] - car_x
                y_values = xy[:, 1] - car_y
            else:
                x_values = self.parking_track_xy[:, 0] - car_x
                y_values = self.parking_track_xy[:, 1] - car_y

            # 更新图像中的散点图
            if self.scatter_plot is None:
                self.scatter_plot = self.ax.scatter(x_values, y_values, c='k', s=10)  # 初始化散点图
                self.start_point = self.ax.scatter(0, 0, c='g', s=40)
                self.end_point = self.ax.scatter(self.parking_point_x - car_x, self.parking_point_y - car_y,
                                                 c='r', s=40)
            else:
                self.scatter_plot.set_offsets(np.c_[x_values, y_values])  # 更新散点图数据
                self.start_point.set_offsets(np.c_[0, 0])
                self.end_point.set_offsets(np.c_[self.parking_point_x - car_x, self.parking_point_y - car_y])

            plt.draw()  # 重新绘制图像
            plt.pause(0.1)  # 刷新图像

    def read_map(self, map_path):
        # 读取地图
        with open(map_path, 'r', encoding='utf-8') as fp:
            json_data = json.load(fp)

        # 读取数据
        map = json_data['lane0']['data']
        path = []
        for data in map:
            a = [data['longitude'], data['latitude'], data['h'], data['x'], data['y']]
            path.append(a)
        path = np.array(path)
        return path

    def get_msgLocalPathPlanning(self, msgLocalPathPlanning, now_idx, path_lonlat, speed, start_lon, start_lat):
        """
        得到需要发布的轨迹
        """
        if now_idx + 80 < len(path_lonlat):
            loc_path_end = path_lonlat[now_idx + 80, 0:2]
            loc_path_lon = path_lonlat[now_idx:now_idx + 80, 0]
            loc_path_lat = path_lonlat[now_idx:now_idx + 80, 1]
            loc_path_ang = path_lonlat[now_idx:now_idx + 80, 2]
        else:
            loc_path_end = path_lonlat[-1, 0:2]
            loc_path_lon = path_lonlat[now_idx:, 0]
            loc_path_lat = path_lonlat[now_idx:, 1]
            loc_path_ang = path_lonlat[now_idx:, 2]
            
                
            pad_len = 80 - len(loc_path_lon)
            
            if now_idx >= len(path_lonlat)-1:
                loc_path_lon = path_lonlat[-1:, 0]
                loc_path_lat = path_lonlat[-1:, 1]
                loc_path_ang = path_lonlat[-1:, 2]
                pad_len = 80-1
            
            print("len: ",len(path_lonlat))
            print("index: ",now_idx)
            print("pad_len :",pad_len)
            
            loc_path_lon = np.pad(loc_path_lon, pad_width=(0, pad_len), mode='edge')
            loc_path_lat = np.pad(loc_path_lat, pad_width=(0, pad_len), mode='edge')
            loc_path_ang = np.pad(loc_path_ang, pad_width=(0, pad_len), mode='edge')

        loc_path_end = path_lonlat[-1, 0:2]
        loc_path_vel = speed * np.ones_like(loc_path_lon)
        msgLocalPathPlanning.startpoint = array.array('d', [start_lon, start_lat])
        msgLocalPathPlanning.endpoint = array.array('d', loc_path_end)
        msgLocalPathPlanning.longitude = array.array('d', loc_path_lon)
        msgLocalPathPlanning.latitude = array.array('d', loc_path_lat)
        msgLocalPathPlanning.angle = array.array('f', loc_path_ang)
        msgLocalPathPlanning.speed = array.array('f', loc_path_vel)
        msgLocalPathPlanning.process_time = time.time()
        
        self.pubLocalPathPlanning.publish(msgLocalPathPlanning)

        return msgLocalPathPlanning

    def sub_callback_gps(self, GpsInterface):
        self.gps = GpsInterface


def main():
    rclpy.init()
    rosNode = ParkingPlanning()
    rclpy.spin(rosNode)
    rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    rosNode = ParkingPlanning()
    rclpy.spin(rosNode)
    rclpy.shutdown()
