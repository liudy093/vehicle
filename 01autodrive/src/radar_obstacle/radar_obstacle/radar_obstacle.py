"""
Author: chen fang
CreateTime: 2023/3/10 下午4:36
File: radar_obstacle.py
Description: 发布毫米波雷达
"""
import sys
import os

ros_path = '/opt/ros/foxy/lib/python3.8/site-packages'
sys.path.append(ros_path)

ros_node_name = "radar_obstacle"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

import rclpy
from rclpy.node import Node
import numpy as np
import time
import yaml
from easydict import EasyDict
import math
from car_interfaces.msg import *
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from .ARS_radar import ARSRadar
# from .delphi_radar import DelphiRadar


class RadarObstacle(Node):
    def __init__(self, name):
        super().__init__(name)
        root_path = os.getcwd() + '/src/radar_obstacle/radar_obstacle/'  # 根目录
        # ======================================================== #
        # 从config文件中读取超参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f)
        self.config = EasyDict(config)

        # ======================================================== #
        # 选择使用那个毫米波雷达,并进行初始化
        witch_rader = self.config.WITCH_RADAR

        if witch_rader == 0:  # 德尔福
            self.radar = DelphiRadar(self.config)
            print("德尔福")
        elif witch_rader == 1:  # 德国大陆
            self.radar = ARSRadar(self.config)
            print("德国大陆")

        # ======================================================== #
        # 订阅task_id
        # self.plannerVis_subscriber = self.create_subscription(PlannerVis, "planner_vis_data", self.plannerVis_callback,
        #                                                       10)
        # self.plannerVis = None

        # ======================================================== #
        # 发布话题
        self.pubRadarState = self.create_publisher(RadarStateInterface, 'radar_state_data', 10)
        self.pubRadarObstacle = self.create_publisher(RadarObstacleInterface, 'radar_obstacle_data', 10)
        self.pubRadarGrid = self.create_publisher(GridCells, 'radar_vis', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)

        # ======================================================== #
        # 监听器
        self.radar.rader_msg_recieve_flag = 1  # 默认状态错误，需要在代码中不断的改为正确
        self.timerRidarOri = self.create_timer(1, self.radar_state_monitor)

    def timer_callback(self):
        """
        发布毫米波数据
        """
        # ======================================================== #
        # 获得全部目标
        objs = self.radar.obj.copy()
        # print('objs::',objs)

        # ======================================================== #
        # 坐标转换
        objs, objs_vis = self.obstacle_coord_tf(objs)
        # print('objs_vis::',objs_vis)

        # ======================================================== #
        # 发布 radar_state_data 话题
        msgRadarState = RadarStateInterface()
        msgRadarState.timestamp = time.time()  # 时间戳
        msgRadarState.id = 0x01  # 毫米波雷达ID
        msgRadarState.state = self.radar.radar_state  # 设备状态，0：状态正常，1：状态异常
        msgRadarState.error = self.radar.radar_error  # 错误码；1:通信错误，2:内部错误，3:阻塞错误，4:过热错误
        self.pubRadarState.publish(msgRadarState)

        # ======================================================== #
        # 发布 radar_obstacle_data 话题
        num = len(objs)
        objs = objs.flatten().tolist()  # 二维numpy --> 一维numpy --> 一维list
        msgRadarObstacle = RadarObstacleInterface()
        msgRadarObstacle.timestamp = time.time()  # 时间戳
        msgRadarObstacle.id = 0  # 毫米波雷达ID
        msgRadarObstacle.number = num  # 障碍物数量
        msgRadarObstacle.obstacledata = objs  # 障碍物数据 [category, l, w, h, x, y, z, v, latv]
        msgRadarObstacle.process_time = float(self.radar.time)  # 进程处理时间
        self.pubRadarObstacle.publish(msgRadarObstacle)

        # ======================================================== #
        # 可视化
        self.vis_obstacle_by_grid(objs_vis)

    def radar_state_monitor(self):
        """
        毫米波状态监听器
        """
        if self.radar.rader_msg_recieve_flag == 0:
            self.radar.rader_msg_recieve_flag = 1
        else:

            print('--------------------error-----------------------')
            msgRadarState = RadarStateInterface()
            msgRadarState.timestamp = time.time()  # 时间戳
            msgRadarState.id = 0x01  # 毫米波雷达ID
            msgRadarState.state = 1  # 设备状态，0：状态正常，1：状态异常
            msgRadarState.error = 1  # 错误码；1:通信错误，2:内部错误，3:阻塞错误，4:过热错误
            self.pubRadarState.publish(msgRadarState)
            # print(msgRadarState)

    def obstacle_coord_tf(self, objs):
        """
        坐标转换，从前左上转为右前上
        """
        objs_vis = objs.copy()
        for i, obj in enumerate(objs):
            x_ = obj[4] * math.cos(self.config.RADAR.RY) - obj[5] * math.sin(self.config.RADAR.RY) + \
                 self.config.RADAR.DX
            y_ = obj[4] * math.sin(self.config.RADAR.RY) + obj[5] * math.cos(self.config.RADAR.RY) + \
                 self.config.RADAR.DY

            objs[i, 4] = -y_
            objs[i, 5] = x_
            objs_vis[i, 4] = x_
            objs_vis[i, 5] = y_
        return np.array(objs), np.array(objs_vis)

    def vis_obstacle_by_grid(self, objs):
        """
        通过grid形式可视化
        """
        cells = GridCells()
        cells.header.frame_id = 'rslidar'
        cells.cell_width = 0.2
        cells.cell_height = 0.2

        for obj in objs:
            if obj[5] != 0 and  obj[4] != 0 and obj[6] != 0:
                print("objs",obj)
                point = Point()
                point.x = float(obj[4])
                point.y = float(obj[5])
                point.z = float(obj[6])
                cells.cells.append(point)
            
        self.pubRadarGrid.publish(cells)

    def plannerVis_callback(self, msg):
        """
        存入可视化信息
        """
        # self.plannerVis = msg


def main():
    rclpy.init()
    rosNode = RadarObstacle(name='radar_obstacle')
    rclpy.spin(rosNode)
    rclpy.shutdown()


if __name__ == '__main__':
    pass
