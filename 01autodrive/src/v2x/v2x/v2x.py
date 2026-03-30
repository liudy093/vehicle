#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: YK
@说明: V2X    upgrade time: 6.19 12:20
"""
import os, time
import sys
ros_node_name = "v2x"
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import socket
import json
from datetime import datetime
import threading
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from car_interfaces.msg import*
import struct
import datetime
import binascii
import logging

class V2X(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2节点父类初始化

        ############################ 初始化默认参数 #################################
        self.car_id = 2
        
        self.planner2v = None

        # 主机IP
        self.HOST = '192.168.20.224'
        self.PORT = 30301
        # OBU IP 
        self.OBU_HOST = '192.168.20.199'
        self.OBU_PORT = 30300
        
        print(self.HOST)
        self.server = socket.socket(type=socket.SOCK_DGRAM)
        # self.server.setblocking(False) #非阻塞模式，读不到数据
        self.server.bind((self.HOST, self.PORT))



        # ############################ 订阅话题，检查话题名称 #########################
        self.gps_subscriber = self.create_subscription(GpsInterface, "gps_data", self.gps_callback, 10)
        self.car_ori_subscriber = self.create_subscription(CarOriInterface, "car_ori_data", self.car_ori_callback, 10)
        self.platoon_subscriber = self.create_subscription(HmiPlatoonInterface, "platoon_state_data", self.platoon_callback, 10)
        # 获取当前车辆状态
        self.subFusion = self.create_subscription(
            FusionInterface,
            'fusion_data',
            self.sub_callback_fusion,
            10)
        # ############################ 发布话题，检查话题名称 #########################
        self.v2xPlanner_publisher = self.create_publisher(VtxInterface, 'v2x_to_planner_data', 10)
        self.timerv2xPlanner = self.create_timer(0.1, self.v2x_send_info)
        
        
        
        
        
        # 配置自定义日志记录器
        self.v2x_logger = logging.getLogger('v2x_logger')
        self.v2x_logger.setLevel(logging.INFO)
        fh = logging.FileHandler('v2x_log_file.log')
        fh.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        self.v2x_logger.addHandler(fh)


        # 借用VtxInterface的结构来存储本车信息
        self.ego_car_msg=self.Init_msg()
        

        self.v2xPlanner_thread = threading.Thread(target=self.v2xPlanner_callback)
        self.v2xPlanner_thread.start()

        


    def platoon_callback(self, msg):
        self.ego_car_msg.platoon_id=msg.platoon_id
        
    def car_ori_callback(self, msg):
        self.ego_car_msg.speed=msg.carspeed
        
        
        
    def gps_callback(self, msgGps):
        self.ego_car_msg.longitude=msgGps.longitude
        self.ego_car_msg.latitude=msgGps.latitude
        self.ego_car_msg.yaw=msgGps.yaw

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        self.ego_car_msg.acceleration = msgFusion.ax

    # 定时发送数据
    def v2x_send_info(self):
        
        outData = {
            "timestamp":int(time.time()),
            "car_id":self.car_id,
            "platoon_id":self.ego_car_msg.platoon_id,
            "longitude":self.ego_car_msg.longitude,
            "latitude":self.ego_car_msg.latitude,
            "speed":self.ego_car_msg.speed,
            "yaw":self.ego_car_msg.yaw,
            "acceleration":self.ego_car_msg.acceleration
        }
        OUB_addr = (self.OBU_HOST,self.OBU_PORT)
        self.server.sendto(json.dumps(outData).encode('gbk'),OUB_addr)

        print("---------- V2X向OBU发送数据 ----------:\n"+json.dumps(outData))
        # self.v2x_logger.info(json.dumps(outData))
        print("车辆编队ID = ",self.ego_car_msg.platoon_id)
        print("车辆速度 = ",self.ego_car_msg.speed)
        print("车辆经度 = ",self.ego_car_msg.longitude)
        print("车辆纬度 = ",self.ego_car_msg.latitude)
        print("车辆航向角 = ",self.ego_car_msg.yaw)
          
        



    # OBU消息上报
    def v2xPlanner_callback(self):
        """
        处理，并发布给 planner
        """
        #print("into ==========================")
        while True:
            
            data, address = self.server.recvfrom(9000)
            data = data.decode("UTF-8")
            
            
            print("--------------收到OBU消息 ----------------")
            # path = os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name)
            # with open(path+'raw_data.txt','a') as f:
            #     f.write('time={}   data={}'.format(time.time(),data))
            #     f.write('\n')
            data_dict = json.loads(data)
            msg = VtxInterface()
            msg.timestamp=float(data_dict['timestamp'])
            msg.platoon_id=data_dict['platoon_id']
            msg.car_id=data_dict['car_id']
            msg.longitude=data_dict['longitude']
            msg.latitude=data_dict['latitude']
            msg.speed=data_dict['speed']
            msg.yaw=data_dict['yaw']
            msg.acceleration=data_dict['acceleration']
            self.v2xPlanner_publisher.publish(msg)    
            print("车辆编队ID = ",msg.platoon_id)
            print("车辆速度 = ",msg.speed)
            print("车辆经度 = ",msg.longitude)
            print("车辆纬度 = ",msg.latitude)
            print("车辆航向角 = ",msg.yaw)
            print("车辆加速度 = ",msg.acceleration)





    def Init_msg(self):
        msg = VtxInterface()
        msg.timestamp=0.0
        msg.platoon_id=-1
        msg.car_id=-1
        msg.longitude=-1.0
        msg.latitude=-1.0
        msg.speed=-1.0
        msg.yaw=-1.0
        msg.acceleration=-1.0
        return msg


        
def main():                                           # ROS2节点主入口main函数
    rclpy.init()                                      # ROS2 Python接口初始化
    rosnode = V2X(name=ros_node_name)                 # 创建ROS2节点对象并进行初始化
    rclpy.spin(rosnode)                               # 循环等待ROS2退出
    rclpy.shutdown()                                  # 关闭ROS2 Python接口