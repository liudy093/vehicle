#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: light_recognition.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
ros_node_name = "light_recognition"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "detectLight"))
import time
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *
from cv_bridge import CvBridge, CvBridgeError
from car_interfaces.msg import CameraObstacleInterface, CameraOriInterface
from sensor_msgs.msg import Image
from detectLight.object_detect import detect_init, detect_ros
from detectLight.utils.length_info import result_to_msg


#相机的xyz坐标系加上下面三个参数变为雷达坐标系
X_DIS, Y_DIS, Z_DIS = -0.6, 0.0, 0.3  # 相机坐标系到雷达坐标系参数

class LightRecognition(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubLightRecognition = self.create_publisher(LightRecognitionInterface, "light_recognition_data", 10)
        self.timerLightRecognition = self.create_timer(0.1, self.pub_callback_light_recognition)

        # 初始化网络模型
        self.model, self.sort_tracker, self.opt, self.class_names, self.data_transforms, self.signmodel = detect_init(imgsz=640)

        # define subscribers
        self.subCameraOri = self.create_subscription(CameraOriInterface, "camera_ori_data", self.sub_callback_camera_ori, 1)
        self.subMatchMap = self.create_subscription(MatchMapInterface, "match_map_data", self.sub_callback_match_map, 1)
        #初始化subMatchMap中的变量
        self.cross = 0
        self.road = 1
        # rviz可视化的结果
        self.pubLightRecognitionRviz = self.create_publisher(Image, 'camera_light_rviz', 10)

        # init image
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)

    def sub_callback_camera_ori(self, msgCameraOri:CameraOriInterface):
        """
        Callback of subscriber, subscribe the topic camera_ori_data.
        :param msgCameraOri: The message heard by the subscriber.
        """
        bridge = CvBridge()
        self.image = msgCameraOri.imagedata
        # get image
        self.image = bridge.imgmsg_to_cv2(self.image, "bgr8")

    def sub_callback_match_map(self, msgMatchMap:MatchMapInterface):
        """
        Callback of subscriber, subscribe the topic match_map_data.
        :param msgMatchMap: The message heard by the subscriber.
        """
        # self.cross = msgMatchMap.cross
        # self.road = msgMatchMap.special_road
        pass

    @staticmethod
    def update_light_recognition(msg, process_time, lightdata, vis_img):
        """
        给 msg 添加数据
        :params msg: 是实例化的话题
        :params process_time: 是处理时间
        :params object_info: 是障碍物信息列表
        :params vis_img: 是可视化后的图片
        """

        # load msg info
        msg.id = 0
        msg.timestamp = time.time()
        msg.lightdata = lightdata
        msg.resultimage = vis_img
        msg.process_time = process_time
    
    def pub_callback_light_recognition(self):
        """
        Callback of publisher (timer), publish the topic light_recognition_data.
        :param None.
        """
        # 图像处理
        t0 = time.time()
        if self.cross == 0  and self.road == 1:
            with torch.no_grad():
                light_info, vis_img = detect_ros(self.image, self.model, self.sort_tracker, 
                                        self.opt, self.class_names, self.data_transforms, self.signmodel)
            light_info = np.array(light_info)
            # 将结果处理为一维数组
            if light_info.shape[0] != 0:
                lightdata = result_to_msg(self.image, light_info[:, 0:7])
            else:
                lightdata = []
            # cv 图像转 imgmsg
            bridge = CvBridge()
            img = bridge.cv2_to_imgmsg(vis_img, "bgr8")

            # 坐标转化 将得到的坐标转化
            result = np.array(lightdata.copy()).reshape(-1, 6)
            result[:, [0, 1]] = result[:, [1, 0]]
            result[:, 0] = -result[:, 0] + Y_DIS
            result[:, 1] = result[:, 1] + X_DIS
            result[:, 2] = result[:, 2] + Z_DIS
            result = sum(result.tolist(), [])
        else:
            result = [0.0,0.0,0.0,0.0,0.0,0.0]
            bridge = CvBridge()
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            img = bridge.cv2_to_imgmsg(img, "bgr8")
        process_time = time.time() - t0
        print(result)
        msgLightRecognition = LightRecognitionInterface()
        self.update_light_recognition(msgLightRecognition, process_time, result, img)
        self.pubLightRecognition.publish(msgLightRecognition)
        self.pubLightRecognitionRviz.publish(img)



def main():
    rclpy.init()
    rosNode = LightRecognition()
    rclpy.spin(rosNode)
    rclpy.shutdown()
