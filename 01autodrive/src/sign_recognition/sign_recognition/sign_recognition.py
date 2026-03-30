#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: sign_recognition.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
ros_node_name = "sign_recognition"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "detectSign"))

import time
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *
from cv_bridge import CvBridge, CvBridgeError
from car_interfaces.msg import CameraObstacleInterface, CameraOriInterface
from sensor_msgs.msg import Image
from detectSign.object_detect import detect_init, detect_ros
from detectSign.utils.length_info import result_to_msg



X_DIS, Y_DIS, Z_DIS = 0.6, 0.0, 0.3  # 相机坐标系到雷达坐标系参数

class SignRecognition(Node):
    def __init__(self, ros_node_name):
        super().__init__(ros_node_name)
        # 输出日志信息, 提示开始完成话题发布
        self.get_logger().info("Node: sign_recognition  publisher: sign_recognition_data")
        
        # 初始化网络模型
        self.model, self.sort_tracker, self.opt, self.class_names = detect_init(imgsz=(640,480))

        # define publishers
        self.pubSignRecognition = self.create_publisher(SignRecognitionInterface, "sign_recognition_data", 10)
        self.timerSignRecognition = self.create_timer(0.1, self.pub_callback_sign_recognition)

        # define subscribers
        self.subCameraOri = self.create_subscription(CameraOriInterface, "camera_ori_data", self.sub_callback_camera_ori, 1)

        # rviz可视化的结果
        self.pubSignRecognitionRviz = self.create_publisher(Image, 'sign_recognition_rviz', 10)

        # init image
        self.image = np.zeros((640, 480, 3), dtype=np.uint8)
        


    def sub_callback_camera_ori(self, msgCameraOri:CameraOriInterface):
        """
        Callback of subscriber, subscribe the topic camera_ori_data.
        :param msgCameraOri: The message heard by the subscriber.
        """
        bridge = CvBridge()
        self.image = msgCameraOri.imagedata
        # get image
        self.image = bridge.imgmsg_to_cv2(self.image, "bgr8")

    @staticmethod
    def update_sign_recognition(msg, process_time, signdata, vis_img):
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
        msg.signnumber = len(signdata) // 9 if len(signdata) > 0 else 0
        msg.signdata = signdata
        msg.resultimage = vis_img
        msg.process_time = process_time


    def pub_callback_sign_recognition(self):
        """
        Callback of publisher (timer), publish the topic sign_recognition_data.
        :param None.
        """
        # 图像处理
        t0 = time.time()

        with torch.no_grad():
            sign_info, vis_img = detect_ros(self.image, self.model, self.sort_tracker, 
                                    self.opt, self.class_names)
        sign_info = np.array(sign_info)
        

        # 将结果处理为一维数组
        if sign_info.shape[0] != 0:
            signdata = result_to_msg(self.image, sign_info[:, 0:7])
        else:
            signdata = []

        # cv 图像转 imgmsg
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg(vis_img, "bgr8")

        # define msg
        msgSignRecognition = SignRecognitionInterface()
    
        # 坐标转化 将得到的坐标转化
        result = np.array(signdata.copy()).reshape(-1, 9)
        result[:, [4, 5]] = result[:, [5, 4]]
        result[:, 4] = -result[:, 4] + Y_DIS
        result[:, 5] = result[:, 5] + X_DIS
        result[:, 6] = result[:, 6] + Z_DIS
        result = sum(result.tolist(), [])
        print(result)
        process_time = time.time() - t0
    
        self.update_sign_recognition(msgSignRecognition, process_time, result, img)
        self.pubSignRecognition.publish(msgSignRecognition)
        self.pubSignRecognitionRviz.publish(img)



def main():
    rclpy.init()
    rosNode = SignRecognition(ros_node_name=ros_node_name)
    rclpy.spin(rosNode)
    rclpy.shutdown()
