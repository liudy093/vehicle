#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: lane_recognition.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import os
import sys
ros_node_name = "lane_recognition"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "detectLane"))

import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import torch
from sensor_msgs.msg import Image
from car_interfaces.msg import LaneRecognitionInterface, CameraOriInterface
from detectLane.detect_lane import lane_detect_init, lane_detect_main
from detectLane.utils.calculate_dis import dis_lane


#默认车道宽度
DEFAULT_WIDTH_LANE = 3

class LaneRecognition(Node):
    def __init__(self, name):
        super().__init__(name)
        # 输出日志信息, 提示开始完成话题发布
        self.get_logger().info("Node: lane_recognition    publisher: lane_recognition_data")

        #初始化图像 网络
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.net, self.img_transforms = lane_detect_init()
    
        # define publishers
        self.pubLaneRecognition = self.create_publisher(LaneRecognitionInterface, 'lane_recognition_data', 10)
        self.pubLaneRecognitionRviz = self.create_publisher(Image, 'lane_recognition_rviz', 10)
        self.timerLaneRecognition = self.create_timer(0.1, self.pub_callback_lane_recognition)

        # define subscribers
        self.subCameraOri = self.create_subscription(CameraOriInterface, 'camera_ori_data', self.sub_callback_camera_ori, 10)



    def sub_callback_camera_ori(self, msgCameraOri: CameraOriInterface):
        """
        Callback of subscriber, subscribe the topic 'camera_ori_data'.
        :param msgCameraOri: The message heard by the subscriber.
        """
        bridge = CvBridge()
        self.image = msgCameraOri.imagedata

        # get image
        self.image = bridge.imgmsg_to_cv2(self.image, "bgr8")


    @staticmethod
    def update_lane_recognition(msg, process_time, vis_img, center_offset):
        """
        给 msg 添加数据
        :params msg: 是实例化的话题
        :params process_time: 是处理时间
        :params object_info: 是障碍物信息列表
        :params vis_img: 是可视化后的图片
        """

        msg.timestamp = time.time()
        msg.id = 0
        msg.centeroffset = center_offset
        msg.resultimage = vis_img
        msg.process_time = process_time


    def pub_callback_lane_recognition(self):
        """
        Callback of publisher (timer), publish the topic 'lane_recognition_data'.
        :param None.
        """
        t0 = time.time()
        with torch.no_grad():
            vis_lane, point_lane = lane_detect_main(self.net, self.image, self.img_transforms)
            center_offset = dis_lane(point_lane, self.image)  # 不确定

        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg(vis_lane)

        process_time = time.time() - t0

        # define msg
        msgLaneRecognition = LaneRecognitionInterface()
        self.update_lane_recognition(msgLaneRecognition, process_time, img, center_offset)

        # publish info
        self.pubLaneRecognition.publish(msgLaneRecognition)
        self.pubLaneRecognitionRviz.publish(img)


def main():
    rclpy.init()
    rosNode = LaneRecognition(name=ros_node_name)
    rclpy.spin(rosNode)
    rclpy.shutdown()