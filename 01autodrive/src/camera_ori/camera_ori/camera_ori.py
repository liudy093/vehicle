#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: camera_ori.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import os
import sys
import time
ros_node_name = "camera_ori"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "MvImport"))

import time
import cv2
import copy
import rclpy
import numpy as np
import tjitools as tji
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from car_interfaces.msg import CameraOriInterface, CameraStateInterface
# from MvImport.HKCameraAPI import HIKCamera


class CameraOri(Node):
    def __init__(self, name):
        super().__init__(name)
        # 输出日志信息, 话题发布
        self.get_logger().info("Node: camera_ori  publisher: camera_state_data, camera_ori_data")

        # 配置选项
        self.ret = False  # 初始相机状态
        self.camera_state = 1  # 可选0,1,2。为0时使用视频进行测试；为1时使用usb相机；为2时使用网口相机。
        self.is_out_video = True
        self.out_image_size = (640, 480)  # 节点发送的图片的大小

        if self.is_out_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            local_time = time.localtime()
            out_video_name = f"{local_time.tm_yday:4d}-{local_time.tm_hour:2d}-{local_time.tm_min:2d}-{local_time.tm_sec:2d}.mp4"
            self.out_video = cv2.VideoWriter('/home/nvidia/camera_video/'+out_video_name, fourcc, 10, (1440, 1080))

        # load video
        self.video_path = os.path.join(os.getcwd(), "src/utils/camera_videos", "2023_02_04_17_11_35.avi")
        self.load_video_camera(self.camera_state, self.video_path)  # 产生相机对象 self.cap

        # 定义相机状态的发布者和定时器
        self.pubCameraState = self.create_publisher(CameraStateInterface, 'camera_state_data', 10)
        self.timerCameraState = self.create_timer(0.1, self.pub_callback_camera_state)

        # 定义相机图像的发布者和定时器
        self.pubCameraOri = self.create_publisher(CameraOriInterface, 'camera_ori_data', 10)
        self.pubCameraOriRviz = self.create_publisher(Image, 'camera_ori_rviz', 10)
        self.timerCameraOri = self.create_timer(0.1, self.pub_callback_camera_ori)
    
        tji.ros_log(self.get_name(), 'Start Node:%s'%self.get_name())

    @staticmethod
    def update_camera_state(msg, ret):
        """
        给msg添加数据
        :parmas msg: 是实例化的话题
        :ret 是相机状态, 为 True 则显示相机正常在线
        """
        msg.timestamp = time.time()
        msg.id = 0x01
        if ret:
            msg.state = 0
            msg.error = 0
        else:
            msg.state = 1
            msg.error = 1

    def pub_callback_camera_state(self):
        """
        Callback of publisher (timer), publish the topic 'camera_ori_data'.
        """
        # define msg
        msgCameraState = CameraStateInterface()

        # load msg
        self.update_camera_state(msgCameraState, self.ret)
    
        # publish
        self.pubCameraState.publish(msgCameraState)

    @staticmethod
    def update_camera_ori(msg, process_time, img):
        """
        给 msg 添加数据
        :params msg: 是实例化的话题
        :params process_time: 是处理时间
        :params img: 是发布的图像
        """

        msg.timestamp = time.time()
        msg.id = 0x01
        msg.imagedata = img
        msg.process_time = process_time

    def pub_callback_camera_ori(self):
        """
        Callback of publisher (timer), publish the topic 'camera_state_data'.
        """

        t0 = time.time()
        # define msg
        msgCameraOri = CameraOriInterface()
        if self.camera_state == 0 or self.camera_state == 1:
            self.ret, frame = self.cap.read()
            
        else:
            try:
                frame = self.hikcamera.get_image_BGR()
                # out.write(frame)
                self.ret = True if type(frame) is np.ndarray else False
            except:
                self.net = False
        
        if self.ret == True:
            # image cv2 to imgmsg
            if self.is_out_video:
                self.out_video.write(frame)
            bridge = CvBridge()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            frame = cv2.resize(frame, self.out_image_size)
            # if self.is_out_video:
            #     self.out_video.write(frame)

            img = bridge.cv2_to_imgmsg(frame, "bgr8")

            # 计算运行时间
            t1 = time.time()
            process_time = (t1-t0)
            self.update_camera_ori(msgCameraOri, process_time, img)

        else:
            print("--------------------Camera_down---------------------")
            try:
                self.load_video_camera(self.camera_state, self.video_path)  # 产生相机对象 self.cap
            except:
                pass
        
            # image cv2 to imgmsg
            bridge = CvBridge()

            # 相机掉线则发送全为0的图像
            img = np.zeros((self.out_image_size[1], self.out_image_size[0], 3), dtype=np.uint8)
            img = bridge.cv2_to_imgmsg(img, "bgr8")

            # 计算运行时间
            t1 = time.time()
            process_time = (t1-t0)
        
            self.update_camera_ori(msgCameraOri, process_time, img)

        # publish msg
        # out.release()
        self.pubCameraOri.publish(msgCameraOri)
        self.pubCameraOriRviz.publish(img)
        tji.ros_log(self.get_name(), 'Publish camera_ori msg')

    def load_video_camera(self, flag, video_path):
        """
        依据 flag 和 video_path 生成 self.cap
        """
        if flag == 0:  # 使用视频进行测试
            self.cap = cv2.VideoCapture(video_path)
        
        elif flag == 1:  # 读取 usb 相机
            self.cap = cv2.VideoCapture("/dev/video7")
            # 只有相机才能通过这种方法改变大小
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        else:  # 读取网口相机
            self.hikcamera = HIKCamera()
            self.hikcamera.check_env()
            self.hikcamera.init_camera()

def main():
    rclpy.init()
    rosNode = CameraOri(name=ros_node_name)
    rclpy.spin(rosNode)
    rclpy.shutdown()
