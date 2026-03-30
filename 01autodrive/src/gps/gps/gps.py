#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: yangkai
# @File: gps.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import rclpy
from rclpy.node import Node
import time
from car_interfaces.msg import *
import serial  # 导入串口包
import math
import copy
import numpy as np
import threading
from threading import Thread

ros_node_name = "gps"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

import tjitools


class Gps(Node):
    def __init__(self, name):
        super().__init__(name)

        # define publishers
        self.gpsData = None
        self.isGpsState = True
        self.pubGpsState = self.create_publisher(GpsStateInterface, 'gps_state_data', 10)
        self.timerGpsState = self.create_timer(0.01, self.pub_callback_gps_state)
        self.pubGps = self.create_publisher(GpsInterface, 'gps_data', 10)
        self.timerGps = self.create_timer(0.01, self.pub_callback_gps)

        # 定义一个线程，读取串口数据
        t_ = threading.Thread(target=self.thread_get_gps)
        t_.setDaemon(True)
        t_.start()

        self.time = 0
        self.gpsData_Last = ""

        pass

    def pub_callback_gps_state(self):
        """
        Callback of publisher (timer), publish the topic 'gps_state_data'.
        :param None.
        """

        msgGpsState = GpsStateInterface()
        msgGpsState.timestamp = time.time()  # 时间戳
        msgGpsState.id = 0x01  # 导航设备id

        if self.isGpsState == True:
            if self.gpsData is not None:
                ## 以下两个有问题
                msgGpsState.state = 0  # 设备状态
                msgGpsState.error = 0  # int(self.gpsData[23])    #错误码
            else:
                msgGpsState.state = 0
                msgGpsState.error = 0
        else:
            ## 以下两个有问题
            msgGpsState.state = 1  # 设备状态
            msgGpsState.error = 1  # 错误码

        self.pubGpsState.publish(msgGpsState)

    def pub_callback_gps(self):
        """
        Callback of publisher (timer), publish the topic 'gps_data'.
        :param None.
        """

        gpsData = copy.deepcopy(self.gpsData)

        msgGps = GpsInterface()
        now_ts = time.time()
        msgGps.timestamp = now_ts
        msgGps.id = 0x01  # id

        if self.isGpsState == True and gpsData is not None:
            print("----------------------------------------")
            msgGps.yaw = (90 - float(gpsData[3]))  # 偏航角
            # print("yaw1",msgGps.yaw)
            if (msgGps.yaw >= 360):
                msgGps.yaw = msgGps.yaw - 360
            if (msgGps.yaw < 0):
                msgGps.yaw = msgGps.yaw + 360
            msgGps.pitch = float(gpsData[4])  # 俯仰角
            msgGps.roll = float(gpsData[5])  # 横滚角
            msgGps.wx = float(gpsData[6])  # 角速度x
            msgGps.wy = float(gpsData[7])  # 角速度y
            msgGps.wz = float(gpsData[8])  # 角速度z

            msgGps.ax = float(gpsData[9]) * 9.8  # 加速度x
            msgGps.ay = float(gpsData[10]) * 9.8  # 加速度y
            msgGps.az = float(gpsData[11]) * 9.8  # 加速度z

            msgGps.latitude = float(gpsData[12])  # 纬度
            msgGps.longitude = float(gpsData[13])  # 经度
            print("lat = ", msgGps.latitude)
            print("lon = ", msgGps.longitude)

            msgGps.height = float(gpsData[14])  # 高度
            msgGps.eastvelocity = float(gpsData[15])  # 东向速度
            msgGps.northvelocity = float(gpsData[16])  # 北向速度
            msgGps.skyvelocity = float(gpsData[17])  # 天向速度

            ENU = tjitools.gps_to_enu(np.array([msgGps.longitude, msgGps.latitude, 0]))
            msgGps.x = float(ENU[0])
            print("x =", msgGps.x)
            msgGps.y = float(ENU[1])
            print("y =", msgGps.y)

            # msgGps.latitude=msgGps.y
            # msgGps.longitude=msgGps.x

            print("yaw", msgGps.yaw)
        else:
            msgGps = GpsInterface()
            msgGps.timestamp = time.time()
            msgGps.id = 0  # id
            msgGps.yaw = 0.0  # 偏航角
            msgGps.pitch = 0.0  # 俯仰角
            msgGps.pitch = 0.0  # 横滚角
            msgGps.wx = 0.0  # 角速度x
            msgGps.wy = 0.0  # 角速度y
            msgGps.wz = 0.0  # 角速度z
            msgGps.ax = 0.0  # 加速度x
            msgGps.ay = 0.0  # 加速度y
            msgGps.az = 0.0  # 加速度z
            msgGps.latitude = 0.0  # 纬度
            msgGps.longitude = 0.0  # 经度
            msgGps.height = 0.0  # 高度
            msgGps.eastvelocity = 0.0  # 东向速度
            msgGps.northvelocity = 0.0  # 北向速度
            msgGps.skyvelocity = 0.0  # 天向速度

        msgGps.process_time = time.time() - now_ts  # 进程处理时间
        self.pubGps.publish(msgGps)

    def thread_get_gps(self):
        print("start threading for gps")
        gps_serial = serial.Serial()
        gps_serial.port = "/dev/ttyUART_232_C"
        # gps_serial.port = "/dev/GNNS"
        # gps_serial.baudrate = 115200
        gps_serial.baudrate = 230400
        # gps_serial.baudrate = 9600
        gps_serial.timeout = 1
        gps_serial.rtscts = True
        gps_serial.dsrdtr = True
        
        #print("111111111111111111111")
        gps_serial.open()
        #print("222222222222222222222")
        while True:

            time.sleep(0.01)
            # print("time = ", time.time() - self.time)
            self.time = time.time()

            if gps_serial.isOpen():
                try:
                    n_ = gps_serial.in_waiting
                    # print("n_", n_)
                    if n_ > 0:
                        # 读出串口数据
                        now_recv_ = gps_serial.read_all().decode("utf-8")
                        #print("now_recv_ =", now_recv_)
                        gps_serial.flushInput()
                        recv_ = self.gpsData_Last + now_recv_
                        # print("recv_ =", recv_)
                        self.gpsData_Last = now_recv_

                        str_data = str(recv_).split(',')
                        #print("str_data=", str_data)
                        #print("333333333333333333333333333333")
                        #print(len(str_data))
                        for i in range(len(str_data)):
                            if (str_data[i][-6:] == '$GPCHC' or str_data[i] == '$GPCHC'):
                                if (len(str_data) - i) < 24:
                                    #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                                    break
                                if str_data[i + 23][4] != "*":
                                    #print("sta_data_24= ", str_data[i+23])
                                    continue
                                self.gpsData = str_data[i:i + 24]
                                #print("gosData = ", self.gpsData)
                                self.isGpsState = True
                                #print("find gps data########################################")
                except:
                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    pass


def main():
    rclpy.init()
    rosNode = Gps(name='gps')
    rclpy.spin(rosNode)
    rclpy.shutdown()
