#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: sonic_obstacle.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import time
import copy
import json
import numpy as np
import array
import struct
import serial
import threading
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "sonic_obstacle"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

sonic_number = 4
sonic_usb_name = (
    '/dev/sonic1',
    '/dev/sonic2',
    '/dev/sonic3',
    '/dev/sonic4',
)
sonic_angles = (
    0.0,
    90.0,
    180.0,
    270.0
)

min_dis = 0.25
max_dis = 5.0

class SonicObstacle(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubSonicState = self.create_publisher(SonicStateInterface, "sonic_state_data", 10)
        self.pubSonicObstacle = self.create_publisher(SonicObstacleInterface, "sonic_obstacle_data", 10)
        self.timerSonic = self.create_timer(0.1, self.pub_callback_sonic)

        # define subscribers
        self.msgSonicStates = [SonicStateInterface() for i in range(sonic_number)]
        self.msgSonicObstacles = [SonicObstacleInterface() for i in range(sonic_number)]
        for i in range(sonic_number):
            self.msgSonicStates[i].id = i+1
            self.msgSonicObstacles[i].id = i+1
            self.msgSonicObstacles[i].obstacledata = [sonic_angles[i], max_dis] * 4
        
        self.sonic_filters = max_dis * np.ones((sonic_number, 4, 5))
        
        # open threading to get sonic data
        Thread_list = []
        for i in range(sonic_number):
            t_ = threading.Thread(target=self.thread_get_sonic, args=(i,))
            t_.setDaemon(True)
            Thread_list.append(t_)
            # t_.start()
            # time.sleep(0.2)
        for i in range(sonic_number):
            Thread_list[i].start()
            time.sleep(0.05)
            

    def pub_callback_sonic(self):
        """
        Callback of publisher (timer), publish the topic sonic_state_data and sonic_obstacle_data.
        :param None.
        """
        now_ts = time.time()
        for i in range(sonic_number):
            self.msgSonicObstacles[i].timestamp = now_ts
            self.pubSonicObstacle.publish(self.msgSonicObstacles[i])
            self.msgSonicStates[i].timestamp = now_ts
            self.pubSonicState.publish(self.msgSonicStates[i])
            time.sleep(0.01)

            o_ = self.msgSonicObstacles[i].obstacledata                
            print("sonic %d: [%.3f, %.3f, %.3f, %.3f]"%(self.msgSonicObstacles[i].id, o_[1], o_[3], o_[5], o_[7]))

    def thread_get_sonic(self, num):
        print("start threading for sonic: %d"%(num+1))
        sonic_serial = serial.Serial()
        sonic_serial.port = sonic_usb_name[num]
        sonic_serial.baudrate = 9600
        sonic_serial.timeout = 1
        sonic_serial.rtscts = True
        sonic_serial.dsrdtr = True

        try:
            sonic_serial.open()
        except:
            print("failed to start sonic aaaaaa %d"%(num+1))
        
        cnt_err = 0
        while True:
            time.sleep(0.05)

            now_ts = time.time()
            cnt_err += 1
            if cnt_err > 20: # timeout, restart the serial

                while sonic_serial.isOpen():
                    sonic_serial.close()
                    time.sleep(0.1)
                
                try:
                    sonic_serial.open()
                    print("reopen serial !!!")
                except:
                    print("failed to start sonic %d"%(num+1))
                    print("failed to start" + sonic_usb_name[num])
                    

                cnt_err = 0

                self.msgSonicStates[num].state = 1
                self.msgSonicStates[num].error = 1

                self.msgSonicObstacles[num].number = 4
                self.msgSonicObstacles[num].process_time = time.time() - now_ts

            # check the data in buffer
            if sonic_serial.isOpen():

                try:
                    n_ = sonic_serial.in_waiting
                    if n_ >= 10:
                        # 读出串口数据
                        recv_ = sonic_serial.read_all()
                        sonic_serial.flushInput()
                        # print(recv_)

                        flag_ = (recv_[0] + recv_[1] + recv_[2] + recv_[3] + recv_[4] + recv_[5] + recv_[6] + recv_[7] + recv_[8]) & 0xFF
                        if recv_[0] == 0xFF and recv_[9] == flag_:
                            cnt_err = 0
                            
                            dis = [0, 0, 0, 0]
                            dis[0] = (recv_[1]*256 + recv_[2]) * 0.001
                            dis[1] = (recv_[3]*256 + recv_[4]) * 0.001
                            dis[2] = (recv_[5]*256 + recv_[6]) * 0.001
                            dis[3] = (recv_[7]*256 + recv_[8]) * 0.001

                            
                            self.msgSonicStates[num].state = 0
                            self.msgSonicStates[num].error = 0

                            self.msgSonicObstacles[num].number = 4
                            for i in range(4):
                                if dis[i] < 0.25:
                                    dis[i] = 5.0
                                if dis[i] >= 0.25:
                                    self.sonic_filters[num, i, :-1] = self.sonic_filters[num, i, 1:]
                                    self.sonic_filters[num, i, -1] = dis[i]
                                self.msgSonicObstacles[num].obstacledata[2*i+1] = round(np.median(self.sonic_filters[num, i]), 3)

                            ddd = self.sonic_filters[num, 2]
                            # print([recv_[i] for i in range(9)])
                            # print(num)
                            # print(self.msgSonicObstacles[num].obstacledata)
                            # print(dis)
                            # print(np.std(ddd), np.min(ddd), np.max(ddd), np.mean(ddd), np.median(ddd))

                            self.msgSonicObstacles[num].process_time = time.time() - now_ts
                
                except:
                    pass

def main():
    rclpy.init()
    rosNode = SonicObstacle()
    rclpy.spin(rosNode)
    rclpy.shutdown()
