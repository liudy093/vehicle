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
import signal
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "sonic_obstacle"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))


ros_path = '/opt/ros/foxy/lib/python3.8/site-packages'
msg_path = os.getcwd() + '/install/car_interfaces/lib/python3.8/site-packages'
obstacle_path = os.getcwd() + '/src/sonic_obstacle/sonic_obstacle'


import inspect
import ctypes
import threading
from time import sleep



import sys

sys.path.append(ros_path)
sys.path.append(msg_path)
sys.path.append(obstacle_path)


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

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)



class SonicObstacle(Node):
    def __init__(self, name):
        super().__init__(name)

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
        print("use 11111111111111 thread")
        # 全局变量初始化----------------------------------------------
        self.sonic_serial_1 = serial.Serial()
        self.sonic_serial_1.port = '/dev/sonic1'
        self.sonic_serial_1.baudrate = 9600
        self.sonic_serial_1.timeout = 0.02
        self.sonic_serial_1.rtscts = True
        self.sonic_serial_1.dsrdtr = True
        # self.sonic_serial_1.open()
        time.sleep(0.01)

        self.sonic_serial_2 = serial.Serial()
        self.sonic_serial_2.port = '/dev/sonic2'
        self.sonic_serial_2.baudrate = 9600
        self.sonic_serial_2.timeout = 0.2
        self.sonic_serial_2.rtscts = True
        self.sonic_serial_2.dsrdtr = True
        # self.sonic_serial_2.open()
        time.sleep(0.01)

        self.sonic_serial_3 = serial.Serial()
        self.sonic_serial_3.port = '/dev/sonic3'
        self.sonic_serial_3.baudrate = 9600
        self.sonic_serial_3.timeout = 0.02
        self.sonic_serial_3.rtscts = True
        self.sonic_serial_3.dsrdtr = True
       
        time.sleep(0.01)

        self.sonic_serial_4 = serial.Serial()
        self.sonic_serial_4.port = '/dev/sonic4'
        self.sonic_serial_4.baudrate = 9600
        self.sonic_serial_4.timeout = 0.02
        self.sonic_serial_4.rtscts = True
        self.sonic_serial_4.dsrdtr = True
        
        time.sleep(0.01)

        self.sonic_1_timeout_flag = 0
        self.sonic_2_timeout_flag = 0
        self.sonic_3_timeout_flag = 0
        self.sonic_4_timeout_flag = 0


        self.t_ = threading.Thread(target=self.get_all_sonic_data)
        self.t_.setDaemon(True)
        # self.t_.detach()
        # threading.detach()

        self.t_.start()


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
            # time.sleep(0.01)

            o_ = self.msgSonicObstacles[i].obstacledata                
            print("sonic %d: [%.3f, %.3f, %.3f, %.3f]"%(self.msgSonicObstacles[i].id, o_[1], o_[3], o_[5], o_[7]))
    
    def get_one_sonic_data(self, sonic_num, sonic_serial, timeout_flag):
        num = sonic_num - 1
        now_ts = time.time()
        timeout_flag = timeout_flag + 1
        if sonic_serial.isOpen():
            try:
                n_ = sonic_serial.in_waiting
                if n_ >= 10:
                    # 读出串口数据
                    recv_ = sonic_serial.read(n_)
                    sonic_serial.reset_input_buffer()
                    # print(recv_)

                    flag_ = (recv_[0] + recv_[1] + recv_[2] + recv_[3] + recv_[4] + recv_[5] + recv_[6] + recv_[7] + recv_[8]) & 0xFF
                    if recv_[0] == 0xFF and recv_[9] == flag_:
                        timeout_flag = 0 # 清除异常标志位        
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
                            # if dis[i] >= 0.25:
                            #     self.sonic_filters[num, i, :-1] = self.sonic_filters[num, i, 1:]
                            #     self.sonic_filters[num, i, -1] = dis[i]
                            # self.msgSonicObstacles[num].obstacledata[2*i+1] = round(np.median(self.sonic_filters[num, i]), 3)
                            self.msgSonicObstacles[num].obstacledata[2*i+1] = dis[i]

                        # ddd = self.sonic_filters[num, 2]
                        print("                                             get data, sonic_num:", sonic_num)
                        # print([recv_[i] for i in range(9)])
                        # print(num)
                        # print(self.msgSonicObstacles[num].obstacledata)
                        # print(dis)
                        # print(np.std(ddd), np.min(ddd), np.max(ddd), np.mean(ddd), np.median(ddd))
                        self.msgSonicObstacles[num].process_time = time.time() - now_ts
                        sonic_serial.flushInput()
            except:
                print("except")
                pass
        else:
            print("sonic_serial not Open()", sonic_num)
        return timeout_flag

    def sonic_data_timeout(self, sonic_num, sonic_serial, timeout_flag):
        now_ts = time.time()
        num = sonic_num - 1
        if timeout_flag > 50:
            # sonic_serial.flushInput()
            # while sonic_serial.isOpen():
            #             sonic_serial.close()
            #             time.sleep(0.01)
            if sonic_serial.isOpen():
                sonic_serial.close()
                time.sleep(0.01)
            try:
                sonic_serial.open()
                print("reopen serial " + sonic_num)
            except:
                print("failed to reopen sonic %d"%(sonic_num))
    
            timeout_flag = 0

            self.msgSonicStates[num].state = 1
            self.msgSonicStates[num].error = 1

            self.msgSonicObstacles[num].number = 4
            self.msgSonicObstacles[num].process_time = time.time() - now_ts
            
        return timeout_flag

    def get_all_sonic_data(self):
        print("get_all_sonic_data")
        # self.sonic_serial_1.open()
        # self.sonic_serial_2.open()

        try:
            self.sonic_serial_1.open()
        except:
            pass
        try:
            self.sonic_serial_2.open()
        except:
            pass
        try:
            self.sonic_serial_3.open()
        except:
            pass
        try:
            self.sonic_serial_4.open()
        except:
            pass
        # try:
        #     # self.sonic_serial_1.open()
        #     self.sonic_serial_2.open()

        #     # self.sonic_serial_3.open()
        #     # self.sonic_serial_4.open()
        #     print("!!!!!!! hhhhhh !!!!!!!")
        # except:
        #     pass
        # # 串口数据读取以及处理------------------------------------
      
        while rclpy.ok():
            time.sleep(0.01)
            # 串口数据读取以及处理------------------------------------
            self.sonic_1_timeout_flag = self.get_one_sonic_data(sonic_num=1, sonic_serial=self.sonic_serial_1, timeout_flag=self.sonic_1_timeout_flag)
            self.sonic_2_timeout_flag = self.get_one_sonic_data(sonic_num=2, sonic_serial=self.sonic_serial_2, timeout_flag=self.sonic_2_timeout_flag)
            self.sonic_3_timeout_flag = self.get_one_sonic_data(sonic_num=3, sonic_serial=self.sonic_serial_3, timeout_flag=self.sonic_3_timeout_flag)
            self.sonic_4_timeout_flag = self.get_one_sonic_data(sonic_num=4, sonic_serial=self.sonic_serial_4, timeout_flag=self.sonic_4_timeout_flag)

            # 串口异常状态判断---------------------------------------
            self.sonic_1_timeout_flag = self.sonic_data_timeout(sonic_num=1, sonic_serial=self.sonic_serial_1, timeout_flag=self.sonic_1_timeout_flag)
            self.sonic_2_timeout_flag = self.sonic_data_timeout(sonic_num=2, sonic_serial=self.sonic_serial_2, timeout_flag=self.sonic_2_timeout_flag)
            self.sonic_3_timeout_flag = self.sonic_data_timeout(sonic_num=3, sonic_serial=self.sonic_serial_3, timeout_flag=self.sonic_3_timeout_flag)
            self.sonic_4_timeout_flag = self.sonic_data_timeout(sonic_num=4, sonic_serial=self.sonic_serial_4, timeout_flag=self.sonic_4_timeout_flag)

            # print("sonic_1_timeout_flag", self.sonic_1_timeout_flag, "sonic_2_timeout_flag", self.sonic_2_timeout_flag)
            # print("sonic_3_timeout_flag", self.sonic_3_timeout_flag, "sonic_4_timeout_flag", self.sonic_4_timeout_flag)
            # print("sonic_1_timeout_flag", sonic_1_timeout_flag)

            # print(sonic_2_timeout_flag, "sonic_2_timeout_flag")
            # print(sonic_3_timeout_flag, "sonic_3_timeout_flag")
            # print(sonic_4_timeout_flag, "sonic_4_timeout_flag")
        self.sonic_serial_1.close()
        self.sonic_serial_2.close()
        self.sonic_serial_3.close()
        self.sonic_serial_4.close()
    


def main():
    rclpy.init()
    rosNode = SonicObstacle(name = "sonic_obstacle")
    rclpy.spin(rosNode)
    rclpy.shutdown()


if __name__ == '__main__':   
    rclpy.init()
    rosNode = SonicObstacle(name='sonic_obstacle')
    rclpy.spin(rosNode)
    rclpy.shutdown()