#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: imu.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import time
import copy
import json
import numpy as np
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "imu"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class Imu(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubImuState = self.create_publisher(ImuStateInterface, "imu_state_data", 10)
        self.timerImuState = self.create_timer(0.1, self.pub_callback_imu_state)
        self.pubImu = self.create_publisher(ImuInterface, "imu_data", 10)
        self.timerImu = self.create_timer(0.1, self.pub_callback_imu)

        # define subscribers
        
        pass

    def pub_callback_imu_state(self):
        """
        Callback of publisher (timer), publish the topic imu_state_data.
        :param None.
        """
        msgImuState = ImuStateInterface()
        msgImuState.timestamp = time.time()
        self.pubImuState.publish(msgImuState)
        pass

    def pub_callback_imu(self):
        """
        Callback of publisher (timer), publish the topic imu_data.
        :param None.
        """
        msgImu = ImuInterface()
        msgImu.timestamp = time.time()
        self.pubImu.publish(msgImu)
        pass

def main():
    rclpy.init()
    rosNode = Imu()
    rclpy.spin(rosNode)
    rclpy.shutdown()
