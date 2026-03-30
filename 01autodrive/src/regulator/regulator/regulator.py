#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: regulator.py
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

ros_node_name = "regulator"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class Regulator(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubRegulator = self.create_publisher(RegulatorInterface, "regulator_data", 10)
        self.timerRegulator = self.create_timer(0.1, self.pub_callback_regulator)

        # define subscribers
        self.subSonicObstacle = self.create_subscription(SonicObstacleInterface, "sonic_obstacle_data", self.sub_callback_sonic_obstacle, 1)
        
        self.obstacle_distance_front_left_2 = 0
        self.obstacle_distance_front_right_2 = 0
        self.obstacle_distance_rear_left_2 = 0
        self.obstacle_distance_rear_right_2 = 0

        self.obstacle_distance_min = 0.25
        self.obstacle_distance_max_front = 4.5
        self.obstacle_distance_max_rear = 2.5 

        pass

    def pub_callback_regulator(self):
        """
        Callback of publisher (timer), publish the topic regulator_data.
        :param None.
        """
        msgRegulator = RegulatorInterface()
        msgRegulator.timestamp = time.time()
        msgRegulator.stop = False

        if self.obstacle_distance_front_left_2 >= self.obstacle_distance_min and self.obstacle_distance_front_left_2 <= self.obstacle_distance_max_front:
            msgRegulator.stop = True
            print("self.obstacle_distance_front_left_2=",self.obstacle_distance_front_left_2)


        if self.obstacle_distance_front_right_2 >= self.obstacle_distance_min and self.obstacle_distance_front_right_2 <= self.obstacle_distance_max_front:
            msgRegulator.stop = True
            print("self.obstacle_distance_front_right_2=",self.obstacle_distance_front_right_2 )

        if self.obstacle_distance_rear_left_2 >= self.obstacle_distance_min and self.obstacle_distance_rear_left_2 <= self.obstacle_distance_max_rear:
            msgRegulator.stop = True
            print("self.obstacle_distance_rear_left_2=",self.obstacle_distance_rear_left_2)

        if self.obstacle_distance_rear_right_2 >= self.obstacle_distance_min and self.obstacle_distance_rear_right_2 <= self.obstacle_distance_max_rear:
            msgRegulator.stop = True
            print("self.obstacle_distance_rear_right_2=",self.obstacle_distance_rear_right_2)

        print(msgRegulator.stop)


        self.pubRegulator.publish(msgRegulator)
        pass

    def sub_callback_sonic_obstacle(self, msgSonicObstacle:SonicObstacleInterface):
        """
        Callback of subscriber, subscribe the topic sonic_obstacle_data.
        :param msgSonicObstacle: The message heard by the subscriber.
        """
        if msgSonicObstacle.id == 1:
            print("1111111")
            self.obstacle_distance_front_left_2 = msgSonicObstacle.obstacledata[3]
            print(msgSonicObstacle.obstacledata[3])
            self.obstacle_distance_front_right_2 = msgSonicObstacle.obstacledata[5]
            print(msgSonicObstacle.obstacledata[5])

        elif msgSonicObstacle.id == 2:
            print("2222222")
            self.obstacle_distance_rear_left_2 = msgSonicObstacle.obstacledata[3]
            print(msgSonicObstacle.obstacledata[3])
            self.obstacle_distance_rear_right_2 = msgSonicObstacle.obstacledata[5]
            print(msgSonicObstacle.obstacledata[5])

        else:
            pass





        pass

def main():
    rclpy.init()
    rosNode = Regulator()
    rclpy.spin(rosNode)
    rclpy.shutdown()
