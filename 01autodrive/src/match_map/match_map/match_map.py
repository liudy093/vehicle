#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: match_map.py
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

ros_node_name = "match_map"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class MatchMap(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubMatchMap = self.create_publisher(MatchMapInterface, "match_map_data", 10)
        self.timerMatchMap = self.create_timer(0.1, self.pub_callback_match_map)

        # define subscribers
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 1)
        
        pass

    def pub_callback_match_map(self):
        """
        Callback of publisher (timer), publish the topic match_map_data.
        :param None.
        """
        msgMatchMap = MatchMapInterface()
        msgMatchMap.timestamp = time.time()
        self.pubMatchMap.publish(msgMatchMap)
        pass

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        Callback of subscriber, subscribe the topic fusion_data.
        :param msgFusion: The message heard by the subscriber.
        """
        pass

def main():
    rclpy.init()
    rosNode = MatchMap()
    rclpy.spin(rosNode)
    rclpy.shutdown()
