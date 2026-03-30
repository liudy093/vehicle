#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: car_decision.py
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

ros_node_name = "car_decision"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import tjitools

class CarDecision(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubCarDecision = self.create_publisher(CarDecisionInterface, "car_decision_data", 10)
        self.timerCarDecision = self.create_timer(0.1, self.pub_callback_car_decision)

        # define subscribers
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 1)
        self.subGlobalPathPlanning = self.create_subscription(GlobalPathPlanningInterface, "global_path_planning_data", self.sub_callback_global_path_planning, 1)
        self.subCameraState = self.create_subscription(CameraStateInterface, "camera_state_data", self.sub_callback_camera_state, 1)
        self.subLidarState = self.create_subscription(LidarStateInterface, "lidar_state_data", self.sub_callback_lidar_state, 1)
        self.subRadarState = self.create_subscription(RadarStateInterface, "radar_state_data", self.sub_callback_radar_state, 1)
        self.subSonicState = self.create_subscription(SonicStateInterface, "sonic_state_data", self.sub_callback_sonic_state, 1)
        self.subGpsState = self.create_subscription(GpsStateInterface, "gps_state_data", self.sub_callback_gps_state, 1)
        self.subImuState = self.create_subscription(ImuStateInterface, "imu_state_data", self.sub_callback_imu_state, 1)
        self.subCanState = self.create_subscription(CanStateInterface, "can_state_data", self.sub_callback_can_state, 1)
        
        self.rcvMsgFusion = None
        self.rcvMsgGlbPath = None
        self.rcvMsgCamState = None
        self.rcvMsgLidState = None
        self.rcvMsgRadState = None
        self.rcvMsgSonState = None
        self.rcvMsgGpsState = None
        self.rcvMsgImuState = None
        self.rcvMsgCanState = None

        tjitools.ros_log(self.get_name(), 'Start Node:%s'%self.get_name())

    def pub_callback_car_decision(self):
        """
        Callback of publisher (timer), publish the topic car_decision_data.
        :param None.
        """
        msgCarDecision = CarDecisionInterface()
        now_ts = time.time()
        msgCarDecision.timestamp = now_ts

        # check if there is sensor error
        if self.rcvMsgCamState is not None and self.rcvMsgCamState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgLidState is not None and self.rcvMsgLidState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgRadState is not None and self.rcvMsgRadState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgSonState is not None and self.rcvMsgSonState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgGpsState is not None and self.rcvMsgGpsState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgImuState is not None and self.rcvMsgImuState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        if self.rcvMsgCanState is not None and self.rcvMsgCanState.state != 0:
            msgCarDecision.stop = True
            msgCarDecision.process_time = time.time() - now_ts
            self.pubCarDecision.publish(msgCarDecision)
            return
        
        if self.rcvMsgFusion is not None:
            # check if there is obstacle
            if len(self.rcvMsgFusion.obstacledata)>0:
                v_ = self.rcvMsgFusion.carspeed
                objs_ = np.array(self.rcvMsgFusion.obstacledata).reshape(-1,9)
                for o_ in objs_:
                    if o_[5]>0 and o_[5]<v_*v_/2/1.0 and abs(o_[4])<2.0: # near obstacle
                        msgCarDecision.stop = True
                        msgCarDecision.process_time = time.time() - now_ts
                        self.pubCarDecision.publish(msgCarDecision)
                        return
            # check if there is speed limitation
            if len(self.rcvMsgFusion.signdata)>0:
                sgns_ = np.array(self.rcvMsgFusion.signdata).reshape(-1,9)
                for s_ in sgns_:
                    if s_[0] == 1:
                        msgCarDecision.speed = True
                        msgCarDecision.maxspeed = s_[7]
                        msgCarDecision.process_time = time.time() - now_ts
                        self.pubCarDecision.publish(msgCarDecision)
                        return

        msgCarDecision.process_time = time.time() - now_ts
        self.pubCarDecision.publish(msgCarDecision)

        tjitools.ros_log(self.get_name(), 'Publish car_decision msg !!!')

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        Callback of subscriber, subscribe the topic fusion_data.
        :param msgFusion: The message heard by the subscriber.
        """
        self.rcvMsgFusion = msgFusion
        pass

    def sub_callback_global_path_planning(self, msgGlobalPathPlanning:GlobalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic global_path_planning_data.
        :param msgGlobalPathPlanning: The message heard by the subscriber.
        """
        self.rcvMsgGlbPath = msgGlobalPathPlanning
        pass

    def sub_callback_camera_state(self, msgCameraState:CameraStateInterface):
        """
        Callback of subscriber, subscribe the topic camera_state_data.
        :param msgCameraState: The message heard by the subscriber.
        """
        self.rcvMsgCamState = msgCameraState
        pass

    def sub_callback_lidar_state(self, msgLidarState:LidarStateInterface):
        """
        Callback of subscriber, subscribe the topic lidar_state_data.
        :param msgLidarState: The message heard by the subscriber.
        """
        self.rcvMsgLidState = msgLidarState
        pass

    def sub_callback_radar_state(self, msgRadarState:RadarStateInterface):
        """
        Callback of subscriber, subscribe the topic radar_state_data.
        :param msgRadarState: The message heard by the subscriber.
        """
        self.rcvMsgRadState = msgRadarState
        pass

    def sub_callback_sonic_state(self, msgSonicState:SonicStateInterface):
        """
        Callback of subscriber, subscribe the topic sonic_state_data.
        :param msgSonicState: The message heard by the subscriber.
        """
        self.rcvMsgSonState = msgSonicState
        pass

    def sub_callback_gps_state(self, msgGpsState:GpsStateInterface):
        """
        Callback of subscriber, subscribe the topic gps_state_data.
        :param msgGpsState: The message heard by the subscriber.
        """
        self.rcvMsgGpsState = msgGpsState
        pass

    def sub_callback_imu_state(self, msgImuState:ImuStateInterface):
        """
        Callback of subscriber, subscribe the topic imu_state_data.
        :param msgImuState: The message heard by the subscriber.
        """
        self.rcvMsgImuState = msgImuState
        pass

    def sub_callback_can_state(self, msgCanState:CanStateInterface):
        """
        Callback of subscriber, subscribe the topic can_state_data.
        :param msgCanState: The message heard by the subscriber.
        """
        self.rcvMsgCanState = msgCanState
        pass

def main():
    rclpy.init()
    rosNode = CarDecision()
    rclpy.spin(rosNode)
    rclpy.shutdown()
