
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: fusion.py
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
import cv2
import threading
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

ros_node_name = "fusion"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import rclpy
from rclpy.node import Node
from car_interfaces.msg import *
import tjitools
from .matching import associate_detections_to_trackers
import sklearn.cluster

class Fusion(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubFusion = self.create_publisher(FusionInterface, "fusion_data", 10)
        self.timerFusion = self.create_timer(0.1, self.pub_callback_fusion)

        # define subscribers
        self.subLightRecognition = self.create_subscription(LightRecognitionInterface, "light_recognition_data", self.sub_callback_light_recognition, 10)
        self.subSignRecognition = self.create_subscription(SignRecognitionInterface, "sign_recognition_data", self.sub_callback_sign_recognition, 10)
        self.subLaneRecognition = self.create_subscription(LaneRecognitionInterface, "lane_recognition_data", self.sub_callback_lane_recognition, 10)
        self.subCameraObstacle = self.create_subscription(CameraObstacleInterface, "camera_obstacle_data", self.sub_callback_camera_obstacle, 10)
        self.subLidarObstacle = self.create_subscription(LidarObstacleInterface, "lidar_obstacle_data", self.sub_callback_lidar_obstacle, 10)
        self.subRadarObstacle = self.create_subscription(RadarObstacleInterface, "radar_obstacle_data", self.sub_callback_radar_obstacle, 10)
        self.subGps = self.create_subscription(GpsInterface, "gps_data", self.sub_callback_gps, 10)
        self.subImu = self.create_subscription(ImuInterface, "imu_data", self.sub_callback_imu, 10)
        self.subCarOri = self.create_subscription(CarOriInterface, "car_ori_data", self.sub_callback_car_ori, 10)
        self.subMatchMap = self.create_subscription(MatchMapInterface, "match_map_data", self.sub_callback_match_map, 10)

        self.pubLidar_fusion_grid = self.create_publisher(GridCells, 'fusion_grid', 10)


        self.timerSensors = self.create_timer(3, self.sensors_state_monitor)
        self.rader_msg_recieve_flag = 0
        self.lider_msg_recieve_flag = 0
        self.camera_msg_recieve_flag = 0


        self.flagReceivedMsg = False
        self.rcvMsgLight = None
        self.rcvMsgSign = None
        self.rcvMsgLane = None
        self.rcvMsgCamera = None
        self.rcvMsgLidar = None
        self.rcvMsgRadar = None
        self.rcvMsgGps = None
        self.rcvMsgImu = None
        self.rcvMsgCar = None
        self.rcvMsgMap = None

        self.objCam = []
        self.objLid = []
        self.objRad = []
        self.objFus = []
        self.clustering_objs = []
        tjitools.ros_log(self.get_name(), 'Start Node:%s'%self.get_name())
    def sensors_state_monitor(self):

        # 数据清理-----------------------------------------------------------
        # self.objCam = []
        # self.objLid = []
        # self.objRad = []

        print("self.rader_msg_recieve_flag", self.rader_msg_recieve_flag)
        print("self.lider_msg_recieve_flag", self.lider_msg_recieve_flag)
        print("self.camera_msg_recieve_flag", self.camera_msg_recieve_flag)
        if self.rader_msg_recieve_flag==0:
                self.rader_msg_recieve_flag = 1
        else:
            self.objRad = []
        
        if self.lider_msg_recieve_flag==0:
                self.lider_msg_recieve_flag = 1
        else:
            self.objLid = []
        
        if self.camera_msg_recieve_flag==0:
                self.camera_msg_recieve_flag = 1
        else:
            self.objCam = []

        




    def pub_callback_fusion(self):
        """
        Callback of publisher (timer), publish the topic fusion_data.
        :param None.
        """
        now_ts = time.time()
        msgFusion = FusionInterface()
        msgFusion.timestamp = now_ts

        # data of car
        if self.rcvMsgCar is not None:
            msgFusion.id = self.rcvMsgCar.id
            msgFusion.carlength = 4.385
            msgFusion.carwidth = 1.850
            msgFusion.carheight = 1.650
            msgFusion.carspeed = self.rcvMsgCar.carspeed
            msgFusion.steerangle = self.rcvMsgCar.steerangle
            msgFusion.gearpos = self.rcvMsgCar.gearpos
            msgFusion.braketq = self.rcvMsgCar.braketq
            msgFusion.parkingstate = self.rcvMsgCar.parkingstate
            msgFusion.soc = self.rcvMsgCar.soc
            msgFusion.batteryvol = self.rcvMsgCar.batteryvol
            msgFusion.batterydischargecur = self.rcvMsgCar.batterydischargecur
            msgFusion.car_run_mode = self.rcvMsgCar.car_run_mode
            msgFusion.throttle_percentage = self.rcvMsgCar.throttle_percentage
            msgFusion.braking_percentage = self.rcvMsgCar.braking_percentage
            msgFusion.left_light = self.rcvMsgCar.left_light
            msgFusion.right_light = self.rcvMsgCar.right_light
            msgFusion.reversing_light = self.rcvMsgCar.reversing_light
            msgFusion.speaker = self.rcvMsgCar.speaker
            msgFusion.start_button = self.rcvMsgCar.start_button
            msgFusion.stop_button = self.rcvMsgCar.stop_button
            msgFusion.state = self.rcvMsgCar.state
            msgFusion.error = self.rcvMsgCar.error

        # data of gps & imu
        if self.rcvMsgGps is not None:
            msgFusion.yaw = self.rcvMsgGps.yaw
            msgFusion.pitch = self.rcvMsgGps.pitch
            msgFusion.roll = self.rcvMsgGps.roll
            msgFusion.wx = self.rcvMsgGps.wx
            msgFusion.wy = self.rcvMsgGps.wy
            msgFusion.wz = self.rcvMsgGps.wz
            msgFusion.ax = self.rcvMsgGps.ax
            msgFusion.ay = self.rcvMsgGps.ay
            msgFusion.az = self.rcvMsgGps.az
            msgFusion.longitude = self.rcvMsgGps.longitude
            msgFusion.latitude = self.rcvMsgGps.latitude
            msgFusion.height = self.rcvMsgGps.height
            msgFusion.eastvelocity = self.rcvMsgGps.eastvelocity
            msgFusion.northvelocity = self.rcvMsgGps.northvelocity
            msgFusion.skyvelocity = self.rcvMsgGps.skyvelocity

            ### test
            # msgFusion.longitude = 117.16684749
            # msgFusion.latitude = 39.10670276
            # msgFusion.yaw = 0.0

        # data of lane detection
        if self.rcvMsgLane is not None:
            if now_ts - self.rcvMsgLane.timestamp > 0.5:
                self.rcvMsgLane = None
            else:
                msgFusion.centeroffset = self.rcvMsgLane.centeroffset

        # data of sign detection
        if self.rcvMsgSign is not None:
            if now_ts - self.rcvMsgSign.timestamp > 0.5:
                self.rcvMsgSign = None
            else:
                msgFusion.signnumber = self.rcvMsgSign.signnumber
                msgFusion.signdata = copy.deepcopy(self.rcvMsgSign.signdata)

        # data of light detection
        if self.rcvMsgLight is not None:
            if now_ts - self.rcvMsgLight.timestamp > 0.5:
                self.rcvMsgLight = None
            else:
                msgFusion.lightdata = copy.deepcopy(self.rcvMsgLight.lightdata)

        # data of object detection
        self.objCam = np.array([])
        if self.rcvMsgCamera is not None and self.rcvMsgCamera.number > 0:
            if now_ts - self.rcvMsgCamera.timestamp > 0.5:
                self.rcvMsgCamera = None
            else:
                self.objCam = np.array(self.rcvMsgCamera.obstacledata).reshape(-1,9)
        
        self.objLid = np.array([])
        if self.rcvMsgLidar is not None and self.rcvMsgLidar.number > 0:
            if now_ts - self.rcvMsgLidar.timestamp > 0.5:
                self.rcvMsgLidar = None
            else:
                self.objLid = np.array(self.rcvMsgLidar.obstacledata).reshape(-1,9)
        
        self.objRad = np.array([])
        if self.rcvMsgRadar is not None and self.rcvMsgRadar.number > 0:
            if now_ts - self.rcvMsgRadar.timestamp > 0.5:
                self.rcvMsgRadar = None
            else:
                self.objRad = np.array(self.rcvMsgRadar.obstacledata).reshape(-1,9)
        
        self.objFus = self.obstacle_fusion(self.objCam, self.objLid, self.objRad)
    


        msgFusion.obstacledata = array.array('f', self.objFus.reshape(-1))

        self.pubFusion.publish(msgFusion)

        self.show_obstacle()        
        tjitools.ros_log(self.get_name(), 'Publish fusion msg !!!')

    def sub_callback_light_recognition(self, msgLightRecognition:LightRecognitionInterface):
        """
        Callback of subscriber, subscribe the topic light_recognition_data.
        :param msgLightRecognition: The message heard by the subscriber.
        """
        self.rcvMsgLight = msgLightRecognition

    def sub_callback_sign_recognition(self, msgSignRecognition:SignRecognitionInterface):
        """
        Callback of subscriber, subscribe the topic sign_recognition_data.
        :param msgSignRecognition: The message heard by the subscriber.
        """
        self.rcvMsgSign = msgSignRecognition

    def sub_callback_lane_recognition(self, msgLaneRecognition:LaneRecognitionInterface):
        """
        Callback of subscriber, subscribe the topic lane_recognition_data.
        :param msgLaneRecognition: The message heard by the subscriber.
        """
        self.rcvMsgLane = msgLaneRecognition

    def sub_callback_camera_obstacle(self, msgCameraObstacle:CameraObstacleInterface):
        """
        Callback of subscriber, subscribe the topic camera_obstacle_data.
        :param msgCameraObstacle: The message heard by the subscriber.
        """
        self.rcvMsgCamera = msgCameraObstacle
        self.camera_msg_recieve_flag = 0

    def sub_callback_lidar_obstacle(self, msgLidarObstacle:LidarObstacleInterface):
        """
        Callback of subscriber, subscribe the topic lidar_obstacle_data.
        :param msgLidarObstacle: The message heard by the subscriber.
        """
        self.rcvMsgLidar = msgLidarObstacle
        self.lider_msg_recieve_flag = 0

    def sub_callback_radar_obstacle(self, msgRadarObstacle:RadarObstacleInterface):
        """
        Callback of subscriber, subscribe the topic radar_obstacle_data.
        :param msgRadarObstacle: The message heard by the subscriber.
        """
        self.rcvMsgRadar = msgRadarObstacle
        self.rader_msg_recieve_flag = 0

    def sub_callback_gps(self, msgGps:GpsInterface):
        """
        Callback of subscriber, subscribe the topic gps_data.
        :param msgGps: The message heard by the subscriber.
        """
        self.rcvMsgGps = msgGps

    def sub_callback_imu(self, msgImu:ImuInterface):
        """
        Callback of subscriber, subscribe the topic imu_data.
        :param msgImu: The message heard by the subscriber.
        """
        self.rcvMsgImu = msgImu

    def sub_callback_car_ori(self, msgCarOri:CarOriInterface):
        """
        Callback of subscriber, subscribe the topic car_ori_data.
        :param msgCarOri: The message heard by the subscriber.
        """
        self.rcvMsgCar = msgCarOri

    def sub_callback_match_map(self, msgMatchMap:MatchMapInterface):
        """
        Callback of subscriber, subscribe the topic match_map_data.
        :param msgMatchMap: The message heard by the subscriber.
        """
        self.rcvMsgMap = msgMatchMap

    def obstacle_fusion_li(self, objCam:np.ndarray, objLid:np.ndarray, objRad:np.ndarray):
        objFus = [o_ for o_ in objCam]
        for rad_ in objRad:
            is_new = True
            for fus_ in objFus:
                if (fus_[4]-rad_[4])**2 + (fus_[5]-rad_[5])**2 < 0.5:
                    is_new = False
                    fus_[4] = 0.5*(fus_[4]+rad_[4])
                    fus_[5] = 0.5*(fus_[5]+rad_[5])
                    fus_[6] = 0.5*(fus_[6]+rad_[6])
                    fus_[7] = rad_[7]
                    fus_[8] = rad_[8]
            if is_new:
                objFus.append(rad_)
        
        for lid_ in objLid:
            is_new = True
            for fus_ in objFus:
                if (fus_[4]-lid_[4])**2 + (fus_[5]-lid_[5])**2 < 0.5:
                    is_new = False
                    fus_[4] = 0.5*(fus_[4]+lid_[4])
                    fus_[5] = 0.5*(fus_[5]+lid_[5])
                    fus_[6] = 0.5*(fus_[6]+lid_[6])
            if is_new:
                objFus.append(lid_)
                
        return np.array(objFus)
    
    def obstacle_fusion(self, objCam: np.ndarray, objLid: np.ndarray, objRad: np.ndarray):
        """
            三个传感器检测结果融合 \n
            Args:
                objCam: [[category,l,w,h,x,y,z,v,latv],[...],[...]]
                objLid: [[category,l,w,h,x,y,z,v,latv],[...],[...]]
                objRad: [[category,l,w,h,x,y,z,v,latv],[...],[...]]

            Returns:

            """
        objFus = [o_ for o_ in objLid]  # 以雷达的xyz为基准，在fusion中添加目标并修改其他数据
        # ======================================================== #
        # 1.先融合radar, xyz以objFus为准, v,latv以毫米波雷达为准
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(objFus, objRad)

        for match in matched:  # 对于匹配的结果，更新x,y,z,v和latv
            objFus[match[0]][4:9] = objRad[match[1], 4:9]

#        for unmatch in unmatched_trks:  # 对于不匹配的结果，加在objFus之后
#            objFus.append(objRad[unmatch])

        for obj in objFus:  # 对于所有的目标，把类别置为0，lwh置为1
            obj[0] = 0
            obj[1:4] = [1, 1, 1]

        # ======================================================== #
        # 2.再融合camera, xyz以objFus为准, category以camera为准
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(objFus, objCam)

        for match in matched:  # 对于匹配的结果，更新category
            objFus[match[0]][0] = objCam[match[1], 0]

        for unmatch in unmatched_trks:  # 对于不匹配的结果，加在objFus之后
            objFus.append(objCam[unmatch])
        
        # 数据清理-----------------------------------------------------------
        


        if len(objFus) > 0:
            if self.rcvMsgCar is None:
                car_speed = 0
            else:
                car_speed = float(self.rcvMsgCar.carspeed)
            objFus = self.clustering(objFus, car_speed)
            print('xxxxxxxxxxxxx')
        else:
            self.clustering_objs = []
            print('yyyyyyyyyyy')

        return np.array(objFus)
    

    def clustering(self, objFus, car_speed):
        objFus = np.array(objFus)
        print("objFus", objFus.shape)
        obj_xyz = objFus[:, 4:6]
        print("---------------------------------------------------------")
        print("obj_xyz", obj_xyz.shape)
        Css = sklearn.cluster.DBSCAN(eps=0.80, min_samples=1).fit(obj_xyz)
        clusters_index_np = np.array(Css.labels_)
        clusters_index = np.expand_dims(clusters_index_np, axis=1)
        print("clusters_index", clusters_index.shape)
        clustering_obj = np.concatenate([objFus, clusters_index], axis=1)
        max_class = np.max(clusters_index)
        print("max_class", max_class)

        np.set_printoptions(suppress=True, 
        precision=10,
        threshold=2000,
        linewidth=150) 

        
        # 选取每个类别中不为0的速度值，将该类别的速度都换成该速度值
        for i in range(max_class+1):
            # 查找该类别所以目标
            objs = objFus[np.where(clusters_index_np == i)]
            # 查找该类别速度不为0目标
            # 横向速度-------------------------------------------------------
            # objs_v_sum = objs[np.where(objs[:, 7] != 0)][:, 7].flatten()
            objs_v_sum = objs[np.where(objs[:, 7] != 0)]
            objs_v_sum = objs_v_sum[:, 7].flatten()
            # 有目标存在速度值
            if objs_v_sum.shape[0] > 0:
                objs_v = np.mean(objs_v_sum) - car_speed
            else:
                objs_v = -car_speed

            # if objs_v_sum.shape[0] > 0:
                # print("##########################")
                # print("objs_v", objs_v)
                # print("objs", objs)
                
            obj_indexs = np.where(clusters_index_np == i)
            # print("obj_indexs", obj_indexs)
            for j, index in enumerate(obj_indexs[0]):
                # print("index", index)
                objFus[index][7] = objs_v
                clustering_obj[index][7] = objs_v

            # 纵向速度-------------------------------------------------------
            objs_v_sum = objs[np.where(objs[:, 8] != 0)]
            objs_v_sum = objs_v_sum[:, 8].flatten()
            # 有目标存在速度值
            if objs_v_sum.shape[0] > 0:
                objs_v = np.mean(objs_v_sum) 
            else:
                objs_v = 0

            # if objs_v_sum.shape[0] > 0:
                # print("##########################")
                # print("objs_v", objs_v)
                # print("objs", objs)
                
            obj_indexs = np.where(clusters_index_np == i)
            # print("obj_indexs", obj_indexs)
            for j, index in enumerate(obj_indexs[0]):
                # print("index", index)
                objFus[index][8] = objs_v
                clustering_obj[index][8] = objs_v

            # 类别分配-------------------------------------------------------
            objs_class =  objs[np.where(objs[:, 0] != 0)]
            
            # 有非0类别
            if objs_class.shape[0] > 0:
                print(objs_class)
                obj_indexs = np.where(clusters_index_np == i)
                print(objs_class)
                for j, index in enumerate(obj_indexs[0]):
                    # print("index", index)
                    objFus[index][0] = objs_class[0, 0]
                    # print("index", index)
                    clustering_obj[index][0] = 1






        print("objFus", objFus)
        print("clustering_obj", clustering_obj)
        # objFus = objFus.tolist()
        self.clustering_objs = clustering_obj
        return objFus




    def show_obstacle_li(self):

        img_h = 400
        img_w = 400
        img_f = 0.25

        img1 = 255*np.ones(shape=(img_h, img_w, 3), dtype=np.uint8)
        for o_ in self.objLid:
            x_ = round(o_[4]/img_f) + img_w//2
            y_ = -round(o_[5]/img_f) + img_h//2
            img1[y_-5:y_+5, x_-5:x_+5,:] = (0,255,0)
        
        for o_ in self.objCam:
            x_ = round(o_[4]/img_f) + img_w//2
            y_ = -round(o_[5]/img_f) + img_h//2
            img1[y_-5:y_+5, x_-5:x_+5,:] = (255,0,0)
        
        for o_ in self.objRad:
            x_ = round(o_[4]/img_f) + img_w//2
            y_ = -round(o_[5]/img_f) + img_h//2
            img1[y_-5:y_+5, x_-5:x_+5,:] = (0,0,255)

        cv2.imshow('img1', cv2.cvtColor(img1, cv2.COLOR_RGB2BGR))
        cv2.waitKey(5)
        
        img2 = 255*np.ones(shape=(img_h, img_w, 3), dtype=np.uint8)
        for o_ in self.objFus:
            x_ = round(o_[4]/img_f) + img_w//2
            y_ = -round(o_[5]/img_f) + img_h//2
            img2[x_-5:x_+5, y_-5:y_+5,:] = (0,0,255)

        cv2.imshow('img2', cv2.cvtColor(img2, cv2.COLOR_RGB2BGR))
        cv2.waitKey(5)
    
    def show_obstacle(self):

        print("self.clustering_objs", self.clustering_objs)
        if len(self.clustering_objs) > 0:
            cells = GridCells()
            cells.header.frame_id = "rslidar"
            cells.cell_width = 0.5  # edit for grid size .3 for simple map
            cells.cell_height = 0.5  # edit for grid size
        
            for o_ in self.clustering_objs:
                point = Point()
                point.x = float(o_[5])
                point.y = -float(o_[4])
                point.z = float(o_[9] * 0.2)
                cells.cells.append(point)
                # x_ = round(o_[4]/img_f) + img_w//2
                # y_ = -round(o_[5]/img_f) + img_h//2
            self.pubLidar_fusion_grid.publish(cells)
            
        else:
            cells = GridCells()
            cells.header.frame_id = "rslidar"
            cells.cell_width = 0.5  # edit for grid size .3 for simple map
            cells.cell_height = 0.5  # edit for grid size
            point = Point()
            point.x = float(0)
            point.y = -float(0)
            point.z = float(0)
            cells.cells.append(point)
            print("sssssssssssssss")
            self.pubLidar_fusion_grid.publish(cells)

        


def main():
    rclpy.init()
    rosNode = Fusion()
    rclpy.spin(rosNode)
    rclpy.shutdown()
