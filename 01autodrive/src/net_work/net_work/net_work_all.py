#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: net_work.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

from abc import ABC
import sys
import os
import time
import copy
import json
import threading
import numpy as np
import struct
import pymysql
import socket
import psutil
from jtop import jtop
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "net_work"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import tjitools as tji

CAR_ID=0x01
ALL_ID=[0x01,0x02,0x03]
DATA_TYPE=['SEND','RECEIVE']

# user infos for ali cloud
DBHOST = 'rm-2zec5959a1jnx38j01o.mysql.rds.aliyuncs.com'
DBUSER = 'fuyun'
DBPASS = 'Kwy26538817'
DBNAME = 'fuyun_test'

class Data(ABC):
    """
    Dtype == 'SEND' : send data
    Dtype == 'RECEIVE' : receive data
    send_data: send data
    receive_data: receive data
    before_send_data: check the data type
    before_receive_data: check the data type
    """
    def __init__(self, data_type='SEND'):
        self.type=data_type
    def send_data(self):
        pass
    def receive_data(self,data_list):
        pass
    def before_send_data(self):
        if self.type=='SEND':
            pass
        else:
            raise ValueError('The data type is not SEND.')
    def before_receive_data(self):
        if self.type=='RECEIVE':
            pass
        else:
            raise ValueError('The data type is not RECEIVE.')
        
class DataCarSelf(Data):
    def __init__(self, data_type='SEND'):
        super().__init__(data_type)
        self.data_ecd = '<BBBQBBfBBBBffHBB'    
        
        self.start1=0x23
        self.start2=0x23
        self.car_id=CAR_ID 
        self.time=0
        self.stateid= 0xCF 
        self.car_mode= 0x03 
        self.car_speed=0
        self.car_angle=0x1E
        self.soc=0
        self.err=0x00
        self.errcode=0x00
        self.lon=0
        self.lat=0
        self.yaw=90
        self.end1=0x24
        self.end2=0x24


        self.data_list_name = ["self.start1",
                                "self.start2",
                                "self.car_id",
                                "self.time",
                                "self.stateid",
                                "self.car_mode",
                                "self.car_speed",
                                "self.car_angle",
                                "self.soc",
                                "self.err",
                                "self.errcode",
                                "self.lon",
                                "self.lat",
                                "self.yaw",
                                "self.end1",
                                "self.end2"]
    def receive_data(self,data_list):
        super().before_receive_data()
        [self.start1,
        self.start2,
        self.car_id,
        self.time,
        self.stateid,
        self.car_mode,
        self.car_speed,
        self.car_angle,
        self.soc,
        self.err,
        self.errcode,
        self.lon,
        self.lat,
        self.yaw,
        self.end1,
        self.end2]=data_list
    
    def send_data(self):
        super().before_send_data()
        data_list = [self.start1,
                    self.start2,
                    self.car_id,
                    self.time,
                    self.stateid,
                    self.car_mode,
                    self.car_speed,
                    self.car_angle,
                    self.soc,
                    self.err,
                    self.errcode,
                    self.lon,
                    self.lat,
                    self.yaw,
                    self.end1,
                    self.end2]

        return data_list
class DataHardwareStatus(Data):
    def __init__(self, data_type='SEND'):
        super().__init__(data_type)
        self.data_ecd = '<BBBQBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'    

        self.start1=0x23
        self.start2=0x23
        self.car_id=CAR_ID
        self.time=0
        self.stateid= 0xDF

        self.radar_start= 0xAA
        self.radar_id= 0x00
        self.radar_state= 0x00
        self.radar_errcode= 0x00
        self.radar_end = 0xAA

        self.lidar_start= 0xAB
        self.lidar_id= 0x00
        self.lidar_state= 0x00
        self.lidar_errcode= 0x00
        self.lidar_end = 0xAC


        self.sonic_start= 0xAD
        self.sonic1_id= 0x01
        self.sonic1_state= 0x00
        self.sonic1_errcode= 0x00

        self.sonic2_id= 0x02
        self.sonic2_state= 0x00
        self.sonic2_errcode= 0x00

        self.sonic3_id= 0x03
        self.sonic3_state= 0x00
        self.sonic3_errcode= 0x00

        self.sonic4_id= 0x04
        self.sonic4_state= 0x00
        self.sonic4_errcode= 0x00
        self.sonic_end = 0xAE

        self.camera_start= 0xBA
        self.camera1_id= 0x00
        self.camera1_state= 0x00
        self.camera1_errcode= 0x00

        self.camera2_id= 0x01
        self.camera2_state= 0x00
        self.camera2_errcode= 0x00

        self.camera3_id= 0x02
        self.camera3_state= 0x00
        self.camera3_errcode= 0x00

        self.camera4_id= 0x03
        self.camera4_state= 0x00
        self.camera4_errcode= 0x00

        self.camera5_id= 0x04
        self.camera5_state= 0x00
        self.camera5_errcode= 0x00
        self.camera_end = 0xBB
        
        # 毫米波CAN卡状态
        self.millimeter_wave_can_start= 0xBC
        self.millimeter_wave_can_id= 0x00
        self.millimeter_wave_can_state= 0x00
        self.millimeter_wave_can_errcode= 0x00
        self.millimeter_wave_can_end= 0xBD

        # 底盘
        self.chassis_can_start= 0xBE
        self.chassis_can_id= 0x01
        self.chassis_can_state= 0x00
        self.chassis_can_errcode= 0x00
        self.chassis_can_end = 0xCA

        #GPS
        self.GPS_start= 0xCB
        self.GPS_id= 0x00
        self.GPS_state= 0x00
        self.GPS_errcode= 0x00
        self.GPS_end = 0xCC

        #麦克风
        self.microphone_start= 0xCD
        self.microphone_id= 0x00
        self.microphone_state= 0x00
        self.microphone_errcode= 0x00
        self.microphone_end= 0xCE

        #红绿灯
        self.light_start= 0xDA
        self.light_id=[]
        self.light_state=[]
        self.light_errcode=[]
        self.light_end=0xDB

        self.end1=0x24
        self.end2=0x24


    def receive_data(self,data_list):
        super().before_receive_data()
        #TODO: TO BE CONTINUED

    def send_data(self):
        super().before_send_data()
        ecd_ = [
            '<BBBQB',
            'B'*5,  # radar
            'B'*5,  # lidar
            'B'*14, # sonic
            'B'*17, # camera
            'B'*5,  # radar-can
            'B'*5,  # vehicle-can
            'B'*5,  # GPS
            'B'*5,  # microphone
            'B'*(2+3*len(self.light_id)), # traffic light
            'BB'
        ]
        self.data_ecd = ''.join(ecd_)
        # self.data_ecd = '<BBBQB' + 'B'*65 + 'BBB'*len(self.light_id)
        data_list1 = [self.start1,
                    self.start2,
                    self.car_id,
                    self.time,
                    self.stateid,

                    self.radar_start,
                    self.radar_id,
                    self.radar_state,
                    self.radar_errcode,
                    self.radar_end,

                    self.lidar_start,
                    self.lidar_id,
                    self.lidar_state,
                    self.lidar_errcode,
                    self.lidar_end,

                    self.sonic_start,
                    self.sonic1_id,
                    self.sonic1_state,
                    self.sonic1_errcode,

                    self.sonic2_id,
                    self.sonic2_state,
                    self.sonic2_errcode,

                    self.sonic3_id,
                    self.sonic3_state,
                    self.sonic3_errcode,

                    self.sonic4_id,
                    self.sonic4_state,
                    self.sonic4_errcode,
                    self.sonic_end,

                    self.camera_start,
                    self.camera1_id,
                    self.camera1_state,
                    self.camera1_errcode,

                    self.camera2_id,
                    self.camera2_state,
                    self.camera2_errcode,

                    self.camera3_id,
                    self.camera3_state,
                    self.camera3_errcode,

                    self.camera4_id,
                    self.camera4_state,
                    self.camera4_errcode,

                    self.camera5_id,
                    self.camera5_state,
                    self.camera5_errcode,
                    self.camera_end,
        
                    # 毫米波CAN卡状态
                    self.millimeter_wave_can_start,
                    self.millimeter_wave_can_id,
                    self.millimeter_wave_can_state,
                    self.millimeter_wave_can_errcode,
                    self.millimeter_wave_can_end,

                    # 底盘
                    self.chassis_can_start,
                    self.chassis_can_id,
                    self.chassis_can_state,
                    self.chassis_can_errcode,
                    self.chassis_can_end,

                    #GPS
                    self.GPS_start,
                    self.GPS_id,
                    self.GPS_state,
                    self.GPS_errcode,
                    self.GPS_end,

                    #麦克风
                    self.microphone_start,
                    self.microphone_id,
                    self.microphone_state,
                    self.microphone_errcode,
                    self.microphone_end]

        #红绿灯        
        data_list2=[]
        data_list2.append(self.light_start)
        for i in range(len(self.light_id)):
            data_list2.append(self.light_id[i])
            data_list2.append(self.light_state[i])
            data_list2.append(self.light_errcode[i])
        data_list2.append(self.light_end)
        data_list3=[self.end1,
                    self.end2]
        data_list = data_list1 + data_list2 + data_list3
                
        # return struct.pack(self.data_ecd, *data_list)
        return data_list
    
class DataObstacles(Data):
    def __init__(self, data_type='SEND'):
        super().__init__(data_type)
        self.data_ecd = '<BBBQBBB'    
        
        self.start1=0x23
        self.start2=0x23
        self.car_id=CAR_ID
        self.time=0
        self.stateid= 0xEF 
        self.obstacle_class=[]
        self.obstacle_lon=[] 
        self.obstacle_lat=[]
        self.end1=0x24
        self.end2=0x24
    
    def receive_data(self,data_list):
        super().before_receive_data()
        #TODO: TO BE CONTINUED

    def send_data(self):
        super().before_send_data()
        ecd_ = [
            '<BBBQB',
            'Bff'*len(self.obstacle_class), # traffic light
            'BB'
        ]
        self.data_ecd = ''.join(ecd_)
        # self.data_ecd = '<BBBQB'+'Bff'*len(self.obstacle_class)+'BB'    
        data_list1 = [self.start1,
                    self.start2,
                    self.car_id,
                    self.time,
                    self.stateid]
        data_list2 = []
        for i in range(len(self.obstacle_class)):
            data_list2.append(self.obstacle_class[i])
            data_list2.append(self.obstacle_lon[i])
            data_list2.append(self.obstacle_lat[i])
        data_list3=[self.end1,
                    self.end2]
        data_list = data_list1+data_list2+data_list3

        # print(data_list)
        # print(self.data_ecd)
        # data_list_name1 = ["self.start1",
        #                    "self.start2",
        #                    "self.car_id",
        #                    "self.time",
        #                    "self.stateid"]
        # data_list_name2 = []
        # for i in range(len(self.obstacle_class)):
        #     data_list_name2.append("self.obstacle_class{0}".format(i+1))
        #     data_list_name2.append("self.obstacle_lon{0}".format(i+1))
        #     data_list_name2.append("self.obstacle_lat{0}".format(i+1))
        # data_list_name3 = ["self.end1",
        #                    "self.end2"]
        # self.data_list_name = data_list_name1 + data_list_name2 + data_list_name3

        # print("------------------------------------")
        # print(len(self.data_ecd), len(data_list))
        # for i in range(len(data_list)):
        #     print(self.data_list_name[i], data_list[i], self.data_ecd[i+1])

        # return struct.pack(self.data_ecd, *data_list)
        return data_list
class DataSoftwareStatus(Data):
    def __init__(self, data_type='SEND'):
        super().__init__(data_type)
        self.data_ecd = '<BBBQB'+'H'*25+'BB'    
    
        self.start1=0x23
        self.start2=0x23
        self.car_id=CAR_ID
        self.time=0
        self.stateid= 0xFF 
        self.CPU_used_capacity=0
        self.CPU_total_capacity=0
        self.GPU_used_capacity=0
        self.GPU_total_capacity=0
        self.memory_used_capacity=0
        self.memory_total_capacity=0    
        self.hardDisk_used_capacity=0
        self.hardDisk_total_capacity=0    
        self.radar_process=0
        self.lidar_process=0
        self.sonic_process=0
        self.camera_process=0
        self.microphone_process=0
        self.GPS_process=0
        self.finiteStateMachine_process=0
        self.spatialFusion_process=0
        self.followTrace_process=0
        self.chassisControl_process=0
        self.network_process=0
        self.radar_frequency=0
        self.lidar_frequency=0
        self.sonic_frequency=0
        self.camera_frequency=0
        self.microphone_frequency=0
        self.GPS_frequency=0
        self.end1=0x24
        self.end2=0x24

        self.data_list_name = ["self.start1",
                                "self.start2",
                                "self.car_id",
                                "self.time",
                                "self.stateid",
                                "self.CPU_used_capacity",
                                "self.CPU_total_capacity",
                                "self.GPU_used_capacity",
                                "self.GPU_total_capacity",
                                "self.memory_used_capacity",
                                "self.memory_total_capacity",
                                "self.hardDisk_used_capacity",
                                "self.hardDisk_total_capacity",
                                "self.radar_process",
                                "self.lidar_process",
                                "self.sonic_process",
                                "self.camera_process",
                                "self.microphone_process",
                                "self.GPS_process",
                                "self.finiteStateMachine_process",
                                "self.spatialFusion_process",
                                "self.followTrace_process",
                                "self.chassisControl_process",
                                "self.network_process",
                                "self.radar_frequency",
                                "self.lidar_frequency",
                                "self.sonic_frequency",
                                "self.camera_frequency",
                                "self.microphone_frequency",
                                "self.GPS_frequency",
                                "self.end1",
                                "self.end2"]
    
    def send_data(self):
        super().before_send_data()
        data_list = [self.start1,
                    self.start2,
                    self.car_id, 
                    self.time,
                    self.stateid, 
                    self.CPU_used_capacity,
                    self.CPU_total_capacity,
                    self.GPU_used_capacity,
                    self.GPU_total_capacity,
                    self.memory_used_capacity,
                    self.memory_total_capacity,    
                    self.hardDisk_used_capacity,
                    self.hardDisk_total_capacity,    
                    self.radar_process,
                    self.lidar_process,
                    self.sonic_process,
                    self.camera_process,
                    self.microphone_process,
                    self.GPS_process,
                    self.finiteStateMachine_process,
                    self.spatialFusion_process,
                    self.followTrace_process,
                    self.chassisControl_process,
                    self.network_process,
                    self.radar_frequency,
                    self.lidar_frequency,
                    self.sonic_frequency,
                    self.camera_frequency,
                    self.microphone_frequency,
                    self.GPS_frequency,
                    self.end1,
                    self.end2]
        return data_list
    
    def get_data(self,data_list):
        super().before_receive_data()
        [self.start1,
        self.start2,
        self.car_id, 
        self.time,
        self.stateid, 
        self.CPU_used_capacity,
        self.CPU_total_capacity,
        self.GPU_used_capacity,
        self.GPU_total_capacity,
        self.memory_used_capacity,
        self.memory_total_capacity,    
        self.hardDisk_used_capacity,
        self.hardDisk_total_capacity,    
        self.radar_process,
        self.lidar_process,
        self.sonic_process,
        self.camera_process,
        self.microphone_process,
        self.GPS_process,
        self.finiteStateMachine_process,
        self.spatialFusion_process,
        self.followTrace_process,
        self.chassisControl_process,
        self.network_process,
        self.radar_frequency,
        self.lidar_frequency,
        self.sonic_frequency,
        self.camera_frequency,
        self.microphone_frequency,
        self.GPS_frequency,
        self.end1,
        self.end2]=data_list




class NetWork(Node):

    def __init__(self):
        super().__init__(ros_node_name)
        
        # define publishers
        self.pubNetReceiveData1 = self.create_publisher(NetReceiveData1Interface, 'cloud_data_1', 10)
        self.pubNetReceiveData2 = self.create_publisher(NetReceiveData2Interface, 'cloud_data_2', 10)

        self.timerNetStartEndPoint = self.create_timer(0.2, self.pub_callback_net_start_end_point)
        # 模拟测试代码
        # self.test = self.create_timer(0.1, self.test_sub)
        
        # define subscribers
        self.subCameraState = self.create_subscription(CameraStateInterface, "camera_state_data", self.sub_callback_camera_state, 1)
        self.subCarmeraOri = self.create_subscription(CameraOriInterface, "carmera_ori_data", self.sub_callback_carmera_ori, 1)
        self.subLidarState = self.create_subscription(LidarStateInterface, "lidar_state_data", self.sub_callback_lidar_state, 1)
        self.subLidarOri = self.create_subscription(LidarOriInterface, "lidar_ori_data", self.sub_callback_lidar_ori, 1)
        self.subRadarState = self.create_subscription(RadarStateInterface, "radar_state_data", self.sub_callback_radar_state, 1)
        self.subRadarObstacle = self.create_subscription(RadarObstacleInterface, "radar_obstacle_data", self.sub_callback_radar_obstacle, 1)
        self.subSonicState = self.create_subscription(SonicStateInterface, "sonic_state_data", self.sub_callback_sonic_state, 1)
        self.subSonicObstacle = self.create_subscription(SonicObstacleInterface, "sonic_obstacle_data", self.sub_callback_sonic_obstacle, 1)
        self.subCanState = self.create_subscription(CanStateInterface, "can_state_data", self.sub_callback_can_state, 1)
        self.subCarOri = self.create_subscription(CarOriInterface, "car_ori_data", self.sub_callback_car_ori, 1)
        self.subGpsState = self.create_subscription(GpsStateInterface, "gps_state_data", self.sub_callback_gps_state, 1)
        self.subGps = self.create_subscription(GpsInterface, "gps_data", self.sub_callback_gps, 1)
        self.subImuState = self.create_subscription(ImuStateInterface, "imu_state_data", self.sub_callback_imu_state, 1)
        self.subImu = self.create_subscription(ImuInterface, "imu_data", self.sub_callback_imu, 1)
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 1)
        self.subCarDecision = self.create_subscription(CarDecisionInterface, "car_decision_data", self.sub_callback_car_decision, 1)
        self.subPid = self.create_subscription(PidInterface, "pid_data", self.sub_callback_pid, 1)
        



        self.db = pymysql.connect(host=DBHOST, user=DBUSER, password=DBPASS, database=DBNAME)
        self.cur = self.db.cursor()
        # ————————————————————————————————
        # self.tcp_client.connect((self.server_ip, self.server_port))

        # init data used for send
        self.data_carself = DataCarSelf(data_type="SEND")
        self.data_hardware = DataHardwareStatus(data_type="SEND")
        # self.data_obstacles = DataObstacles(data_type="SEND")
        self.data_software = DataSoftwareStatus(data_type="SEND")

        #init data used for receiving
        self.other_vehicle_data1=dict()
        self.other_vehicle_data2=dict()
        self.other_vehicle_data1['data_carself']=DataCarSelf(data_type="RECEIVE")
        self.other_vehicle_data2['data_carself']=DataCarSelf(data_type="RECEIVE")
        self.other_vehicle_data1['data_hardware']=DataHardwareStatus(data_type="RECEIVE")
        self.other_vehicle_data2['data_hardware']=DataHardwareStatus(data_type="RECEIVE")
        # self.other_vehicle_data1['data_obstacles']=DataObstacles(data_type="RECEIVE")
        # self.other_vehicle_data2['data_obstacles']=DataObstacles(data_type="RECEIVE")
        self.other_vehicle_data1['data_software']=DataSoftwareStatus(data_type="RECEIVE")
        self.other_vehicle_data2['data_software']=DataSoftwareStatus(data_type="RECEIVE")

        ALL_ID.remove(CAR_ID)
        self.other_vehicle_id1=ALL_ID[0]
        self.other_vehicle_id2=ALL_ID[1]

        
        ##用来计算频率
        self.radar_lastTimestamp=0
        self.lidar_lastTimestamp=0
        self.sonic_lastTimestamp=[0,0,0,0]
        self.carmera_lastTimestamp=0
        self.microphone_lastTimestamp=0
        self.GPS_lastTimestamp=0

        # 创建硬件线程监听Orin硬件状态
        t_ = threading.Thread(target=self.thread_orin_hardware, args=())
        t_.setDaemon(True)
        t_.start()

    def pub_callback_net_start_end_point(self):
        """
        Callback of publisher (timer), publish the topic 'net_start_end_point_data'.
        :param None.
        """
        ts=time.time()
        ######################################################################
        # 时间戳
        self.data_carself.time = np.int64(ts*1000)
        # 发送车辆自身状态数据
        # self.tcp_client.send(self.data_carself.send_data())
        
        
        value_data_carself=self.data_carself.send_data()

        fields = ', '.join(['field%d' % (i) for i in range(len(value_data_carself))])
        placeholders = ', '.join(['%s'] * len(value_data_carself))
        if CAR_ID!=1:
            sql = "INSERT INTO data_carself%d (%s) VALUES (%s)" % (CAR_ID, fields, placeholders)
        else:
            sql = "INSERT INTO data_carself (%s) VALUES (%s)" % (fields, placeholders)

        self.cur.execute(sql, value_data_carself)
        self.db.commit()
        
        
        
        time.sleep(0.05)

        # 时间戳
        self.data_hardware.time = np.int64(ts*1000)
        # 发送硬件状态数据

        value_data_hardware=self.data_hardware.send_data()

        fields = ', '.join(['field%d' % (i) for i in range(len(value_data_hardware))])
        placeholders = ', '.join(['%s'] * len(value_data_hardware))
        if CAR_ID!=1:
            sql = "INSERT INTO data_hardware%d (%s) VALUES (%s)" % (CAR_ID, fields, placeholders)
        else:
            sql = "INSERT INTO data_hardware (%s) VALUES (%s)" % (fields, placeholders)

        self.cur.execute(sql, value_data_hardware)
        self.db.commit()


        # self.tcp_client.send(self.data_hardware.send_data())
        time.sleep(0.05)

        print('Vehilce id: %d'%(self.data_hardware.car_id))
        print('CAN: id: %d, state: %d, error: %d'%(
            self.data_hardware.chassis_can_id, self.data_hardware.chassis_can_state, self.data_hardware.chassis_can_errcode))
        print('Sonic 1: id: %d, state: %d, error: %d'%(
            self.data_hardware.sonic1_id, self.data_hardware.sonic1_state, self.data_hardware.sonic1_errcode))
        print('Camera: id: %d, state: %d, error: %d'%(
            self.data_hardware.camera1_id, self.data_hardware.camera1_state, self.data_hardware.camera1_errcode))
        print('Lidar: id: %d, state: %d, error: %d'%(
            self.data_hardware.lidar_id, self.data_hardware.lidar_state, self.data_hardware.lidar_errcode))

        # # 时间戳
        # self.data_obstacles.time = np.int64(ts*1000)
        # 发送障碍物数据


        # value_data_obstacles=self.data_obstacles.send_data()

        # fields = ', '.join(['field%d' % (i) for i in range(len(value_data_obstacles))])
        # placeholders = ', '.join(['%s'] * len(value_data_obstacles))
        # if CAR_ID!=1:
        #     sql = "INSERT INTO data_obstacles%d (%s) VALUES (%s)" % (CAR_ID, fields, placeholders)
        # else:
        #     sql = "INSERT INTO data_obstacles (%s) VALUES (%s)" % (fields, placeholders)

        # self.cur.execute(sql, value_data_obstacles)
        # self.db.commit()


        # self.tcp_client.send(self.data_obstacles.send_data())
        time.sleep(0.05)

        # # 时间戳
        self.data_software.time = np.int64(ts*1000)
        # 计算network的进程时间
        self.data_software.network_process=np.int16(1000*(time.time()-ts))
        # 发送软件状态数据
        # self.tcp_client.send(self.data_software.send_data())
        value_data_software=self.data_software.send_data()

        fields = ', '.join(['field%d' % (i) for i in range(len(value_data_software))])
        placeholders = ', '.join(['%s'] * len(value_data_software))
        if CAR_ID!=1:
            sql = "INSERT INTO data_software%d (%s) VALUES (%s)" % (CAR_ID, fields, placeholders)
        else:
            sql = "INSERT INTO data_software (%s) VALUES (%s)" % (fields, placeholders)

        self.cur.execute(sql, value_data_software)
        self.db.commit()

        #"SELECT * FROM data_carself"
        msgNetReceiveData1Interface = NetReceiveData1Interface()
        msgNetReceiveData2Interface = NetReceiveData2Interface()

        sqlmsg1="SELECT * FROM data_carself"+str(self.other_vehicle_id1)+" ORDER BY field3 DESC LIMIT 1"
        sqlmsg2="SELECT * FROM data_carself"+str(self.other_vehicle_id2)+" ORDER BY field3 DESC LIMIT 1"
        if self.other_vehicle_id1 == 1:
            sqlmsg1="SELECT * FROM data_carself ORDER BY field3 DESC LIMIT 1"
        if self.other_vehicle_id2 == 1:
            sqlmsg2="SELECT * FROM data_carself ORDER BY field3 DESC LIMIT 1"
        other_vehicle_data1=self.get_data_sql(sql_msg=sqlmsg1)
        other_vehicle_data2=self.get_data_sql(sql_msg=sqlmsg2)
        
        # print(other_vehicle_data1)
        # for d in other_vehicle_data1:
        #     print(type(d))      
        self.other_vehicle_data1['data_carself'].receive_data(other_vehicle_data1)
        self.other_vehicle_data2['data_carself'].receive_data(other_vehicle_data2)

        msgNetReceiveData1Interface.start1 = self.other_vehicle_data1['data_carself'].start1
        msgNetReceiveData1Interface.start2 = self.other_vehicle_data1['data_carself'].start2
        msgNetReceiveData1Interface.car_id = self.other_vehicle_data1['data_carself'].car_id
        msgNetReceiveData1Interface.time = float(self.other_vehicle_data1['data_carself'].time)
        msgNetReceiveData1Interface.stateid = self.other_vehicle_data1['data_carself'].stateid
        msgNetReceiveData1Interface.car_mode = self.other_vehicle_data1['data_carself'].car_mode
        msgNetReceiveData1Interface.car_speed = float(self.other_vehicle_data1['data_carself'].car_speed)
        msgNetReceiveData1Interface.car_angle = float(self.other_vehicle_data1['data_carself'].car_angle)
        msgNetReceiveData1Interface.soc = self.other_vehicle_data1['data_carself'].soc
        msgNetReceiveData1Interface.err = self.other_vehicle_data1['data_carself'].err
        msgNetReceiveData1Interface.errcode = self.other_vehicle_data1['data_carself'].errcode
        msgNetReceiveData1Interface.lon = self.other_vehicle_data1['data_carself'].lon
        msgNetReceiveData1Interface.lat = self.other_vehicle_data1['data_carself'].lat
        msgNetReceiveData1Interface.yaw = float(self.other_vehicle_data1['data_carself'].yaw)
        msgNetReceiveData1Interface.end1 = self.other_vehicle_data1['data_carself'].end1
        msgNetReceiveData1Interface.end2 = self.other_vehicle_data1['data_carself'].end2

        msgNetReceiveData2Interface.start1 = self.other_vehicle_data2['data_carself'].start1
        msgNetReceiveData2Interface.start2 = self.other_vehicle_data2['data_carself'].start2
        msgNetReceiveData2Interface.car_id = self.other_vehicle_data2['data_carself'].car_id
        msgNetReceiveData2Interface.time = float(self.other_vehicle_data2['data_carself'].time)
        msgNetReceiveData2Interface.stateid = self.other_vehicle_data2['data_carself'].stateid
        msgNetReceiveData2Interface.car_mode = self.other_vehicle_data2['data_carself'].car_mode
        msgNetReceiveData2Interface.car_speed = float(self.other_vehicle_data2['data_carself'].car_speed)
        msgNetReceiveData2Interface.car_angle = float(self.other_vehicle_data2['data_carself'].car_angle)
        msgNetReceiveData2Interface.soc = self.other_vehicle_data2['data_carself'].soc
        msgNetReceiveData2Interface.err = self.other_vehicle_data2['data_carself'].err
        msgNetReceiveData2Interface.errcode = self.other_vehicle_data2['data_carself'].errcode
        msgNetReceiveData2Interface.lon = self.other_vehicle_data2['data_carself'].lon
        msgNetReceiveData2Interface.lat = self.other_vehicle_data2['data_carself'].lat
        msgNetReceiveData2Interface.yaw = float(self.other_vehicle_data2['data_carself'].yaw)
        msgNetReceiveData2Interface.end1 = self.other_vehicle_data2['data_carself'].end1
        msgNetReceiveData2Interface.end2 = self.other_vehicle_data2['data_carself'].end2


        self.pubNetReceiveData1.publish(msgNetReceiveData1Interface)
        self.pubNetReceiveData2.publish(msgNetReceiveData2Interface)

    def sub_callback_camera_state(self, msgCameraState:CameraStateInterface):
        """
        Callback of subscriber, subscribe the topic camera_state_data.
        :param msgCameraState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        if msgCameraState.id==1:
            self.data_hardware.camera1_id=np.int8(msgCameraState.id)
            self.data_hardware.camera1_state=np.int8(msgCameraState.state)
            self.data_hardware.camera1_errcode=np.int8(msgCameraState.error)
        elif msgCameraState.id==2:
            self.data_hardware.camera2_id=np.int8(msgCameraState.id)
            self.data_hardware.camera2_state=np.int8(msgCameraState.state)
            self.data_hardware.camera2_errcode=np.int8(msgCameraState.error)
        elif msgCameraState.id==3:
            self.data_hardware.camera3_id=np.int8(msgCameraState.id)
            self.data_hardware.camera3_state=np.int8(msgCameraState.state)
            self.data_hardware.camera3_errcode=np.int8(msgCameraState.error)
        elif msgCameraState.id==4:
            self.data_hardware.camera4_id=np.int8(msgCameraState.id)
            self.data_hardware.camera4_state=np.int8(msgCameraState.state)
            self.data_hardware.camera4_errcode=np.int8(msgCameraState.error)
        elif msgCameraState.id==5:
            self.data_hardware.camera5_id=np.int8(msgCameraState.id)
            self.data_hardware.camera5_state=np.int8(msgCameraState.state)
            self.data_hardware.camera5_errcode=np.int8(msgCameraState.error)
        else:
            pass

    def sub_callback_carmera_ori(self, msgCameraOri:CameraOriInterface):
        """
        Callback of subscriber, subscribe the topic carmera_ori_data.
        :param msgCarmeraOri: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.camera_process = np.int16(1000*msgCameraOri.process_time)
        self.data_software.camera_frequency=np.int16(1/(msgCameraOri.timestamp-self.carmera_lastTimestamp))
        self.carmera_lastTimestamp=msgCameraOri.timestamp
        pass

    def sub_callback_lidar_state(self, msgLidarState:LidarStateInterface):
        """
        Callback of subscriber, subscribe the topic lidar_state_data.
        :param msgLidarState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        self.data_hardware.lidar_id=np.int8(msgLidarState.id)
        self.data_hardware.lidar_state=np.int8(msgLidarState.state)
        self.data_hardware.lidar_errcode=np.int8(msgLidarState.error)
        pass

    def sub_callback_lidar_ori(self, msgLidarOri:LidarOriInterface):
        """
        Callback of subscriber, subscribe the topic lidar_ori_data.
        :param msgLidarOri: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.lidar_process = np.int16(1000*msgLidarOri.process_time)
        self.data_software.lidar_frequency=np.int16(1/(msgLidarOri.timestamp-self.lidar_lastTimestamp))
        self.lidar_lastTimestamp=msgLidarOri.timestamp
        pass

    def sub_callback_radar_state(self, msgRadarState:RadarStateInterface):
        """
        Callback of subscriber, subscribe the topic radar_state_data.
        :param msgRadarState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        self.data_hardware.radar_id=np.int8(msgRadarState.id)
        self.data_hardware.radar_state=np.int8(msgRadarState.state)
        self.data_hardware.radar_errcode=np.int8(msgRadarState.error)
        pass

    def sub_callback_radar_obstacle(self, msgRadarObstacle:RadarObstacleInterface):
        """
        Callback of subscriber, subscribe the topic radar_obstacle_data.
        :param msgRadarObstacle: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.radar_process = np.int16(1000*msgRadarObstacle.process_time)
        self.data_software.radar_frequency=np.int16(1/(msgRadarObstacle.timestamp-self.radar_lastTimestamp))
        self.radar_lastTimestamp=msgRadarObstacle.timestamp
        pass

    def sub_callback_sonic_state(self, msgSonicState:SonicStateInterface):
        """
        Callback of subscriber, subscribe the topic sonic_state_data.
        :param msgSonicState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        if msgSonicState.id==1:
            self.data_hardware.sonic1_id=np.int8(msgSonicState.id)
            self.data_hardware.sonic1_state=np.int8(msgSonicState.state)
            self.data_hardware.sonic1_errcode=np.int8(msgSonicState.error)
        elif msgSonicState.id==2:
            self.data_hardware.sonic2_id=np.int8(msgSonicState.id)
            self.data_hardware.sonic2_state=np.int8(msgSonicState.state)
            self.data_hardware.sonic2_errcode=np.int8(msgSonicState.error)
        elif msgSonicState.id==3:
            self.data_hardware.sonic3_id=np.int8(msgSonicState.id)
            self.data_hardware.sonic3_state=np.int8(msgSonicState.state)
            self.data_hardware.sonic3_errcode=np.int8(msgSonicState.error)
        elif msgSonicState.id==4:
            self.data_hardware.sonic4_id=np.int8(msgSonicState.id)
            self.data_hardware.sonic4_state=np.int8(msgSonicState.state)
            self.data_hardware.sonic4_errcode=np.int8(msgSonicState.error)
        else:
            pass

    def sub_callback_sonic_obstacle(self, msgSonicObstacle:SonicObstacleInterface):
        """
        Callback of subscriber, subscribe the topic sonic_obstacle_data.
        :param msgSonicObstacle: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.sonic_process = np.int16(1000*msgSonicObstacle.process_time)
        num = msgSonicObstacle.id - 1
        self.data_software.sonic_frequency=np.int16(1/(msgSonicObstacle.timestamp-self.sonic_lastTimestamp[num]))
        self.sonic_lastTimestamp[num] = msgSonicObstacle.timestamp
        pass

    def sub_callback_can_state(self, msgCanState:CanStateInterface):
        """
        Callback of subscriber, subscribe the topic can_state_data.
        :param msgCanState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        self.data_hardware.chassis_can_id=np.uint8(msgCanState.id)
        self.data_hardware.chassis_can_state=np.uint8(msgCanState.state)
        self.data_hardware.chassis_can_errcode=np.uint8(msgCanState.error)
        pass

    def sub_callback_car_ori(self, msgCarOri:CarOriInterface):
        """
        Callback of subscriber, subscribe the topic car_ori_data.
        :param msgCarOri: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.chassisControl_process = np.int16(1000*msgCarOri.process_time)

    def sub_callback_gps_state(self, msgGpsState:GpsStateInterface):
        """
        Callback of subscriber, subscribe the topic gps_state_data.
        :param msgGpsState: The message heard by the subscriber.
        """
        #################################硬件状态数据#######################################
        self.data_hardware.GPS_id=np.uint8(msgGpsState.id)
        self.data_hardware.GPS_state=np.uint8(msgGpsState.state)
        self.data_hardware.GPS_errcode=np.uint8(msgGpsState.error)
        pass

    def sub_callback_gps(self, msgGps:GpsInterface):
        """
        Callback of subscriber, subscribe the topic gps_data.
        :param msgGps: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.GPS_process = np.int16(1000*msgGps.process_time)
        self.data_software.GPS_frequency=np.int16(1/(msgGps.timestamp-self.GPS_lastTimestamp))
        self.GPS_lastTimestamp=msgGps.timestamp
        pass

    def sub_callback_imu_state(self, msgImuState:ImuStateInterface):
        """
        Callback of subscriber, subscribe the topic imu_state_data.
        :param msgImuState: The message heard by the subscriber.
        """
        pass

    def sub_callback_imu(self, msgImu:ImuInterface):
        """
        Callback of subscriber, subscribe the topic imu_data.
        :param msgImu: The message heard by the subscriber.
        """
        pass

    def sub_callback_car_decision(self, msgCarDecision:CarDecisionInterface):
        """
        Callback of subscriber, subscribe the topic car_decision_data.
        :param msgCarDecision: The message heard by the subscriber.
        """
        #################################软件状态数据#######################################
        self.data_software.finiteStateMachine_process = np.int16(1000*msgCarDecision.process_time)
        pass

    def sub_callback_pid(self, msgPid:PidInterface): 
        #################################软件状态数据#######################################
        self.data_software.followTrace_process = np.int16(1000*msgPid.process_time)

    def test_sub(self):
        ####################################sonic_state_sub#############################################
        self.data_hardware.sonic1_id=0x02
        if np.random.random() > 0.8:
            self.data_hardware.sonic1_state=0x01
            self.data_hardware.sonic1_errcode=0x01
        else:
            self.data_hardware.sonic1_state = 0x00
            self.data_hardware.sonic1_errcode = 0x00

        self.data_hardware.GPS_id=0X01
        if np.random.random() > 0.8:
            self.data_hardware.GPS_state=0X01
            self.data_hardware.GPS_errcode=0X03
        else:
            self.data_hardware.GPS_state=0X00
            self.data_hardware.GPS_errcode=0X00

        self.data_hardware.camera1_id = 0x01
        p_ = np.random.random()
        if p_ > 0.8:
            self.data_hardware.camera1_state = 0x01
            self.data_hardware.camera1_errcode = 0x02
        elif p_ > 0.6:
            self.data_hardware.camera1_state = 0x02
            self.data_hardware.camera1_errcode = 0x00
        else:
            self.data_hardware.camera1_state = 0x00
            self.data_hardware.camera1_errcode = 0x00

        self.data_hardware.lidar_id = 0x01
        p_ = np.random.random()
        if p_ > 0.8:
            self.data_hardware.lidar_state = 0x01
            self.data_hardware.lidar_errcode = 0x01
        elif p_ > 0.6:
            self.data_hardware.lidar_state = 0x02
            self.data_hardware.lidar_errcode = 0x00
        else:
            self.data_hardware.lidar_state = 0x00
            self.data_hardware.lidar_errcode = 0x00
            


        ####################################sonic_obstact_sub#############################################
        # self.data_software.sonic_process = np.int16(1000*0.05)
        # self.data_software.sonic_frequency=np.int16(1/(time.time()-self.sonic_lastTimestamp))
        # self.sonic_lastTimestamp=time.time()
        ####################################fusion_sub#############################################
        # 车辆ID
        self.data_carself.car_id = CAR_ID
        # 车辆速度
        self.data_carself.car_speed = round(12 * np.random.random())
        # 车辆转角
        self.data_carself.car_angle = round(60 * np.random.random())
        # 电池电量
        self.data_carself.soc = round(100 * np.random.random())
        # 故障大类
        self.data_carself.err = 0x00
        # 具体故障码
        if self.data_carself.err == 0x00:
            self.data_carself.errcode = 0x00
        else:
            self.data_carself.errcode = 0x01
        # 经度
        self.data_carself.lon = 0 
        # 纬度
        self.data_carself.lat = 0
        # 航向角
        self.data_carself.yaw = 90


        # 车辆ID
        self.data_hardware.car_id = CAR_ID
        # 红绿灯
        lightdata=[0,2,3,1,2,3,1,2,1,2,3,1]
        # lightdata=[]
        self.data_hardware.light_id=[]
        self.data_hardware.light_state=[]
        self.data_hardware.light_errcode=[]
        for i in range(int(len(lightdata)/6)):
            self.data_hardware.light_id.append(lightdata[i+1])
            if lightdata[6*i+3]==0x04 and lightdata[6*i+4]==0x04 and lightdata[6*i+5]==0x04:
                self.data_hardware.light_state.append(0x02)
            #这里需要写一个故障判断
            else:
                self.data_hardware.light_state.append(0x00)
            self.data_hardware.light_errcode.append(0x00)

        # 车辆ID
        # self.data_obstacles.car_id = CAR_ID
        # 车辆障碍物
        obstacledata=[2,3,7,1,7,5,1,20,0,1,1,1,1,15,15,1,5,0]
        # obstacledata=[]
        # self.data_obstacles.obstacle_class=[]
        # self.data_obstacles.obstacle_lat=[]
        # self.data_obstacles.obstacle_lon=[]
        # for i in range(int(len(obstacledata)/9)):
        #     self.data_obstacles.obstacle_class.append(obstacledata[i*9])
        #     x=obstacledata[i*9+4]
        #     y=obstacledata[i*9+5]
        #     car_lon=self.data_carself.lon
        #     car_lat=self.data_carself.lat
        #     yaw=self.data_carself.yaw
        #     lon_lat_hei=tji.rfu_to_gps(np.array([x,y,0]),np.array([car_lon,car_lat,0]),np.array([0,0,yaw]))
        #     lon_lat_hei = np.float32(lon_lat_hei)
        #     self.data_obstacles.obstacle_lon.append(lon_lat_hei[0])
        #     self.data_obstacles.obstacle_lat.append(lon_lat_hei[1])

        pass

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        ###############################车辆本体数据########################################
        # 车辆ID
        self.data_carself.car_id = CAR_ID#np.uint8(msgFusion.id)
        # print(self.data_carself.car_id)
        # 车辆速度
        self.data_carself.car_speed = msgFusion.carspeed
        # 车辆转角
        self.data_carself.car_angle = round(msgFusion.steerangle/540 * 30) + 30
        self.data_carself.car_angle = max(self.data_carself.car_angle, 0)
        self.data_carself.car_angle = min(self.data_carself.car_angle, 60)
        # 电池电量
        self.data_carself.soc = msgFusion.soc
        # 故障大类
        self.data_carself.err = msgFusion.state
        # 具体故障码
        if self.data_carself.err == 0x00:
            self.data_carself.errcode = 0x00
        else:
            self.data_carself.errcode = msgFusion.error
        # 经度
        self.data_carself.lon = msgFusion.longitude
        # 纬度
        self.data_carself.lat = msgFusion.latitude
        # 航向角
        self.data_carself.yaw = np.int16(msgFusion.yaw)

        # print(" sub here !!!!!!")
        # print(self.data_carself.car_speed)

        #################################硬件状态数据#######################################
        # 车辆ID
        self.data_hardware.car_id = CAR_ID#np.uint8(msgFusion.id)
        # 红绿灯
        self.data_hardware.light_id=[]
        self.data_hardware.light_state=[]
        self.data_hardware.light_errcode=[]
        for i in range(len(msgFusion.lightdata)//6):
            self.data_hardware.light_id.append(msgFusion.lightdata[i+1])
            if msgFusion.lightdata[6*i+3]==0x04 and msgFusion.lightdata[6*i+4]==0x04 and msgFusion.lightdata[6*i+5]==0x04:
                self.data_hardware.light_state.append(0x02)
            #这里需要写一个故障判断
            else:
                self.data_hardware.light_state.append(0x00)
            self.data_hardware.light_errcode.append(0x00)


        #################################障碍物数据#######################################
        # # 车辆ID
        # self.data_obstacles.car_id = CAR_ID#np.uint8(msgFusion.id)
        # # 车辆障碍物
        # self.data_obstacles.obstacle_class=[]
        # self.data_obstacles.obstacle_lat=[]
        # self.data_obstacles.obstacle_lon=[]
        # for i in range(len(msgFusion.obstacledata)//9):
        #     self.data_obstacles.obstacle_class.append(msgFusion.obstacledata[i*9])
        #     x=msgFusion.obstacledata[i*9+4]
        #     y=msgFusion.obstacledata[i*9+5]
        #     car_lon=msgFusion.longitude
        #     car_lat=msgFusion.latitude
        #     yaw=msgFusion.yaw
        #     lon_lat_hei=tji.rfu_to_gps(np.array([x,y,0]),np.array([car_lon,car_lat,0]),np.array([0,0,yaw]))
        #     self.data_obstacles.obstacle_lon.append(lon_lat_hei[0])
        #     self.data_obstacles.obstacle_lat.append(lon_lat_hei[1])

        #################################软件状态数据#######################################
        self.data_software.car_id = CAR_ID#np.uint8(msgFusion.id)
        self.data_software.spatialFusion_process = np.int16(1000*msgFusion.process_time)

    def thread_orin_hardware(self):
        while True:
            # cpu
            self.data_software.CPU_total_capacity = 100
            self.data_software.CPU_used_capacity = np.uint16(
                round(psutil.cpu_percent(interval=1, percpu=False))
            )

            self.data_software.CPU_used_capacity = round(np.random.random()*100)

            # gpu (using jtop)
            self.data_software.GPU_total_capacity = 100
            try:
                with jtop() as jetson:
                    if jetson.ok():
                        self.data_software.GPU_used_capacity = np.uint16(
                            round(jetson.gpu['val'])
                        )
            except:
                pass
            
            # ram
            self.data_software.memory_total_capacity = np.uint16(
                round(psutil.virtual_memory().total / 1024**3)
            )
            self.data_software.memory_used_capacity = np.uint16(
                round(psutil.virtual_memory().used / 1024**3)
            )

            # disk
            disk_ = psutil.disk_usage("/")
            self.data_software.hardDisk_total_capacity = np.uint16(
                round(disk_.total / 1024**3)
            )
            self.data_software.hardDisk_used_capacity = np.uint16(
                round(disk_.used / 1024**3)
            )

            time.sleep(1)

    def get_data_sql(self,sql_msg):
        self.cur.execute(sql_msg)
        results = self.cur.fetchall()
        return results[-1]

    def get_data_carself1(self):
        sql = "SELECT * FROM data_carself ORDER BY field3 DESC LIMIT 1"
        self.cur.execute(sql)
        results = self.cur.fetchall()
        return results[-1]

    def get_data_carself2(self):
        sql = "SELECT * FROM data_carself2 ORDER BY field3 DESC LIMIT 1"
        self.cur.execute(sql)
        results = self.cur.fetchall()
        return results[-1]

    def get_data_carself3(self):
        sql = "SELECT * FROM data_carself3 ORDER BY field3 DESC LIMIT 1"
        self.cur.execute(sql)
        results = self.cur.fetchall()
        return results[-1]


def main():
    rclpy.init()
    rosNode = NetWork()
    rclpy.spin(rosNode)
    rclpy.shutdown()
