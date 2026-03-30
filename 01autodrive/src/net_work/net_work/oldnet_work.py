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
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

import tjitools as tji

CAR_ID = 0x01
ALL_ID = [0x01, 0x02, 0x03]
DATA_TYPE = ['SEND', 'RECEIVE']

# user infos for ali cloud
DBHOST = 'rm-2zec5959a1jnx38j01o.mysql.rds.aliyuncs.com'
DBUSER = 'fuyun'
DBPASS = 'Kwy26538817'
DBNAME = 'fuyun_test'


class Data(ABC):
    def __init__(self, data_type='SEND'):
        self.type = data_type

    def send_data(self):
        pass

    def receive_data(self, data_list):
        pass

    def before_send_data(self):
        if self.type == 'SEND':
            pass
        else:
            raise ValueError('The data type is not SEND.')

    def before_receive_data(self):
        if self.type == 'RECEIVE':
            pass
        else:
            raise ValueError('The data type is not RECEIVE.')


class DataCarSelf(Data):
    def __init__(self, data_type='SEND'):
        super().__init__(data_type)
        self.data_ecd = '<BBBQBBfBBBBffHBB'

        self.start1 = 0x23
        self.start2 = 0x23
        self.car_id = CAR_ID
        self.time = 0
        self.stateid = 0xCF
        self.car_mode = 0x03
        self.car_speed = 0
        self.car_angle = 0x1E
        self.soc = 0
        self.err = 0x00
        self.errcode = 0x00
        self.lon = 0
        self.lat = 0
        self.yaw = 90
        self.end1 = 0x24
        self.end2 = 0x24

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

    def receive_data(self, data_list):
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
         self.end2] = data_list

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



class NetWork(Node):

    def __init__(self):
        super().__init__(ros_node_name)


        self.timerNetStartEndPoint = self.create_timer(0.1, self.pub_callback_net_start_end_point)

        # define subscribers
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 1)

        self.db = pymysql.connect(host=DBHOST, user=DBUSER, password=DBPASS, database=DBNAME)
        self.cur = self.db.cursor()

        self.data_carself = DataCarSelf(data_type="SEND")


    def pub_callback_net_start_end_point(self):
        """
        Callback of publisher (timer), publish the topic 'net_start_end_point_data'.
        :param None.
        """
        ts = time.time()
        ######################################################################
        # 时间戳
        self.data_carself.time = np.int64(ts * 1000)

        value_data_carself = self.data_carself.send_data()

        fields = ', '.join(['field%d' % (i) for i in range(len(value_data_carself))])
        placeholders = ', '.join(['%s'] * len(value_data_carself))
        if CAR_ID != 1:
            sql = "INSERT INTO data_carself%d (%s) VALUES (%s)" % (CAR_ID, fields, placeholders)
        else:
            sql = "INSERT INTO data_carself (%s) VALUES (%s)" % (fields, placeholders)

        self.cur.execute(sql, value_data_carself)
        self.db.commit()
        print('Vehilce id: %d' % (self.data_carself.car_id))
        print("data carself: ", value_data_carself[3])


    def sub_callback_fusion(self, msgFusion: FusionInterface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        ###############################车辆本体数据########################################
        # 车辆ID
        self.data_carself.car_id = CAR_ID  # np.uint8(msgFusion.id)
        # 车辆速度
        self.data_carself.car_speed = msgFusion.carspeed
        # 车辆转角
        self.data_carself.car_angle = round(msgFusion.steerangle / 540 * 30) + 30
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


    def get_data_sql(self, sql_msg):
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
