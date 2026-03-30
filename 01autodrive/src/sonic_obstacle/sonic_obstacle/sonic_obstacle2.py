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
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "sonic_obstacle"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import rclpy
from rclpy.node import Node
import time
from car_interfaces.msg import *
import serial # 导入串口包
import tjitools
import math
import threading
from threading import Thread
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker

forward_sonic = [[4,-3],[4,-3],[4,-1],[4,-1],[4,1],[4,1],[4,3],[4,3]]
backward_sonic = [[-4,-3],[-4,-1],[-4,1],[-4,3]]
left_sonic = [[-3,4],[-1,4],[1,4],[3,4]]
right_sonic = [[-3,-4],[-1,-4],[1,-4],[3,-4]]

sonic_number = 4
sonic_usb_name = [
    '/dev/sonic1',
    '/dev/sonic2',
    '/dev/sonic3',
    '/dev/sonic4'
]

class SonicObstacle(Node):
    def __init__(self, name):
        super().__init__(name)

        self.pubSonicState = self.create_publisher(SonicStateInterface, 'sonic_state_data', 10)
        self.pubSonicObstacle = self.create_publisher(SonicObstacleInterface, 'sonic_obstacle_data', 10)
        self.timerSonic = self.create_timer(0.1, self.pub_callback_sonic)
        

        self.msgSonicState = [None for i in range(sonic_number)]



        #定义串口接口
        self.serone = serial.Serial("/dev/sonic1",9600,timeout = 1,rtscts=True, dsrdtr=True) 
        # self.serone = serial.Serial("/dev/sonic1",9600) 
        if self.serone.isOpen():
            print("one open success")
        else:
            print("one open failed")
        self.serone.flushInput() # 清空缓冲区
        countone = self.serone.inWaiting() 
        print("串口一 缓存数据个数",countone)
        #定义一个线程，读取串口数据
        getsoniconeData = threading.Thread(target=self.get_sonicone_data)
        getsoniconeData.start()

        # self.sertwo = serial.Serial("/dev/sonic2",9600,timeout = 1,rtscts=True, dsrdtr=True) 
        # # self.sertwo = serial.Serial("/dev/sonic2",9600) 
        # if self.sertwo.isOpen():
        #     print("two open success")
        # else:
        #     print("two open failed")
        # self.sertwo.flushInput() # 清空缓冲区
        # counttwo = self.serone.inWaiting() 
        # print("串口二 缓存数据个数",counttwo)
        # #定义一个线程，读取串口数据
        # getsonictwoData = threading.Thread(target=self.get_sonictwo_data)
        # getsonictwoData.start()

        # self.serthr = serial.Serial("/dev/sonic3",9600,timeout = 1,rtscts=True, dsrdtr=True)
        # # self.serthr = serial.Serial("/dev/sonic3",9600) 
        # if self.serthr.isOpen():
        #     print("thr open success")
        # else:
        #     print("thr open failed") 
        # self.serthr.flushInput() # 清空缓冲区
        # countthr = self.serone.inWaiting() 
        # print("串口三 缓存数据个数",countthr)
        # #定义一个线程，读取串口数据
        # getsonicthrData = threading.Thread(target=self.get_sonicthr_data)
        # getsonicthrData.start()

        # self.serfou = serial.Serial("/dev/sonic4",9600,timeout = 1,rtscts=True, dsrdtr=True) 
        # # self.serfou = serial.Serial("/dev/sonic4",9600) 
        # if self.serfou.isOpen():
        #     print("fou open success")
        # else:
        #     print("fou open failed")
        # self.serfou.flushInput() # 清空缓冲区
        # countfou = self.serone.inWaiting() 
        # print("串口四 缓存数据个数",countfou)
        # #定义一个线程，读取串口数据
        # getsonicfouData = threading.Thread(target=self.get_sonicfou_data)
        # getsonicfouData.start()

        
        pass

    def get_sonicone_data(self):
        print("get_sonicone_data")
        errorone = 0
        while True:
            errorone += 1
            count = self.serone.inWaiting() 

            if errorone > 1000:
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                
                print("sonic1    error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                errorone = 0
                self.serone.close()
                time.sleep(0.1)
                if self.serone.isOpen():
                    self.serone.close()
                else:
                    # self.serone.open()
                    self.serone = serial.Serial("/dev/sonic1",9600,timeout = 1,rtscts=True, dsrdtr=True) 
                
                obj = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]  
                msgSonicObstacle = SonicObstacleInterface()
                msgSonicObstacle.timestamp = time.time()
                msgSonicObstacle.id = 1 
                msgSonicObstacle.number = 0
                msgSonicObstacle.obstacledata= obj
                msgSonicObstacle.process_time = 0.0
                self.pubSonicObstacle.publish(msgSonicObstacle)

                msgSonicState = SonicStateInterface()
                msgSonicState.timestamp = time.time() #时间戳
                msgSonicState.id=0 #导航设备id
                
                msgSonicState.state=1 #设备状态
                msgSonicState.error=1    #错误码
                self.pubSonicState.publish(msgSonicState)

            if count ==10 :
                recv = self.serone.read_all() # 读出串口数据
                
                # print('one recv:',len(recv))

                if len(recv) != 10:
                    continue


                flag = (recv[0] + recv[1] + recv[2] + recv[3] + recv[4] + recv[5] + recv[6] + recv[7] + recv[8]) & 0x00FF

                if recv[0]==0xFF and flag == recv[9]:
                    errorone = 0

                    t1 = time.time()

                    # print("获取Sonicone_state!!!!!!!!")
                    angleone = 0.0
                    numberone = 4
                    data = [recv[1], recv[2], recv[3], recv[4],recv[5], recv[6], recv[7],recv[8]]
                    obj,numberone = self.process_radar_obstacledata(data,angleone)
                    # print("data = ",data, ' obj ', obj)

                    disone = (data[0] * 256 + data[1]) * 0.001
                    distwo = (data[2] * 256 + data[3]) * 0.001
                    disthr = (data[4] * 256 + data[5]) * 0.001
                    disfou = (data[6] * 256 + data[7]) * 0.001

                    all_dis = np.array([disone, distwo, disthr, disfou])
                    if len(all_dis[all_dis < 3.0]) == 0:
                        print("all max than 3")
                        max_dis = 3.0
                    else:
                        max_dis = np.max(all_dis[all_dis < 3.0])

                    if len(all_dis[all_dis > 0.01]) == 0:
                        print("all min than 0.01")
                        min_dis = 0.0
                    else:
                        min_dis = np.min(all_dis[all_dis > 0.01])

                    print("dis = [%.2f, %.2f, %.2f, %.2f], max = %.2f, min = %.2f"%(disone, distwo, disthr, disfou, max_dis, min_dis))

                    t2 = time.time()

                    t3 = t2 - t1

                    # view

                    # self.pubmarkertext.publish(self.markerD)
                    # self.pubmarkerponit.publish(self.markerD)

                    mArrpoint = MarkerArray()
                    mArrtext = MarkerArray()
                    for i, box in enumerate(obj):

                        # print("i= ",i, ' box ', str(box))

                        if i%2 != 0:
                            # print("next if    i= ",i, ' box ', str(box))
                            if box == -1.0:
                                box = 0.0

                            marker_txt = Marker()
                            marker_txt.header.frame_id = "/rslidar"
                            marker_txt.id = i  # must be unique
                            marker_txt.type = Marker.TEXT_VIEW_FACING
                            marker_txt.action = Marker.ADD
                            marker_txt.pose.position.x = forward_sonic[i][1]*1.0
                            marker_txt.pose.position.y = forward_sonic[i][0]*1.0
                            marker_txt.pose.position.z = 1.0
                            marker_txt.color.r = 1.0
                            marker_txt.color.a = 1.0
                            marker_txt.scale.z = 0.5  # size of text
                            marker_txt.text = str(round(box, 3))  # str
                            mArrtext.markers.append(marker_txt)


                            marker_p = Marker()
                            marker_p.header.frame_id = "/rslidar"
                            marker_p.id = i  # must be unique
                            marker_p.type = Marker.CUBE
                            marker_p.action = Marker.ADD
                            marker_p.ns = "basic_shapes"
                            marker_p.pose.position.x = forward_sonic[i][1]*1.0
                            marker_p.pose.position.y = forward_sonic[i][0]*1.0
                            marker_p.pose.position.z = 1.0
                            marker_p.scale.x = 0.4
                            marker_p.scale.y = 0.4
                            marker_p.scale.z = 0.4
                            marker_p.color.r = 1.0
                            marker_p.color.a = 1.0
                            # marker_p.lifetime = rclpy.time.Duration(seconds=1.0)
                            mArrpoint.markers.append(marker_p)

                            
                            
                        
                    self.pubmarkerponitone.publish(mArrpoint)
                    self.pubmarkertextone.publish(mArrtext)

                    mArrpoint.markers.clear()
                    mArrtext.markers.clear()

                    #  view end!!!!


                    msgSonicObstacle = SonicObstacleInterface()
                    msgSonicObstacle.timestamp = time.time()
                    msgSonicObstacle.id = 0 #id
                    msgSonicObstacle.number = numberone 
                    msgSonicObstacle.obstacledata = obj
                    msgSonicObstacle.process_time = t3
                    self.pubSonicObstacle.publish(msgSonicObstacle)

                    msgSonicState = SonicStateInterface()
                    msgSonicState.timestamp = time.time() #时间戳
                    msgSonicState.id=0 #设备id
                    
                    msgSonicState.state=0 #设备状态
                    msgSonicState.error=0    #错误码
                    
                    self.pubSonicState.publish(msgSonicState)

                # else:
                    
                #     print("未获取Sonicone_data..............")
                #     # self.pubmarkertext.publish(self.markerD)
                #     # self.pubmarkerponit.publish(self.markerD)

                #     obj = np.zeros((4)) 
                #     msgSonicObstacle = SonicObstacleInterface()
                #     msgSonicObstacle.timestamp = time.time()
                #     msgSonicObstacle.id = 0 
                #     msgSonicObstacle.number = 0
                #     msgSonicObstacle.obstacledata= obj
                #     msgSonicObstacle.process_time = 0
                #     self.pubSonicObstacle.publish(msgSonicObstacle)

                #     msgSonicState = SonicStateInterface()
                #     msgSonicState.timestamp = time.time() #时间戳
                #     msgSonicState.id=0 #导航设备id
                    
                #     msgSonicState.state=1 #设备状态
                #     msgSonicState.error=1    #错误码
                #     self.pubSonicState.publish(msgSonicState)

            else:        
                self.serone.flushInput() # 清空缓冲区

            time.sleep(0.1) # 延时0.1秒，免得CPU出问题

    def get_sonictwo_data(self):
        print("get_sonictwo_data")
        errortwo = 0
        while True:
            errortwo += 1
            count = self.sertwo.inWaiting() 

            if errortwo > 1000:
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("sonic2       error   ###")    
                errortwo = 0
                self.sertwo.close()
                time.sleep(0.1)
                if self.sertwo.isOpen():
                    self.sertwo.close()
                else:
                    # self.sertwo.open()
                    self.sertwo = serial.Serial("/dev/sonic2",9600,timeout = 1,rtscts=True, dsrdtr=True)
                 
                obj = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]  
                msgSonicObstacle = SonicObstacleInterface()
                msgSonicObstacle.timestamp = time.time()
                msgSonicObstacle.id = 1 
                msgSonicObstacle.number = 0
                msgSonicObstacle.obstacledata= obj
                msgSonicObstacle.process_time = 0.0
                self.pubSonicObstacle.publish(msgSonicObstacle)

                msgSonicState = SonicStateInterface()
                msgSonicState.timestamp = time.time() #时间戳
                msgSonicState.id=1 #导航设备id
                
                msgSonicState.state=1 #设备状态
                msgSonicState.error=1    #错误码
                self.pubSonicState.publish(msgSonicState)
            
            if count !=0 :
                recv = self.sertwo.read_all() # 读出串口数据
                print('two recv:',len(recv))
                # print('recv:',recv)
                if len(recv) != 10:
                    continue

                flag = (recv[0] + recv[1] + recv[2] + recv[3] + recv[4] + recv[5] + recv[6] + recv[7] + recv[8]) & 0x00FF

                if recv[0]==0xFF and flag == recv[9]:
                    errortwo = 0

                    t1 = time.time()

                    print("获取Sonictwo_state!!!!!!!!")

                    angletwo = 90.0
                    numtwo = 4
                    
                    data = [recv[1], recv[2], recv[3], recv[4],recv[5], recv[6], recv[7],recv[8]]
                    obj,numtwo = self.process_radar_obstacledata(data,angletwo)
                    print("data = ",data, ' obj ', obj)

                    t2 = time.time()
                    t3 = t2 - t1

                    # view

                    mArrpoint = MarkerArray()
                    mArrtext = MarkerArray()
                    for i, box in enumerate(obj):

                        # print("i= ",i, ' box ', str(box))

                        if i%2 != 0:
                            # print("next if    i= ",i, ' box ', str(box))
                            if box == -1.0:
                                box = 0.0

                            marker_txt = Marker()
                            marker_txt.header.frame_id = "/rslidar"
                            marker_txt.id = i  # must be unique
                            marker_txt.type = Marker.TEXT_VIEW_FACING
                            marker_txt.action = Marker.ADD
                            marker_txt.pose.position.x = float(3*i - 12)
                            marker_txt.pose.position.y = -float(box) * 30.0
                            marker_txt.pose.position.z = 1.0
                            marker_txt.color.r = 1.0
                            marker_txt.color.a = 1.0
                            marker_txt.scale.z = 1.5  # size of text
                            marker_txt.text = str(round(box, 3))  # str
                            mArrtext.markers.append(marker_txt)

                            marker_p = Marker()
                            marker_p.header.frame_id = "/rslidar"
                            marker_p.id = i  # must be unique
                            marker_p.type = Marker.CUBE
                            marker_p.action = Marker.ADD
                            marker_p.ns = "basic_shapes"
                            marker_p.pose.position.x = float(3*i - 12)
                            marker_p.pose.position.y = -float(box) * 30.0
                            marker_p.pose.position.z = 1.0
                            marker_p.scale.x = 0.4
                            marker_p.scale.y = 0.4
                            marker_p.scale.z = 0.4
                            marker_p.color.g = 1.0
                            marker_p.color.a = 1.0
                            
                            mArrpoint.markers.append(marker_p)

                        
                    self.pubmarkerponittwo.publish(mArrpoint)
                    self.pubmarkertexttwo.publish(mArrtext)

                    mArrpoint.markers.clear()
                    mArrtext.markers.clear()

                    #  view end!!!!

                    
                    # obstacle_list.append(obstacle_information.copy())
                    # # print("obstacle_list",obstacle_list)
                    # # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("obstacle_list",len(obstacle_list))

                    msgSonicObstacle = SonicObstacleInterface()
                    msgSonicObstacle.timestamp = time.time()
                    msgSonicObstacle.id = 1 #id
                    msgSonicObstacle.number = numtwo 
                    msgSonicObstacle.obstacledata = obj
                    msgSonicObstacle.process_time = t3
                    self.pubSonicObstacle.publish(msgSonicObstacle)

                    msgSonicState = SonicStateInterface()
                    msgSonicState.timestamp = time.time() #时间戳
                    msgSonicState.id=1 #设备id
                    
                    msgSonicState.state=0 #设备状态
                    msgSonicState.error=0    #错误码
                    
                    self.pubSonicState.publish(msgSonicState)

                # else:
                    
                #     print("未获取Sonictwo_data..............")
                #     # self.pubmarkertext.publish(self.markerD)
                #     # self.pubmarkerponit.publish(self.markerD)

                #     obj = np.zeros((4)) 
                #     msgSonicObstacle = SonicObstacleInterface()
                #     msgSonicObstacle.timestamp = time.time()
                #     msgSonicObstacle.id = 1 
                #     msgSonicObstacle.number = 0
                #     msgSonicObstacle.obstacledata= obj
                #     msgSonicObstacle.process_time = 0
                #     self.pubSonicObstacle.publish(msgSonicObstacle)

                #     msgSonicState = SonicStateInterface()
                #     msgSonicState.timestamp = time.time() #时间戳
                #     msgSonicState.id=1 #导航设备id
                    
                #     msgSonicState.state=1 #设备状态
                #     msgSonicState.error=1    #错误码
                #     self.pubSonicState.publish(msgSonicState)

            else:    
                self.sertwo.flushInput() # 清空缓冲区
                # self.sertwo.open()

            time.sleep(0.001) # 延时0.1秒，免得CPU出问题

    def get_sonicthr_data(self):
        print("get_sonicthr_data")
        errorthr = 0
        while True:
            errorthr += 1
            count = self.serthr.inWaiting() 

            if errorthr > 1000:
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("sonic3    error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                errorthr = 0
                self.serthr.close()
                if self.serthr.isOpen():
                    self.serthr.close()
                else:
                    # self.serthr.open()
                    self.serthr = serial.Serial("/dev/sonic3",9600,timeout = 1,rtscts=True, dsrdtr=True) 
                obj = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
                msgSonicObstacle = SonicObstacleInterface()
                msgSonicObstacle.timestamp = time.time()
                msgSonicObstacle.id = 2 
                msgSonicObstacle.number = 0
                msgSonicObstacle.obstacledata= obj
                msgSonicObstacle.process_time = 0.0
                self.pubSonicObstacle.publish(msgSonicObstacle)

                msgSonicState = SonicStateInterface()
                msgSonicState.timestamp = time.time() #时间戳
                msgSonicState.id=2 #导航设备id
                
                msgSonicState.state=1 #设备状态
                msgSonicState.error=1    #错误码
                self.pubSonicState.publish(msgSonicState)
            
            if count !=0 :
                recv = self.serthr.read_all() # 读出串口数据
                # print('recv:',recv)
                print('thr recv:',len(recv))
                if len(recv) != 10:
                    continue

                flag = (recv[0] + recv[1] + recv[2] + recv[3] + recv[4] + recv[5] + recv[6] + recv[7] + recv[8]) & 0x00FF

                if recv[0]==0xFF and flag == recv[9]:
                    errorthr = 0

                    t1 = time.time()

                    print("获取Sonicthr_state!!!!!!!!")

                    anglethr = 180.0
                    numthr = 4
                    
                    data = [recv[1], recv[2], recv[3], recv[4],recv[5], recv[6], recv[7],recv[8]]
                    obj,numthr = self.process_radar_obstacledata(data,anglethr)
                    print("data = ",data, ' obj ', obj)

                    t2 = time.time()
                    t3 = t2 - t1

                    # view

                    # self.pubmarkertext.publish(self.markerD)
                    # self.pubmarkerponit.publish(self.markerD)

                    mArrpoint = MarkerArray()
                    mArrtext = MarkerArray()
                    for i, box in enumerate(obj):

                        # print("i= ",i, ' box ', str(box))

                        if i%2 != 0:
                            # print("next if    i= ",i, ' box ', str(box))
                            if box == -1.0:
                                box = 0.0

                            marker_txt = Marker()
                            marker_txt.header.frame_id = "/rslidar"
                            marker_txt.id = i  # must be unique
                            marker_txt.type = Marker.TEXT_VIEW_FACING
                            marker_txt.action = Marker.ADD
                            marker_txt.pose.position.x = -float(box) * 30.0
                            marker_txt.pose.position.y = float(3*i - 12)
                            marker_txt.pose.position.z = 1.0
                            marker_txt.color.r = 1.0
                            marker_txt.color.a = 1.0
                            marker_txt.scale.z = 1.5  # size of text
                            marker_txt.text = str(round(box, 3))  # str
                            mArrtext.markers.append(marker_txt)

                            marker_p = Marker()
                            marker_p.header.frame_id = "/rslidar"
                            marker_p.id = i  # must be unique
                            marker_p.type = Marker.CUBE
                            marker_p.action = Marker.ADD
                            marker_p.ns = "basic_shapes"
                            marker_p.pose.position.x = -float(box) * 30.0
                            marker_p.pose.position.y = float(3*i - 12)
                            marker_p.pose.position.z = 1.0
                            marker_p.scale.x = 0.4
                            marker_p.scale.y = 0.4
                            marker_p.scale.z = 0.4
                            marker_p.color.b = 1.0
                            marker_p.color.a = 1.0
                            mArrpoint.markers.append(marker_p)

                        
                    self.pubmarkerponitthr.publish(mArrpoint)
                    self.pubmarkertextthr.publish(mArrtext)

                    mArrpoint.markers.clear()
                    mArrtext.markers.clear()

                    #  view end!!!!

                    
                    # obstacle_list.append(obstacle_information.copy())
                    # # print("obstacle_list",obstacle_list)
                    # # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("obstacle_list",len(obstacle_list))

                    msgSonicObstacle = SonicObstacleInterface()
                    msgSonicObstacle.timestamp = time.time()
                    msgSonicObstacle.id = 2 #id
                    msgSonicObstacle.number = numthr 
                    msgSonicObstacle.obstacledata = obj
                    msgSonicObstacle.process_time = t3
                    self.pubSonicObstacle.publish(msgSonicObstacle)

                    msgSonicState = SonicStateInterface()
                    msgSonicState.timestamp = time.time() #时间戳
                    msgSonicState.id=2 #设备id
                    
                    msgSonicState.state=0 #设备状态
                    msgSonicState.error=0    #错误码
                    
                    self.pubSonicState.publish(msgSonicState)

                # else:
                    
                #     print("未获取Sonicthr_data..............")
                #     # self.pubmarkertext.publish(self.markerD)
                #     # self.pubmarkerponit.publish(self.markerD)

                #     obj = np.zeros((4)) 
                #     msgSonicObstacle = SonicObstacleInterface()
                #     msgSonicObstacle.timestamp = time.time()
                #     msgSonicObstacle.id = 2 
                #     msgSonicObstacle.number = 0
                #     msgSonicObstacle.obstacledata= obj
                #     msgSonicObstacle.process_time = 0
                #     self.pubSonicObstacle.publish(msgSonicObstacle)

                #     msgSonicState = SonicStateInterface()
                #     msgSonicState.timestamp = time.time() #时间戳
                #     msgSonicState.id=2 #导航设备id
                    
                #     msgSonicState.state=1 #设备状态
                #     msgSonicState.error=1    #错误码
                #     self.pubSonicState.publish(msgSonicState)

            
            else:    
                self.serthr.flushInput() # 清空缓冲区
                # self.serthr.open()

            time.sleep(0.001) # 延时0.1秒，免得CPU出问题

    def get_sonicfou_data(self):
        print("get_sonicfou_data")
        errorfou = 0
        while True:
            errorfou += 1
            count = self.serfou.inWaiting() 

            if errorfou > 1000:
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("#########################################################################")
                print("sonic4    error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                errorfou = 0

                self.serfou.close()
                
                time.sleep(0.1)
                if self.serfou.isOpen():
                    self.serfou.close()
                else:
                    # self.serfou.open()
                    self.serfou = serial.Serial("/dev/sonic4",9600,timeout = 1,rtscts=True, dsrdtr=True) 
                
                obj = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0] 
                msgSonicObstacle = SonicObstacleInterface()
                msgSonicObstacle.timestamp = time.time()
                msgSonicObstacle.id = 3 
                msgSonicObstacle.number = 0
                msgSonicObstacle.obstacledata= obj
                msgSonicObstacle.process_time = 0.0
                self.pubSonicObstacle.publish(msgSonicObstacle)

                msgSonicState = SonicStateInterface()
                msgSonicState.timestamp = time.time() #时间戳
                msgSonicState.id=3 #导航设备id
                
                msgSonicState.state=1 #设备状态
                msgSonicState.error=1    #错误码
                self.pubSonicState.publish(msgSonicState)
            
            if count !=0 :
                recv = self.serfou.read_all() # 读出串口数据
                # print('recv:',recv)
                print('fou recv:',len(recv))
                if len(recv) != 10:
                    continue

                flag = (recv[0] + recv[1] + recv[2] + recv[3] + recv[4] + recv[5] + recv[6] + recv[7] + recv[8]) & 0x00FF

                if recv[0]==0xFF and flag == recv[9]:
                    errorfou = 0

                    t1 = time.time()

                    print("获取Sonicfou_state!!!!!!!!")

                    anglefou = 270.0
                    numfou = 4
                    
                    data = [recv[1], recv[2], recv[3], recv[4],recv[5], recv[6], recv[7],recv[8]]
                    obj,numfou = self.process_radar_obstacledata(data,anglefou)
                    print("data = ",data, ' obj ', obj)

                    t2 = time.time()
                    t3 = t2 - t1

                    # view

                    # self.pubmarkertext.publish(self.markerD)
                    # self.pubmarkerponit.publish(self.markerD)

                    mArrpoint = MarkerArray()
                    mArrtext = MarkerArray()
                    for i, box in enumerate(obj):

                        # print("i= ",i, ' box ', str(box))

                        if i%2 != 0:
                            # print("next if    i= ",i, ' box ', str(box))
                            if box == -1.0:
                                box = 0.0

                            marker_txt = Marker()
                            marker_txt.header.frame_id = "/rslidar"
                            marker_txt.id = i  # must be unique
                            marker_txt.type = Marker.TEXT_VIEW_FACING
                            marker_txt.action = Marker.ADD
                            marker_txt.pose.position.x = float(3*i - 12)
                            marker_txt.pose.position.y = float(box) * 30.0
                            marker_txt.pose.position.z = 1.0
                            marker_txt.color.r = 1.0
                            marker_txt.color.a = 1.0
                            marker_txt.scale.z = 1.5  # size of text
                            marker_txt.text = str(round(box, 3))  # str
                            mArrtext.markers.append(marker_txt)

                            marker_p = Marker()
                            marker_p.header.frame_id = "/rslidar"
                            marker_p.id = i  # must be unique
                            marker_p.type = Marker.CUBE
                            marker_p.action = Marker.ADD
                            marker_p.ns = "basic_shapes"
                            marker_p.pose.position.x = float(3*i - 12)
                            marker_p.pose.position.y = float(box) * 30.0
                            marker_p.pose.position.z = 1.0
                            marker_p.scale.x = 0.4
                            marker_p.scale.y = 0.4
                            marker_p.scale.z = 0.4
                            marker_p.color.r = 1.0
                            # marker_p.color.g = 1.0
                            marker_p.color.a = 1.0
                            
                            mArrpoint.markers.append(marker_p)

                            
                        
                    self.pubmarkerponitfou.publish(mArrpoint)
                    self.pubmarkertextfou.publish(mArrtext)

                    mArrpoint.markers.clear()
                    mArrtext.markers.clear()

                    #  view end!!!!
                    
                    # obstacle_list.append(obstacle_information.copy())
                    # # print("obstacle_list",obstacle_list)
                    # # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print("obstacle_list",len(obstacle_list))

                    msgSonicObstacle = SonicObstacleInterface()
                    msgSonicObstacle.timestamp = time.time()
                    msgSonicObstacle.id = 3 #id
                    msgSonicObstacle.number = numfou 
                    msgSonicObstacle.obstacledata = obj
                    msgSonicObstacle.process_time = t3
                    self.pubSonicObstacle.publish(msgSonicObstacle)

                    msgSonicState = SonicStateInterface()
                    msgSonicState.timestamp = time.time() #时间戳
                    msgSonicState.id=3 #设备id
                    
                    msgSonicState.state=0 #设备状态
                    msgSonicState.error=0    #错误码
                    
                    self.pubSonicState.publish(msgSonicState)

                # else:
                    
                #     print("未获取Sonicfou_data..............")
                #     # self.pubmarkertext.publish(self.markerD)
                #     # self.pubmarkerponit.publish(self.markerD)
                    
                #     obj = np.zeros((4)) 
                #     msgSonicObstacle = SonicObstacleInterface()
                #     msgSonicObstacle.timestamp = time.time()
                #     msgSonicObstacle.id = 3 
                #     msgSonicObstacle.number = 0
                #     msgSonicObstacle.obstacledata= obj
                #     msgSonicObstacle.process_time = 0
                #     self.pubSonicObstacle.publish(msgSonicObstacle)

                #     msgSonicState = SonicStateInterface()
                #     msgSonicState.timestamp = time.time() #时间戳
                #     msgSonicState.id=3 #导航设备id
                    
                #     msgSonicState.state=1 #设备状态
                #     msgSonicState.error=1    #错误码
                #     self.pubSonicState.publish(msgSonicState)

            
            else:    
                self.serfou.flushInput() # 清空缓冲区
                # self.serfou.open()

            time.sleep(0.001) # 延时0.1秒，免得CPU出问题

    def process_radar_obstacledata(self, data, angle):

        # print("data = ",data)

        # disone = round( ( (data[0] * 256 + data[1]) * 0.001) , 2)
        # distwo = round( ( (data[2] * 256 + data[3]) * 0.001) , 2)
        # disthr = round( ( (data[4] * 256 + data[5]) * 0.001) , 2)
        # disfou = round( ( (data[6] * 256 + data[7]) * 0.001) , 2)

        disone = (data[0] * 256 + data[1]) * 0.001
        distwo = (data[2] * 256 + data[3]) * 0.001
        disthr = (data[4] * 256 + data[5]) * 0.001
        disfou = (data[6] * 256 + data[7]) * 0.001

        num = 4

        if disone ==0 :
            num -= 1
            disone = -1.0
        if distwo ==0 :
            num -= 1
            distwo = -1.0
        if disthr ==0 :
            num -= 1
            disthr = -1.0
        if disfou ==0 :
            num -= 1
            disfou = -1.0

        # print('number :',num)

        return [angle,disone, angle,distwo, angle,disthr, angle,disfou],num
        
    

        
def main():
    rclpy.init()
    rosNode = SonicObstacle(name='sonic_obstacle')
    rclpy.spin(rosNode)
    rclpy.shutdown()
