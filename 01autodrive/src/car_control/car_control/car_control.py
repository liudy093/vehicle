#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: DYX
# @File: car_control.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import rclpy
from rclpy.node import Node
import time
from car_interfaces.msg import *
import can
import math
import threading
from threading import Thread


ros_node_name = "car_control"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class CarControl(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.carControlBus = can.interface.Bus(channel='can1', bustype='socketcan')
        # define publishers
        
        # define subscribers
        self.subPid = self.create_subscription(PidInterface, 'pid_data', self.sub_callback_pid, 10)
        self.subRegulator = self.create_subscription(RegulatorInterface, 'regulator_data', self.sub_callback_regulator, 10)



        #getCarOriData = threading.Thread(target=self.cantest)
        #getCarOriData.start()

        self.ignore_regulator = True
        self.whetherStop = 0

        

    def cantest(self):
        contourLamp=1
        # 近光灯 0：关闭   1：打开
        dippedHeadlight=1
        # 超声波开关 0：开启   1：关闭
        ultrasonicSwitch=0
        # 使能信号    0：未使能        1：使能
        enableSignal=0
        # 挡位 0x00： P 档；  0x01：倒档R；  0x02：N 档；  0x03：D 档
        gearpos=3
        # 目标车速
        targetSpeed=110
        # 转向角度
        SteeringAngle=380
        #制动使能 1：使能刹车    0：不使能刹车
        brakeEnable=0
        # 制动压力请求
        brakePressureRequest=0
        # 警报灯
        alarmLamp=1
        # 转向灯控制
        turnSignalControl=2
        #APP接入
        appInsert=1

        ## 数据编码
        Byte7=appInsert<<7 | turnSignalControl<<1 | alarmLamp
        Byte6=brakePressureRequest<<1 | brakeEnable
        Byte5=(SteeringAngle&0xFF00)>>8
        Byte4=SteeringAngle & 0x00FF
        Byte3=0
        Byte2=((targetSpeed*10)&0xFF00)>>8
        Byte1=(targetSpeed*10) & 0x00FF
        Byte0=gearpos<<6 | enableSignal<<5 | ultrasonicSwitch<<4 | dippedHeadlight<<1 | contourLamp
        thedata=[Byte7,Byte6,Byte5,Byte4,Byte3,Byte2,Byte1,Byte0] ####每一个byte是8位，是否需要写成8位2进制的形式，还是直接给byte赋10进制或16进制的值即可呢？
        
        print("get_car_ori_data")
        while 1:    
            print("test!!!")
            thedata=[100,10,1,20,30,49,101,20]
            carControlMsg = can.Message(arbitration_id = 0x201, data=thedata, extended_id = False)
            self.carControlBus.send(carControlMsg)     
            time.sleep(1)                      
    
        pass




    def sub_callback_pid(self, msgPid:PidInterface):
        """
        Callback of subscriber, subscribe the topic 'pid_data'.
        :param msgPid: The message heard by the subscriber.
        """


        ## 需要的数据
        # 轮廓灯 0：关闭   1：打开
        contourLamp = 0

        # 近光灯 0：关闭   1：打开
        dippedHeadlight = 0

        # 超声波开关 0：开启   1：关闭
        ultrasonicSwitch = 0

        # 使能信号    0：未使能        1：使能
        enableSignal = 1

        # 挡位 0x00： P 档；  0x01：倒档R；  0x02：N 档；  0x03：D 档
        gearpos = msgPid.gear


        # 目标车速
        if 0 == self.whetherStop or self.ignore_regulator == True:
            refThrottle = msgPid.throttle_percentage
            refbraking = msgPid.braking_percentage
        else:
            refThrottle = 0
            refbraking = 100
        
        refbraking = int(refbraking/125*60)

        # 转向角度 rad
        SteeringAngle = msgPid.angle

        #制动使能 1：使能刹车    0：不使能刹车
        brakeEnable = 0

        # 制动压力请求
        if(refbraking !=0):
            brakeEnable = 1
        else:
            brakeEnable = 0

        
        # 警报灯
        alarmLamp = 0
        
        # 转向灯控制

        if msgPid.turn_signals == -1:
            turnSignalControl = 1
        elif msgPid.turn_signals == 1:            
            turnSignalControl = 2
        else:
            turnSignalControl = 0
        
        #APP接入
        appInsert = 0

        # SteeringAngle = 30
        # refbraking = 0
        # brakeEnable = 0
        # refThrottle = 30


        ## 数据编码
        Byte7 = appInsert<<7 | turnSignalControl<<1 | alarmLamp
        Byte6 = refbraking <<1 | brakeEnable
        
        # Byte5=((SteeringAngle*180/math.pi)&0xFF00)>>8
        # Byte4=(SteeringAngle*180/math.pi) & 0x00FF
        Byte5=(int(SteeringAngle)&0xFF00)>>8
        Byte4=int(SteeringAngle) & 0x00FF
        
        Byte3=0
        Byte2=((refThrottle*10)&0xFF00)>>8
        Byte1=(refThrottle*10) & 0x00FF
        Byte0=gearpos<<6 | enableSignal<<5 | ultrasonicSwitch<<4 | dippedHeadlight<<1 | contourLamp
        
        
        canData=[Byte0,Byte1,Byte2,Byte3,Byte4,Byte5,Byte6,Byte7] ####每一个byte是8位，是否需要写成8位2进制的形式，还是直接给byte赋10进制或16进制的值即可呢？
        print("test!!!")
        carControlMsg = can.Message(arbitration_id = 0x210, data=canData, extended_id = False)
        self.carControlBus.send(carControlMsg) 

        pass
        
    def sub_callback_regulator(self, msgRegulator:RegulatorInterface):
        """
        Callback of subscriber, subscribe the topic 'regulator_data'.
        :param msgRegulator: The message heard by the subscriber.
        """
        self.whetherStop = msgRegulator.stop

        pass
        
def main():
    rclpy.init()
    rosNode = CarControl(name='car_control')
    rclpy.spin(rosNode)
    rclpy.shutdown()
