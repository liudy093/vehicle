#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: DYX
# @File: car_ori.py
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
import numpy as np
import copy
import threading
from threading import Thread
import math

ros_node_name = "car_ori"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))


class CarOri(Node):
    def __init__(self, name): 
        super().__init__(name)


        self.carOriState = False
        self.carOriData201 = [0,0,0,0,0,0,0,0] #np.zeros(8)
        self.carOriData202 = [0,0,0,0,0,0,0,0] 
        self.carOriData203 = [0,0,0,0,0,0,0,0] 

        self.pubCanState = self.create_publisher(CanStateInterface, 'can_state_data', 10)
        self.timerCanState = self.create_timer(0.1, self.pub_callback_can_state)
        self.pubCarOri = self.create_publisher(CarOriInterface, 'car_ori_data', 10)        
        self.timerCarOri = self.create_timer(0.1, self.pub_callback_car_ori)

        # try:
        #     self.carOriBus = can.interface.Bus(channel='can1', bustype='socketcan')
        # except: 
        #     print('端口连接失败,错误原因')
        #     self.carOriState = False
        
        # if True == self.carOriState:
        #     #定义一个线程，读取can线数据
        #     getCarOriData = threading.Thread(target=self.get_car_ori_data)
        #     getCarOriData.start()

        self.carOriBus = can.interface.Bus(channel='can1', bustype='socketcan')
        getCarOriData = threading.Thread(target=self.get_car_ori_data)
        getCarOriData.start()

                
        pass
    
    def get_car_ori_data(self):
        print("get_car_ori_data")
        while True:
            carOriData = self.carOriBus.recv()
            if (carOriData.arbitration_id == 0x0201): 
                # print("ID=201")
                self.carOriData201 = carOriData.data.copy()
                self.carOriState = 1
            elif (carOriData.arbitration_id == 0x0202):
                # print("ID=202")
                self.carOriData202 = carOriData.data.copy()        
                self.carOriState = 1
            elif (carOriData.arbitration_id == 0x0203):
                # print("ID=203")
                self.carOriData203 = carOriData.data.copy()
                self.carOriState = 1
            else:
                pass
    
        pass
    
    def pub_callback_can_state(self):
        """
        Callback of publisher (timer), publish the topic 'can_state_data'.
        :param None.
        """
        msgCanState = CanStateInterface()
        #时间戳
        msgCanState.timestamp = time.time()
        ## 获取不到的数据
        # CANID
        msgCanState.id= 0x01

        if(True == self.carOriState):
            # 设备状态，0：状态正常，1：状态异常
            msgCanState.state = 0
             # 错误码
            msgCanState.error = 0
        else:
            msgCanState.state = 1
             # 错误码
            msgCanState.error = 2
            print("can error")

        self.pubCanState.publish(msgCanState)
        pass
        
    def pub_callback_car_ori(self):
        """
        Callback of publisher (timer), publish the topic 'car_ori_data'.
        :param None.
        """
        msgCarOri = CarOriInterface()
        now_ts = time.time()
        msgCarOri.timestamp = now_ts
        # 车辆ID, 手动设置
        msgCarOri.id = 0x01

        #ID=201
        #驾驶模式
        msgCarOri.car_run_mode = (self.carOriData201[0])&0x03
        if(msgCarOri.car_run_mode == 0x00):
            msgCarOri.car_run_mode = 0
        elif(msgCarOri.car_run_mode == 0x01):
            msgCarOri.car_run_mode = 1
        elif(msgCarOri.car_run_mode == 0x02):
            msgCarOri.car_run_mode = 2
        else:
            pass

        #喇叭模式
        speaker = (self.carOriData201[0]>>6)&0x01
        if(speaker  == 0x00):
            msgCarOri.speaker  = False
        elif(speaker  == 0x01):
            msgCarOri.speaker  = True
        else:
            pass


        # 车辆档位信号
        msgCarOri.gearpos= (self.carOriData201[0]>>2)&0x03
        if(msgCarOri.gearpos==0x00):
            msgCarOri.gearpos=1
        elif(msgCarOri.gearpos==0x01):
            msgCarOri.gearpos=4
        elif(msgCarOri.gearpos==0x02):
            msgCarOri.gearpos=2
        elif(msgCarOri.gearpos==0x03):
            msgCarOri.gearpos=3
        else:
            msgCarOri.gearpos=5
        #print("gearpos")
        #print(msgCarOri.gearpos)



        # 车辆转角，左转为正，右转为负 
        if ((self.carOriData201[2]&0x80)>>7)==1:
            msgCarOri.steerangle=-1.0*((~((self.carOriData201[2]<< 8 | self.carOriData201[1] )-0X01)) &0xffff)  #*math.pi/180
        else:
            msgCarOri.steerangle=1.0*(self.carOriData201[2] << 8 | self.carOriData201[1] )                      #*math.pi/180
        #print("steerangle")
        #print(msgCarOri.steerangle)

        #油门踏板开度： 取值0～100
        # throttle_percentage_signal=(self.carOriData204[6]&0x01)
        # if(throttle_percentage_signal==0):
        #     print("无信号")
        # else:
        #     msgCarOri.throttle_percentage=(self.carOriData204[6]>>1&0x7F)
        #     print("throttle_percentage")
        #     print(msgCarOri.throttle_percentage)   
        msgCarOri.throttle_percentage = int((self.carOriData201[7] << 8 | self.carOriData201[6] ) *0.1)
        #print("throttle_percentage")
        #print(msgCarOri.throttle_percentage)   

                
        #ID=202

        #ID=203        
        #刹车
        #msgCarOri.braking_percentage = int((self.carOriData203[6] << 8 | self.carOriData203[7] )*100/125)
        msgCarOri.braking_percentage = int((self.carOriData203[0])*100/60)
        msgCarOri.braking_percentage = max(msgCarOri.braking_percentage, 0)
        msgCarOri.braking_percentage = min(msgCarOri.braking_percentage, 100)

        print("braking_percentage")
        print(msgCarOri.braking_percentage)

        # 车辆速度信号        
        msgCarOri.carspeed=(self.carOriData203[3] << 8 | self.carOriData203[2] ) *0.1/3.6
        #print("carspeed")
        #print(msgCarOri.carspeed)

        #SOC状态
        SOC0=self.carOriData203[6] 
        # if(SOC0==0xFE):
        #     print("异常")
        # elif(SOC0==0xFF):
        #     print("无效")
        # else:
        #     msgCarOri.soc=SOC0
        #     print("soc")
        #     print(msgCarOri.soc)
        msgCarOri.soc = SOC0
        print("soc = %d"%msgCarOri.soc)
        #print("soc")
        #print(msgCarOri.soc)


        #左转向灯状态：0：关闭，1：打开
        msgCarOri.left_light=bool((self.carOriData203[7]>>4)&0x01)
        #右转向灯状态：0：关闭，1：打开
        msgCarOri.right_light=bool((self.carOriData203[7]>>5)&0x01)
        

        #不能读取的信息
        # 制动量（-50-50nm）
        msgCarOri.braketq=0.0  
        # 制动状态（00：驻车状态，01：驻车释放状态）  
        msgCarOri.parkingstate=0  
        # 电池放电电流（0-100A）  
        msgCarOri.batterydischargecur=0    

        #倒车灯状态：0：关闭，1：打开  
        msgCarOri.reversing_light=False   

        #启动按钮状态：0：按键无效，1：按键有效    
        msgCarOri.start_button=False
        #急停按钮状态：0：按键无效，1：按键有效
        msgCarOri.stop_button=False
        # 设备状态，0：状态正常，1：电池箱报警；2：电机控制器报警     
        msgCarOri.state=0
        # 错误码；电池箱报警：1：单体过压或欠压，2：放电电流异常，3：电压报警，4：电池温度报警，5：电池SOC过低。电机控制器报警：1：转向电机控制器故障，2：驱动电机控制器故障          
        msgCarOri.error=0      
        # 进程处理时间    
        msgCarOri.process_time=time.time() - now_ts 

        #publish msgs
        self.pubCarOri.publish(msgCarOri)
        pass
        
def main():
    rclpy.init()
    rosNode = CarOri(name='car_ori')
    rclpy.spin(rosNode)
    rclpy.shutdown()
