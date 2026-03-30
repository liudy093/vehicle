#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: ***
# @File: pid.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import rclpy
from rclpy.node import Node
import time
import yaml
from math import *
from car_interfaces.msg import *
from easydict import EasyDict

PI = 3.1415926535

ros_node_name = "pid"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

class Pid(Node):
    def __init__(self, name):
        super().__init__(name)
        
        
        # define publishers
        self.pubPid = self.create_publisher(PidInterface, 'pid_data', 10)
        self.timerPid = self.create_timer(0.1, self.pub_callback_pid)
        
        # define subscribers
        self.subLocalPathPlanning = self.create_subscription(LocalPathPlanningInterface, 'local_path_planning_data', self.sub_callback_local_path_planning, 10)
        self.subFusion = self.create_subscription(FusionInterface, 'fusion_data', self.sub_callback_fusion, 10)
        #self.subGPS = self.create_subscription(GpsInterface, 'gps_data', self.sub_callback_gps, 10)

        # #, encoding='utf8'
        # with open("./src/pid/pid/config.json", 'r') as fp:
        #     self.json_data = json.load(fp)
        #     print("Success to open the json")
        #     #print(self.json_data)
        #     self.json_state = True

        
        root_path = os.getcwd() + "/src/pid/pid/"  # 根目录
        # ======================================================== #
        # 从config文件中读取超参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f)
        self.yaml_data = EasyDict(config)
        print(self.yaml_data)

        
        
        self.nowSpeed = 0.0
        self.nowLatitude = 0.0
        self.nowLongitude = 0.0
        self.nowYaw = 0.0
        self.nowPitch = 0.0
        
        self.refSpeed = 0.0
        self.refLatitude = 0.0
        self.refLongitude = 0.0
        self.refYaw = 0.0

        self.TD_refSpeed = 0.0
        self.TD_refSpeedLast = 0.0
        self.TD_nowAccelerationLast = 0.0
        self.TD_nowAcceleration = 0.0
        self.speedControlEii = 0.0
        self.braking_distance = 0.0

        self.control_value_last = 0

        self.turn_signals = 0.0


        self.trajectory_state = False
        self.GPS_state = False
        self.arrived = False

        self.saveData = open("./src/pid/test/controlData.txt","w")


        # self.z1 = 0
        # self.z2 = 0
        # self.s1 = 0
        # self.s2 = 0

        pass
        
    def pub_callback_pid(self):
        """
        Callback of publisher (timer), publish the topic 'pid_data'.
        :param None.
        """
        
        
        msgPid = PidInterface()
        msgPid.timestamp = time.time()


        nowPosX, nowPosY, nowPosZ = self.conversion_of_coordinates(self.nowLatitude, self.nowLongitude, 0)
        refPosX, refPosY, refPosz = self.conversion_of_coordinates(self.refLatitude, self.refLongitude, 0)

        print("nowyaw=", self.nowYaw)
        print("refyaw=", self.refYaw)
        errorYaw=-self.nowYaw+self.refYaw

        if(errorYaw > PI):
            errorYaw=errorYaw - 2*PI
        if(errorYaw < -PI):
            errorYaw=errorYaw + 2*PI
        print("ephi=", errorYaw)

        errorDistance = ((-nowPosY+refPosY)*cos(self.refYaw)-(-nowPosX+refPosX)*sin(self.refYaw))
        
        print("ed=", errorDistance)

        if(self.nowSpeed<3.5):
            pidEphi = self.yaml_data.lateral_ephi_1
            pidEd = self.yaml_data.lateral_ed_1
        elif(self.nowSpeed<5.5):
            pidEphi = self.yaml_data.lateral_ephi_2
            pidEd = self.yaml_data.lateral_ed_2
        elif(self.nowSpeed<7.5):
            pidEphi = self.yaml_data.lateral_ephi_3
            pidEd = self.yaml_data.lateral_ed_3  
        else:
            pidEphi = self.yaml_data.lateral_ephi_4
            pidEd = self.yaml_data.lateral_ed_4


        controlValue = (pidEphi*errorYaw + pidEd*errorDistance)*180/PI #-right +left
        controlValue = controlValue*20.0

        control_number = 0.3
        controlValue = control_number * controlValue + (1 - control_number) * self.control_value_last
        self.control_value_last = controlValue

        if(controlValue > 500):
            controlValue = 500.0
        if(controlValue < -500):
            controlValue = -500.0
        print("control=", controlValue)

        msgPid.timestamp = 0.0
        msgPid.angle = controlValue
        
        #########################
        #self.refSpeed = 3.0
        #########################

        msgPid.velocity = self.refSpeed
        print("refspeed=", msgPid.velocity)
        msgPid.throttle_percentage = 0
        msgPid.braking_percentage = 0
        msgPid.process_time = 0.0

        
        if self.refSpeed < 0.05:
            if self.nowSpeed < 0.1:
                msgPid.throttle_percentage = 0
                msgPid.braking_percentage = 30
            else:
                self.braking_distance = self.braking_distance + self.nowSpeed * 0.1
                msgPid.throttle_percentage = 0
                refBraking = int(self.yaml_data.longitudinal_dec_open_loop * self.braking_distance + self.yaml_data.longitudinal_dec_open_loop_start)
                if refBraking > 125:
                    refBraking = 125
                if refBraking < 0:
                    refBraking = 0
                msgPid.braking_percentage = refBraking


        else:
            self.braking_distance = 0

            #速度PID
            TD_s = 4
            self.TD_refSpeed = self.TD_refSpeedLast + 0.1*self.TD_nowAccelerationLast
            self.TD_nowAcceleration = self.TD_nowAccelerationLast + 0.1*(-TD_s*TD_s*(self.TD_refSpeedLast - self.refSpeed) - 2*TD_s*self.TD_nowAccelerationLast)
            if(self.TD_nowAcceleration < -2):
                self.TD_nowAcceleration = -2
            self.TD_refSpeedLast = self.TD_refSpeed
            self.TD_nowAccelerationLast = self.TD_nowAcceleration



            errorSpeed = self.nowSpeed - self.TD_refSpeed
            print("error speed =", errorSpeed)

            if(errorSpeed >0.5):    #减速
                refBraking = errorSpeed * self.yaml_data.longitudinal_dec_kp
                if(refBraking > 125):
                    refBraking = 125
                if refBraking < 0:
                    refBraking = 0
                msgPid.braking_percentage = int(refBraking)
                msgPid.throttle_percentage = 0
            
            else:                   #加速
                if(abs(errorSpeed) < 1):                                                                                               
                    self.speedControlEii = self.speedControlEii + errorSpeed
                else:
                    self.speedControlEii = 0
                
                if self.nowPitch > 2:
                    accPID_kp = self.yaml_data.longitudinal_acc_slope_kp
                    accPID_ki = self.yaml_data.longitudinal_acc_slope_ki                 
                else:
                    accPID_kp = self.yaml_data.longitudinal_acc_plain_kp
                    accPID_ki = self.yaml_data.longitudinal_acc_plain_ki

                refThrottle = 15-100*(errorSpeed * accPID_kp + self.speedControlEii * accPID_ki)


                if(refThrottle > 100):
                    refThrottle = 100
                elif(refThrottle < 0):
                    refThrottle = 0
                else:
                    pass
                msgPid.throttle_percentage = int(refThrottle)
                msgPid.braking_percentage = 0



        # eso_l1 = 2*5
        # eso_l2 = 5*5
        # self.z1 = self.z1 +0.1 * (self.z2 + msgPid.throttle_percentage + eso_l1*(self.nowSpeed - self.z1))
        # self.z2 = self.z2 +0.1 * (eso_l2*(self.nowSpeed - self.z1))
        # self.s1 = self.s1 +0.1 * (msgPid.throttle_percentage + self.s2)
        
        # eso_l3 = 100
        # self.s2 = self.s2 +0.1 * (eso_l3*(self.nowSpeed - self.s1))

        # errorSpeed1 = self.refSpeed - self.nowSpeed

        # self.saveData.write(str(self.z1))
        # self.saveData.write('\t')
        # self.saveData.write(str(self.z2))
        # self.saveData.write('\t')
        # self.saveData.write(str(self.s1))
        # self.saveData.write('\t')
        # self.saveData.write(str(self.s2))
        # self.saveData.write('\t')
        # self.saveData.write(str(errorSpeed1))
        # self.saveData.write('\n')



        #self.nowYaw+self.refYaw
# ######################
#         self.saveData.write(str(self.refYaw))
#         self.saveData.write('\t')
#         self.saveData.write(str(self.nowYaw))
#         self.saveData.write('\t')
#         self.saveData.write(str(refPosX))
#         self.saveData.write('\t')
#         self.saveData.write(str(refPosY))
#         self.saveData.write('\t')
#         self.saveData.write(str(nowPosX))
#         self.saveData.write('\t')   
#         self.saveData.write(str(nowPosY))
#         self.saveData.write('\t')       
#         self.saveData.write(str(errorDistance))
#         self.saveData.write('\t')
#         self.saveData.write(str(errorYaw))
#         self.saveData.write('\n')
# #####################

        if self.trajectory_state == False or self.GPS_state == False:
            msgPid.throttle_percentage = 0
            msgPid.braking_percentage = 40
            msgPid.angle = 0.0
            self.control_value_last = 0.0
        #print("msgPid.angle = ", msgPid.angle)

        # if msgPid.throttle_percentage > 100:
        #     msgPid.throttle_percentage = 100
        # if msgPid.throttle_percentage < 0:
        #     msgPid.throttle_percentage = 0
        # if msgPid.braking_percentage > 125:
        #     msgPid.braking_percentage = 125
        # if msgPid.braking_percentage < 0:
        #     msgPid.braking_percentage = 0



        
        if self.arrived:
            msgPid.gear = 2
        else:
            msgPid.gear = 3
        #print("gear = ", msgPid.gear)
        msgPid.turn_signals = self.turn_signals


        self.pubPid.publish(msgPid)

        print("throttle = %d, brake = %d"%(msgPid.throttle_percentage, msgPid.braking_percentage))
        pass
        
    def sub_callback_local_path_planning(self, msgLocalPathPlanning:LocalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic 'local_path_planning_data'.
        :param msgLocalPathPlanning: The message heard by the subscriber.
        """
        if msgLocalPathPlanning.latitude is None or len(msgLocalPathPlanning.latitude) == 0:
            self.trajectory_state = False
            return
        
        self.refSpeed = msgLocalPathPlanning.speed[0]

        #print("refspeed###################")
        #print(self.refSpeed)

        if(self.nowSpeed<3.5):
            advanceDistance = self.yaml_data.advance_distance_1
        elif(self.nowSpeed<5.5):
            advanceDistance = self.yaml_data.advance_distance_2
        elif(self.nowSpeed<7.5):
            advanceDistance = self.yaml_data.advance_distance_3
        else:
            advanceDistance = self.yaml_data.advance_distance_4

        self.refLatitude = msgLocalPathPlanning.latitude[advanceDistance]
        self.refLongitude = msgLocalPathPlanning.longitude[advanceDistance]
        self.refYaw = msgLocalPathPlanning.angle[advanceDistance]/180*PI

        self.turn_signals = msgLocalPathPlanning.turn_signals

        #print("len(msgLocalPathPlanning.latitude", len(msgLocalPathPlanning.latitude))
        
        if len(msgLocalPathPlanning.latitude) == 500:
            self.trajectory_state = True
            if abs(msgLocalPathPlanning.latitude[1] - msgLocalPathPlanning.latitude[-1]) < 1e-8 and \
                abs(msgLocalPathPlanning.longitude[1] - msgLocalPathPlanning.longitude[-1]) < 1e-8 and \
                self.nowSpeed < 0.001:
                self.arrived = True
            else:
                self.arrived = False

        else:
            self.trajectory_state = False

        pass
        
    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        self.nowSpeed = msgFusion.carspeed
        print("nowspeed^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^6")
        print(self.nowSpeed) 

        self.nowLatitude = msgFusion.latitude
        self.nowLongitude = msgFusion.longitude
        #print("lat = ", self.nowLatitude)
        #print("lon = ", self.nowLongitude)

        if abs(self.nowLatitude) < 1 and abs(self.nowLongitude) < 1:
            self.GPS_state = False
        else:
            self.GPS_state = True 

        self.nowYaw = msgFusion.yaw/180*PI
        self.nowPitch = msgFusion.pitch
        #print("!!!!!!!!!!!!!!!!Pitch =", self.nowPitch)
        pass

    # def sub_callback_gps(self, msgGps:GpsInterface):
    #     self.nowSpeed = 1.0
    #     self.nowLatitude = msgGps.latitude
    #     self.nowLongitude = msgGps.longitude
    #     self.nowYaw = msgGps.yaw/180*PI
    #     pass
    
    #坐标转换函数,经纬高LLA 转 东北天ENU
    #入口参数：纬度，经度，高度
    #出口参数：局部坐标x，局部坐标y，局部坐标z
    #需要导入math包 from math import *
    def conversion_of_coordinates(self, conversionLatitude, conversionLongitude, conversionAltitude):
        
        #WGS84 参数定义
        wgs84_a  = 6378137.0
        wgs84_f  = 1/298.257223565
        wgs84_e2 = wgs84_f * (2-wgs84_f)
        D2R      = PI / 180.0

        #局部坐标原点经纬度

        #天南街原点
        Latitude0 = 36.65538784
        Longitude0 = 114.58287107
        # Latitude0 = 39.1052154
        # Longitude0 = 117.1641687


        lat0 = Latitude0  *D2R
        lon0 = Longitude0 *D2R
        alt0 = 0
        N0 = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat0)*sin(lat0))
        x0 = (N0 + alt0) * cos(lat0) * cos(lon0)
        y0 = (N0 + alt0) * cos(lat0) * sin(lon0)
        z0 = (N0*(1-wgs84_e2) + alt0) * sin(lat0)

        lat = conversionLatitude  *D2R
        lon = conversionLongitude *D2R
        alt = conversionAltitude
        N = wgs84_a / sqrt(1 - wgs84_e2 * sin(lat)*sin(lat))
        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N*(1-wgs84_e2) + alt) * sin(lat)

        dx = x - x0
        dy = y - y0
        dz = z - z0

        s11 = -sin(lon0)
        s12 = cos(lon0)
        s13 = 0
        s21 = -sin(lat0) * cos(lon0)
        s22 = -sin(lat0) * sin(lon0)
        s23 = cos(lat0)
        s31 = cos(lat0) * cos(lon0)
        s32 = cos(lat0) * sin(lon0)
        s33 = sin(lat0)

        posX = s11 * dx + s12 * dy + s13 * dz
        posY = s21 * dx + s22 * dy + s23 * dz
        posZ = s31 * dx + s32 * dy + s33 * dz

        return posX, posY, posZ


        
def main():
    rclpy.init()
    rosNode = Pid(name='pid')
    rclpy.spin(rosNode)
    rclpy.shutdown()
