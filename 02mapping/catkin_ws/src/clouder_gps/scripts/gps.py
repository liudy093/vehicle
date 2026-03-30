#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pyproj
import serial
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

class GPSToXYConverter:
    def __init__(self, lat0, lon0):
        """
        地图原点经纬度(lat0, lon0)。
        lat 纬度
        lon 经度
        """
        self.projection = pyproj.Proj(f'+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs')

    def gps_to_xy(self, lat, lon):
        """
        将GPS坐标转换为地图平面坐标系米
        """
        x, y = self.projection(lon, lat)
        return x, y

if __name__ == '__main__':
    #ser = serial.Serial('/dev/ttyUSB1', 115200)  # 创建一个串口对象,检测数据输入
    ser = serial.Serial('/dev/ttyUSB1', 115200)
    i=0
    lat0 = 0 #纬度 
    lon0 = 0 #经度
    datax = []
    datay = []
    pub = rospy.Publisher('gps', Pose2D, queue_size=10)  
    rospy.init_node('GPS_pub', anonymous=False)
    rate = rospy.Rate(50)
    plt.figure()
    x=0
    y=0
    GPS_msg=Pose2D()

    line = ser.readline()
    #line = str(line,encoding= 'utf-8',errors='ignore')

    while not rospy.is_shutdown():
        line = ser.readline()
        line = str(line,encoding= 'utf-8',errors='ignore')

        # if  i<=1:
        #     i+=1
        #     lat0 = float(line.split(',')[3])
        #     lon0 = float(line.split(',')[2])
        #     continue

        # if i ==2:
        #     print("Location Module Working...")
            
        # i+=1

        #base point
        # lat0 = 39.95757040
        # lon0 = 116.30460026
        lat0 = 39.95754418
        lon0 = 116.30460525
        gps_converter = GPSToXYConverter(lat0, lon0)
        lat = float(line.split(',')[3])
        lon = float(line.split(',')[2])
        theta = float(line.split(',')[5])

        x, y = gps_converter.gps_to_xy(lat, lon)

        #from (0-360) to +-(0-180)
        if theta -90 > 180:
            theta = 360 - theta + 90 
        else:
            theta = -theta + 90

        #rospy.loginfo(x)
        #rospy.loginfo(y)
        #datax.append(float(x))
        #datay.append(float(y))

        GPS_msg.x=x
        GPS_msg.y=y
        GPS_msg.theta = theta *3.1415926/180
        #degree to rad
        
        pub.publish(GPS_msg)    
        rate.sleep()

        #if i ==1500:
            #print(i)
            #print(len(datax))
            #plt.scatter(datax,datay)
            # plt.show()





