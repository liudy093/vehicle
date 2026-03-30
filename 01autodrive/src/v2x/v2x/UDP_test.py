#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: YK
@说明: V2X    upgrade time: 6.19 12:20
"""
import os, time
import sys
ros_node_name = "v2x"
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import socket
import json
from datetime import datetime                 # ROS2 节点类
import threading

class V2X:
    def __init__(self):
        ############################ 初始化默认参数 #################################

        # 主机IP
        self.HOST = '192.168.20.224'
        self.PORT = 30301
        # OBU IP 
        self.OBU_HOST = '192.168.20.199'
        self.OBU_PORT = 30300
        
        print(self.HOST)
        self.server = socket.socket(type=socket.SOCK_DGRAM)
        # self.server.setblocking(False) #非阻塞模式，读不到数据
        self.server.bind((self.HOST, self.PORT))

        self.v2xPlanner_thread = threading.Thread(target=self.recv)
        self.v2xPlanner_thread.start()
    def send(self):
        outData = {
            "deviceNo":3,
            "elevation":0,
            "latitude":111.1,
            "longitude":222.2,
            "tag":1001,
            "timestamp":int(time.time())     # 时间戳 10位
        }
        OUB_addr = (self.OBU_HOST,self.OBU_PORT)
        self.server.sendto(json.dumps(outData).encode('gbk'),OUB_addr)
        print("send data to OBU!!!")
    
    def recv(self):
        while True:
            data, address = self.server.recvfrom(9000)
            # data = data.decode("UTF-8")
            print("recv data from OBU!!!")
            print(data)


        

def main():                                          
    v2x = V2X()               
    print(11111)  
    while True:
        v2x.send()
        time.sleep(1)

main()