"""
Author: chen fang
CreateTime: 2023/5/31 上午11:16
File: record_track.py
Description: 
"""

import sys
import os
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "record_track"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

ros_path = '/opt/ros/foxy/lib/python3.8/site-packages'
msg_path = os.getcwd() + '/install/car_interfaces/lib/python3.8/site-packages'

sys.path.append(ros_path)
sys.path.append(msg_path)
sys.path.append("..")

import json
import time
import math


class RecordTrack(Node):
    def __init__(self, name):
        super().__init__(name)
        # ======================================================== #
        # 输如一些信息
        print("------------- Record Track ---------------")
        print("请注意！！！！！最后录制完成的道路轨迹，laneID必须满足0、1、2.....")
        print("即道路的id必须从0开始，且连续。例如只有 1、2 或者 0、2 这些情况会在规划中报错")
        json_name = input("输入道路ID(例如： 01 ): ")
        self.json_path = os.getcwd() + "/src/parking_planning/parking_planning/map/{}.json".format(json_name)
        # 先判断是否存在该文件
        if os.path.isfile(self.json_path):
            while True:
                is_new_json = input("{}.json 文件已存在，是否替换文件(y or n): ".format(json_name))
                if is_new_json == 'y' or is_new_json == 'Y':
                    os.remove(self.json_path)
                    os.mknod(self.json_path)
                    with open(self.json_path, 'w') as f:
                        json.dump({}, f)
                    break
                elif is_new_json == 'n' or is_new_json == 'N':
                    break
                else:
                    continue
        else:
            os.mknod(self.json_path)
            with open(self.json_path, 'w') as f:
                json.dump({}, f)

        # 打开json文件
        with open(self.json_path, 'r', encoding='utf-8') as fp:
            self.json_data = json.load(fp)

        if len(self.json_data) > 0:
            # 对字典中的键进行排序
            self.json_data = dict(sorted(self.json_data.items(), key=lambda x: x[0]))
            for key, val in self.json_data.items():
                print(key, " points num: ", len(val["data"]))

        while True:
            lane_id = input("输入车道线ID(例如： 1 ): ")
            lane_id = str(int(lane_id))
            self.lane_name = "lane" + str(lane_id)
            # 如果lane_id存在
            if self.json_data.get(self.lane_name) is not None:
                is_new_lane = input("lane ID 已存在，是否删除(y or n): ")
                if is_new_lane == 'y' or is_new_lane == 'Y':
                    self.json_data.pop(self.lane_name, None)
                    self.json_data[self.lane_name] = {}
                    break
                elif is_new_json == 'n' or is_new_json == 'N':
                    print("本程序不支持继续存储轨迹！！！！！")
                    sys.exit()
                else:
                    continue
            else:
                self.json_data[self.lane_name] = {}
                break

        lane_left_width = input("输入距离左车道线的距离:  ")
        lane_right_width = input("输入距离右车道线的距离: ")

        self.json_data[self.lane_name]["data"] = []
        self.json_data[self.lane_name]["lane_id"] = int(lane_id)
        self.json_data[self.lane_name]["lane_left_width"] = float(lane_left_width)
        self.json_data[self.lane_name]["lane_right_width"] = float(lane_right_width)

        self.last_x = 0
        self.last_y = 0
        self.start_flag = True

        print("------------- Start Record Track ---------------")

        # ======================================================== #
        # 订阅器
        self.subGPS = self.create_subscription(GpsInterface, 'gps_data', self.sub_callback_gps, 10)

    def sub_callback_gps(self, GpsInterface):
        x = GpsInterface.x  # x
        y = GpsInterface.y  # y
        h = GpsInterface.yaw  # 偏航角
        longitude = GpsInterface.longitude  # 经度
        latitude = GpsInterface.latitude  # 纬度

        if self.start_flag:
            self.last_x = x
            self.last_y = y
            self.start_flag = False

        dis = (self.last_x - x) * (self.last_x - x) + (self.last_y - y) * (self.last_y - y)
        if math.sqrt(dis) > 0.5:
            aaa = dict()
            aaa["x"] = x
            aaa["y"] = y
            aaa["h"] = h
            aaa["longitude"] = longitude
            aaa["latitude"] = latitude
            self.json_data[self.lane_name]["data"].append(aaa)
            self.last_x = x
            self.last_y = y
            print("已经保存的点数： ", len(self.json_data[self.lane_name]["data"]), end='\r')
            sys.stdout.flush()

    def write_json(self):
        print("------- Save Json --------")
        # 对字典中的键进行排序
        self.json_data = dict(sorted(self.json_data.items(), key=lambda x: x[0]))
        with open(self.json_path, 'w') as f:
            json.dump(self.json_data, f)
        print("-------- Save Json Done ---------")
        print("-------- 检查轨迹 ---------")
        print("已经保存的点数： ", len(self.json_data[self.lane_name]["data"]))
        data = self.json_data[self.lane_name]["data"]
        data_last = [data[0]["x"], data[0]["y"]]
        error_num = 0
        for index, aaa in enumerate(data):
            dis = math.sqrt(pow(data_last[0] - aaa["x"], 2) + pow(data_last[1] - aaa["y"], 2))
            if dis < 0.5 or dis > 0.6:
                print("error point!   index: ", index, "  dis: ", dis)
                error_num = error_num + 1
            data_last = [aaa["x"], aaa["y"]]
        print("error point num: ", error_num)
        print("-------- 检查轨迹 Done ---------")


def main():
    rclpy.init()
    rosNode = RecordTrack(name='record_track')
    try:
        rclpy.spin(rosNode)

    except KeyboardInterrupt:
        rosNode.write_json()

# if __name__ == '__main__':
#     rclpy.init()
#     rosNode = RecordTrack(name='record_track')
#     rclpy.spin(rosNode)
#     rclpy.shutdown()
