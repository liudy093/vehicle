"""
Author: chen fang
CreateTime: 2023/3/11 下午6:44
File: ARS_radar.py
Description: 解析德国大陆毫米波雷达
             代码中全部代码都在一个线程中运行，只需初始化类，不用调用。如果需要获得检测结果： out = DelphiRadar.obj()
             检测结果的格式为 [category, l, w, h, x, y, z, v, latv]
"""

import can
import math
import time
import threading
import numpy as np
from .mot_dis import MOT_dis


class ARSRadar(object):
    def __init__(self, config):
        self.max_dis = config.RADAR.MAX_RADAR_DIS  # 毫米波检测的最远距离，如果目标超过此距离就删掉
        # ======================================================== #
        # 输出的接口
        self.obj = []  # 保存检测结果.从类中获得检测结果就从这里拿  type:二维numpy
        self.radar_state = 0  # 毫米波雷达状态
        self.radar_error = 0  # 毫米波雷达错误码
        self.time = 0  # 进程处理时间
        self.rader_msg_recieve_flag = 1  # 监听器

        # ======================================================== #
        # 定义CAN接口
        self.radarOriBus = can.interface.Bus(channel='can0', bustype='socketcan')

        # ======================================================== #
        # 毫米波相关
        self.radar_can_data = np.zeros((100, 10))  # 保存radar检测结果
        self.flag = -1  # 标志位
        self.rader_obj_num = 0  # 毫米波检测到多少的物体
        self.jishu = 0  # 记录已经收到了几个目标
        self.start_time = 0

        # ======================================================== #
        # 滤波
        self.tracker = MOT_dis(min_hits=config.TRACK.MIN_HITS, max_age=config.TRACK.MAX_AGE,
                               max_dis=config.TRACK.MAX_MATE_DIS, ED_threshold=config.TRACK.ED_THRESHOLD)

        # ======================================================== #
        # 定义一个线程，读取can线数据
        get_radar_can_data = threading.Thread(target=self.run)
        get_radar_can_data.start()

    def run(self):
        while True:
            radarOriData = self.radarOriBus.recv()
            # ======================================================== #
            # 读取毫米波检测到的目标数量 0x060A
            if radarOriData.arbitration_id == 0x060A:
                self.rader_msg_recieve_flag = 0
                num = self.get_radar_obj_num(radarOriData.data)
                self.rader_obj_num = num
                self.flag = 1
                self.start_time = time.time()

                self.radar_can_data = np.zeros((100, 10)).copy()
                # self.flag = -1
                self.jishu = 0

            # ======================================================== #
            # 读取radar的数据位 0x060B
            if self.rader_obj_num > 0 and radarOriData.arbitration_id == 0x060B:
                obj_info = self.get_radar_obj_info(radarOriData.data)  # [category, l, w, h, x, y, z, v, latv]
                #print('obj_info::',obj_info)
                self.radar_can_data[self.jishu, 0:9] = obj_info.copy()
                self.radar_can_data[self.jishu, -1] = 1  # 最后一位为计数位
                self.jishu += 1

            # ======================================================== #
            # 如果 读全 self.rader_obj_num 个目标 and 读到标志位
            SUM = self.radar_can_data.sum(axis=0)
            # print('SUM',SUM[-1])
            # print('self.rader_obj_num',self.rader_obj_num)


# 012345679461     48

# 40

            if SUM[-1] == self.rader_obj_num and self.flag == 1:
                # ======================================================== #
                # 1.先把最后一列删除
                self.radar_can_data = np.delete(self.radar_can_data, -1, axis=1)
                
                #print("ph1 real ",self.radar_can_data )

                # ======================================================== #
                # 2.找到真实值并提取出来

                radar_can_data = []
                for i in range(self.rader_obj_num):
                    if pow(self.radar_can_data[i, 4], 2) + pow(self.radar_can_data[i, 5], 2) <= pow(self.max_dis, 2):
                        radar_can_data.append(self.radar_can_data[i, :])

                # print("ph2 real ",radar_can_data)

                # ======================================================== #
                # 3.按帧跟踪（滤波）
                
                track_result = self.tracker.run(self.radar_can_data, frame=0)

                # ======================================================== #
                # 4.处理跟踪结果
                trks = np.array(track_result, dtype="float32")  # 二维list --> 二维numpy
                self.obj = trks.copy()
                number_track = len(trks)
                self.time = time.time() - self.start_time

                # ======================================================== #
                # 打印毫米波检测结果
                print("#########################################")
                print("number_track: ", number_track)
                if number_track != 0:
                    ttt_ = trks.copy()
                    ttt_ = ttt_[np.argsort(ttt_[:, 5])]
                    for t_ in ttt_:
                        # print("x=%.2f, y=%.2f, v=%.2f, lc=%.2f" % (t_[4], t_[5], t_[7], t_[8]))
                        pass

                # ======================================================== #
                # 5.清空数组和标志位
                self.radar_can_data = np.zeros((100, 10)).copy()
                self.jishu = 0
                self.flag = -1

    def get_radar_obj_num(self, data):
        """
        通过Cluster列表状态(0x60A)解码obj的数量
        Args:
            data:

        Returns:近处的目标数量和远处的目标数量

        """
        num = int(data[0])  # 近距离的目标数量
        if num > 100:
            num = 100
        return num

    def get_radar_obj_info(self, data):
        """
        通过Cluster一般信息(0x60B)解码obj的信息
        Args:
            data:

        Returns:近处的目标数量和远处的目标数量

        """
        # Target_ID
        # 目标物体的ID
        # 0-255
        
        
        '''
        print('data[0]',data[0])
        print('data[1]',data[1])
        print('data[2]',data[2])
        '''

        
        cluster_id = int(data[0])

        # Target_DistLong
        # x轴坐标 （前向为x）
        # -500 - +1138.2  分辨率0.2  无正负，是一个线性关系
        x = ((data[1] << 5 )& 0x1FE0 ) |   ((data[2] >> 3) & 0x001F)  # 取出值    0001 1111
        x = x * 0.2 - 500  

        #  0001 1111 1110 0000  
        #  0000 0000 0001 1111 
            
 
        # Target_DistLat
        # y轴坐标 （左向为x）
        # -204.6 ~ +204.8  分辨率0.2 无正负，是一个线性关系
        y = ((data[2] << 8) & 0x0700) | (data[3] & 0x00FF)  # 取出值
        y = y * 0.2 - 204.6
# 0000 0111 0000 0000
# 0000 0000 1111 1111

        # Target_VrelLong
        # 纵向速度
        # -128 ~ +127.75  分辨率0.25 无正负，是一个线性关系
        v = ((data[4] << 2) & 0x03FC) | ((data[5] >> 6) & 0x0003)  # 取出值
        v = v * 0.25 - 128
        
# 0000 0011 1111  1100    
# 0000 0000 0000  0011

        # Target_VrelLat
        # 横向速度
        # -64 ~ +63.75 分辨率0.25
        latv = ((data[5] << 3) & 0x01F8) | ((data[6] >> 5) & 0x0007)  # 取出值
        latv = latv * 0.25 - 64
# 0000 0001 1111 1000
# 0000 0000 0000 0111
        

        z = 1.0
        category = 0.0
        l = 1.0
        w = 1.0
        h = 1.0

        
        # y=1

        '''
        print('x',x)      
        print('y',y)
        print('vx',v)
        print('latv',latv)
        '''


        return [category, l, w, h, x, y, z, v, latv]


if __name__ == '__main__':
    pass
