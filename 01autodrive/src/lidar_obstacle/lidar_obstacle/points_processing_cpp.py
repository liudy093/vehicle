"""
Author: chen fang
CreateTime: 2023/5/23 下午3:35
File: points_processing_cpp.py
Description: 使用c++处理点云
"""

import time
import pcl
import pybind11
import numpy as np
import math
import os
from point_processing_cpp import point_processing


class points_processing_cpp:
    def __init__(self, config):
        self.config = config

    def run(self, point_cloud_pcl):
        ##############################################################
        # 体素化滤波
        start = time.time()
        filter_vox = point_cloud_pcl.make_voxel_grid_filter()
        filter_vox.set_leaf_size(0.1, 0.1, 0.1)
        cloud_filtered = filter_vox.filter()
        end = time.time()
        print("voxel filter time:", end - start)

        ##############################################################
        # 使用c++处理点云
        point_cloud_numpy = self.points_information(cloud_filtered).astype(np.float32)  # [N,7]

        # 参数装载
        paremeters = np.zeros(21).astype(np.float32)
        paremeters[0] = point_cloud_numpy.shape[0]  # 点云数量
        paremeters[1] = self.config.obstacle.lidar_height  # 传感器安装高度
        paremeters[2] = self.config.obstacle.h_range  # 高度范围
        paremeters[3] = self.config.obstacle.h_extr  # 体素中高度的极差
        paremeters[20] = self.config.obstacle.h_extr_max  # 体素极差如果大于这个值，则一定是障碍物，不经过第二次判断
        paremeters[4] = self.config.obstacle.h_mean_extr  # 与周围8个地面体素高度均值的差
        paremeters[11] = self.config.obstacle.min_points_num_10m  # 一个体素中最少的点云数量
        paremeters[12] = self.config.obstacle.min_points_num_20m  # 一个体素中最少的点云数量
        paremeters[13] = self.config.obstacle.min_points_num_30m  # 一个体素中最少的点云数量
        paremeters[14] = self.config.obstacle.min_points_num_100m  # 一个体素中最少的点云数量

        paremeters[15] = self.config.obstacle.min_up_mean_points_num_10m  # 一个体素中超过平均高度的点的数量
        paremeters[16] = self.config.obstacle.min_up_mean_points_num_20m  # 一个体素中超过平均高度的点的数量
        paremeters[17] = self.config.obstacle.min_up_mean_points_num_30m  # 一个体素中超过平均高度的点的数量
        paremeters[18] = self.config.obstacle.min_up_mean_points_num_100m  # 一个体素中超过平均高度的点的数量
        paremeters[19] = self.config.obstacle.up_mean_height

        paremeters[5] = self.config.mot.is_use  # 是否使用mot
        paremeters[6] = self.config.mot.min_hits  # min_hits
        paremeters[7] = self.config.mot.max_age  # max_age
        paremeters[8] = self.config.mot.max_match_dis  # max_match_dis

        paremeters[9] = self.config.nms.is_use  # 是否使用nms
        paremeters[10] = self.config.nms.threshold  # threshold

        start = time.time()
        obj_list = point_processing.ground_segmentation(point_cloud_numpy, paremeters)
        end = time.time()
        print("point_processing_cpp time:", end - start)
        obj_list = np.array(obj_list).reshape(-1, 3)

        return obj_list

    def points_information(self, cloud_filtered):
        start = time.time()
        # pcl -> numpy
        point_cloud_numpy = cloud_filtered.to_array().transpose(0, 1)
        # 体素化点云
        point_voxel_index = np.zeros((point_cloud_numpy.shape[0], 3))  # x,y,z
        point_voxel_index = point_cloud_numpy[:, 0:3] - self.config.obstacle.range[0:3]
        point_voxel_index = point_voxel_index // self.config.obstacle.voxel_size

        point_cloud_numpy_all_data = np.concatenate([point_cloud_numpy, point_voxel_index], axis=1)
        end = time.time()
        print("points_information time:", end - start)
        return point_cloud_numpy_all_data


if __name__ == '__main__':
    pass
