"""
Author: chen fang
CreateTime: 2023/5/23 下午1:10
File: lidar_obstacle.py
Description: 
"""

import sys
import os
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "right_lidar_obstacle"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

ros_path = '/opt/ros/foxy/lib/python3.8/site-packages'
msg_path = os.getcwd() + '/install/car_interfaces/lib/python3.8/site-packages'
obstacle_path = os.getcwd() + '/src/right_lidar_obstacle/right_lidar_obstacle'

sys.path.append(ros_path)
sys.path.append(msg_path)
sys.path.append(obstacle_path)
sys.path.append("..")

import time
import yaml
from easydict import EasyDict
import numpy as np
import array
import pcl
import ros2_numpy as rnp
from points_processing_cpp import points_processing_cpp

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from std_msgs.msg import String,Bool

class LidarObstacle(Node):
    def __init__(self, name):
        super().__init__(name)
        # ======================================================== #
        # 从config文件中读取超参数
        root_path = os.getcwd() + '/src/right_lidar_obstacle/right_lidar_obstacle/'  # 根目录
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f, Loader=yaml.FullLoader)
        self.config = EasyDict(config)

        # ======================================================== #
        # 初始化点云处理类
        self.func = points_processing_cpp(self.config)

        # ======================================================== #
        # 订阅器
        self.subLidarOri_rslidar = self.create_subscription(PointCloud2, '/right/rslidar_points',
                                                            self.sub_callback_lidar_rslidar, 10)
        
        
                # 监听器
        # ======================================================== #
        self.PubRightLidarState = self.create_publisher(Bool, 'RightLidarState', 10)
        self.right_msg_recieve_flag = False  # 默认状态错误，需要在代码中不断的改为正确
        self.timerRidarOri = self.create_timer(1, self.RightLidarState_monitor)
        # ======================================================== #



        # ======================================================== #
        # 发布器
        # 障碍物数据
        self.pubLidarObstacle = self.create_publisher(LidarObstacleInterface, 'right_lidar_obstacle_data', 10)
        # grid可视化
        if self.config.vis_publisher.obstacle_grid:
            self.obstacle_grid = self.create_publisher(GridCells, 'right_obstacle_grid', 10)
        # 可视化旋转之后的全部点云
        if self.config.vis_publisher.all_points:
            self.pubLidar_all_points = self.create_publisher(PointCloud2, 'right_point_after_trans', 10)
            
    
    def RightLidarState_monitor(self):
        """
        毫米波状态监听器
        """
        msg = Bool()
        if self.right_msg_recieve_flag  == True:
            msg.data = True
            self.PubRightLidarState.publish(msg)
            self.right_msg_recieve_flag = False
        else:
            print('--------------------leftLidarState_error-----------------------')
            self.right_msg_recieve_flag = False
            msg.data = False
            self.PubRightLidarState.publish(msg)        
    

    # 点云的回调函数
    def sub_callback_lidar_rslidar(self, LidarOriInterface):
        all_start = time.time()
        print("------------- right lidar -----------------")
        ##############################################################
        # 点云格式转换并裁剪
        start = time.time()
        point_cloud_pcl, point_numpy = self.ros_to_numpy_to_pcl(LidarOriInterface)
        end = time.time()
        print("ros_to_numpy_to_pcl time:", end - start)

        ##############################################################
        # 点云处理
        start = time.time()
        obstacle_list = self.func.run(point_cloud_pcl)
        end = time.time()
        print("points_processing time:", end - start)
        # 对障碍物和点云进行平移
        cal_left = np.array(self.config.trans)
        t_left = cal_left[0:3, 3]
        t_left = t_left.reshape(3, 1)
        # 障碍物平移操作
        obstacles_xyz = obstacle_list[:, :3]
        obstacles_xyz_final = np.transpose(obstacles_xyz.T + t_left)
        obstacle_list[:, :3] = obstacles_xyz_final
        # 点云进行平移操作
        point_cloud_xyz = point_numpy[:, :3]
        point_cloud_xyz_new = np.transpose(point_cloud_xyz.T + t_left)
        point_numpy[:, :3] = point_cloud_xyz_new

        ##############################################################
        # 障碍物数据装载及发布
        start = time.time()
        self.publish_lidar_obstacle(obstacle_list)
        end = time.time()
        print("publish_lidar_obstacle time:", end - start)

        ##############################################################
        # 可视化相关
        # grid可视化
        if self.config.vis_publisher.obstacle_grid:
            start = time.time()
            self.publish_lidar_obstacle_grid(obstacle_list)
            end = time.time()
            print("publish_lidar_obstacle_grid time:", end - start)
        # 可视化旋转之后的全部点云
        if self.config.vis_publisher.all_points:
            start = time.time()
            self.publish_lidar_point_cloud(point_numpy, self.pubLidar_all_points)
            end = time.time()
            print("publish_trans_point_cloud time:", end - start)

        all_end = time.time()
        print("all time: ", all_end - all_start, "hz:", 1 / (all_end - all_start))
        print("################################################")

    def ros_to_numpy_to_pcl(self, ros_cloud):
        # ros_points -> numpy
        data = rnp.numpify(ros_cloud)
        data = rnp.point_cloud2.get_xyzi_points(data)
        data = data.astype(np.float32)

        print('point_cloud num: ', data.shape[0])

        # TODO: 坐标转换(旋转)
        cal_left = np.array(self.config.trans)
        xyz = data[:, 0:3]
        i = data[:, 3].reshape(-1, 1)
        xyz = np.dot(xyz, cal_left[:, :3])
        data = np.concatenate([xyz, i], axis=1)


        # TODO:额外的点云裁剪
        mask_ = (data[:, 0] >= self.config.delete.back) & (data[:, 0] <= self.config.delete.forward) \
                & (data[:, 1] >= self.config.delete.right) & (data[:, 1] <= self.config.delete.left)
        data = data[~mask_]

        # TODO:点云裁剪
        # 用numpy裁剪比pcl快
        start = time.time()
        mask = (data[:, 0] >= self.config.obstacle.range[0]) & (data[:, 0] <= self.config.obstacle.range[3]) \
               & (data[:, 1] >= self.config.obstacle.range[1]) & (data[:, 1] <= self.config.obstacle.range[4]) \
               & (data[:, 2] >= self.config.obstacle.range[2]) & (data[:, 2] <= self.config.obstacle.range[5])
        data = data[mask]
        end = time.time()
        print("point_cloud_crop time:", end - start)

        # numpy -> pcl
        pcl_data = pcl.PointCloud_PointXYZI()
        data = data.astype(np.float32)
        pcl_data.from_array(data)

        return pcl_data, data

    def publish_lidar_obstacle(self, obstacle_np):
        msgLidarObstacle = LidarObstacleInterface()
        msgLidarObstacle.timestamp = time.time()
        msgLidarObstacle.id = 0
        msgLidarObstacle.number = len(obstacle_np)
        msgLidarObstacle.process_time = 0.0

        obstacle_msg_np = np.zeros((len(obstacle_np), 9))
        obstacle_msg_np[:, 1] = 0.5
        obstacle_msg_np[:, 2] = 0.5
        obstacle_msg_np[:, 3] = obstacle_np[:, 2]
        obstacle_msg_np[:, 4] = -obstacle_np[:, 1]
        obstacle_msg_np[:, 5] = obstacle_np[:, 0]
        obstacle_msg_np[:, 6] = obstacle_np[:, 2]
        obstacle_msg_np = obstacle_msg_np.flatten().astype(np.float32)
        msgLidarObstacle.obstacledata = array.array('f', obstacle_msg_np)

        self.pubLidarObstacle.publish(msgLidarObstacle)
        # state monitor
        # ======================================================== #
        self.right_msg_recieve_flag = True
        msg = Bool()
        msg.data = True
        self.PubRightLidarState.publish(msg)
        # ======================================================== #
        pass

    def publish_lidar_point_cloud(self, point_cloud, publisher):
        msg = PointCloud2()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "rslidar"
        if len(point_cloud.shape) == 3:
            msg.height = point_cloud.shape[1]
            msg.width = point_cloud.shape[0]
        else:
            msg.height = 1
            msg.width = len(point_cloud)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * point_cloud.shape[0]
        msg.is_dense = False
        msg.data = np.asarray(point_cloud, np.float32).tostring()

        publisher.publish(msg)

    def publish_lidar_obstacle_grid(self, obstacle_list):
        cells = GridCells()
        cells.header.frame_id = "rslidar"
        cells.cell_width = 0.5  # edit for grid size .3 for simple map
        cells.cell_height = 0.5  # edit for grid size
        for i in range(len(obstacle_list)):
            point = Point()
            point.x = float(obstacle_list[i, 0])
            point.y = float(obstacle_list[i, 1])
            point.z = float(obstacle_list[i, 2])
            cells.cells.append(point)
        self.obstacle_grid.publish(cells)

    def plannerVis_callback(self, msg):
        """
        存入可视化信息
        """
        self.plannerVis = msg


def main():
    rclpy.init()
    rosNode = LidarObstacle(name='right_lidar_obstacle')
    rclpy.spin(rosNode)
    rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    rosNode = LidarObstacle(name='right_lidar_obstacle')
    rclpy.spin(rosNode)
    rclpy.shutdown()
