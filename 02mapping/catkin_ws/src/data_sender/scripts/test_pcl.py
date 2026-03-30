#!/usr/bin/env python3

import rospy
import random
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import struct

def generate_fan_pointcloud():
    rospy.init_node('fan_pointcloud_generator', anonymous=True)
    
    # --- 从 ROS 参数服务器获取扇形点云参数 ---
    # rospy.get_param('~param_name', default_value)
    # '~' 表示私有参数，即只作用于当前节点
    num_points = rospy.get_param('num_points', 1000)       # 点云中点的数量
    min_radius = rospy.get_param('min_radius', 0.5)        # 扇形内半径 (米)
    max_radius = rospy.get_param('max_radius', 5.0)        # 扇形外半径 (米)
    start_angle_deg = rospy.get_param('start_angle_deg', -45.0) # 扇形起始角度 (度)
    end_angle_deg = rospy.get_param('end_angle_deg', 45.0)    # 扇形结束角度 (度)
    min_height = rospy.get_param('min_height', -0.1)       # 点云最低高度 (米)
    max_height = rospy.get_param('max_height', 0.1)        # 点云最高高度 (米)
    publish_rate = rospy.get_param('publish_rate', 10)     # 发布频率 (Hz)
    frame_id = rospy.get_param('frame_id', 'map')  # 点云的坐标系框架 ID  
    pub_name = rospy.get_param('pub_name', 'fan_pointcloud')  # 发布的点云主题名称  
    # 将角度转换为弧度
    start_angle_rad = np.deg2rad(start_angle_deg)
    end_angle_rad = np.deg2rad(end_angle_deg)
    
    rate = rospy.Rate(publish_rate)

    pub = rospy.Publisher(pub_name, PointCloud2, queue_size=10)
    
    rospy.loginfo("Starting fan point cloud generator with parameters:")
    rospy.loginfo("  num_points: %d", num_points)
    rospy.loginfo("  min_radius: %.2f m", min_radius)
    rospy.loginfo("  max_radius: %.2f m", max_radius)
    rospy.loginfo("  start_angle: %.1f deg", start_angle_deg)
    rospy.loginfo("  end_angle: %.1f deg", end_angle_deg)
    rospy.loginfo("  min_height: %.2f m", min_height)
    rospy.loginfo("  max_height: %.2f m", max_height)
    rospy.loginfo("  publish_rate: %d Hz", publish_rate)

    while not rospy.is_shutdown():
        points_data = []
        for _ in range(num_points):
            r = random.uniform(min_radius, max_radius)
            theta = random.uniform(start_angle_rad, end_angle_rad)
            z = random.uniform(min_height, max_height)
            
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            points_data.append([x, y, z])
            
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = frame_id # 保持不变，可以根据需要更改

        cloud_msg.height = 1
        cloud_msg.width = num_points
        
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        buffer = bytearray(cloud_msg.row_step)
        for i, point in enumerate(points_data):
            struct.pack_into('<fff', buffer, i * cloud_msg.point_step, point[0], point[1], point[2])
        
        cloud_msg.data = buffer
        
        pub.publish(cloud_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_fan_pointcloud()
    except rospy.ROSInterruptException:
        pass