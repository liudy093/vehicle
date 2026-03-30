#!/usr/bin/env python3

import rospy
import numpy as np # 用于角度转换
from geometry_msgs.msg import Pose2D # 导入 Pose2D 消息类型

def publish_2d_pose_2d_msg():
    rospy.init_node('given_2d_pose2d_publisher', anonymous=False)
    
    pub = rospy.Publisher('/gps', Pose2D, queue_size=10)
    
    # --- 从 ROS 参数服务器获取 2D 位姿参数 ---
    # '~' 表示私有参数，即只作用于当前节点
    pose_x = rospy.get_param('~pose_x', 0.0)      # 默认 X 坐标
    pose_y = rospy.get_param('~pose_y', 0.0)      # 默认 Y 坐标
    yaw_deg = rospy.get_param('~yaw_deg', 0.0)    # 默认偏航角 (度)
    publish_rate = rospy.get_param('~publish_rate', 10) # 默认发布频率 (Hz)
    
    # Pose2D 中的 theta 是弧度，所以需要将度转换为弧度
    pose_theta_rad = np.deg2rad(yaw_deg) 
    
    rate = rospy.Rate(publish_rate)
    
    rospy.loginfo("Starting 2D Pose (Pose2D msg) Publisher.")
    rospy.loginfo("  Configured Pose: x=%.2f, y=%.2f, theta_deg=%.1f (theta_rad=%.2f)", 
                  pose_x, pose_y, yaw_deg, pose_theta_rad)
    rospy.loginfo("  Publishing rate: %d Hz", publish_rate)

    while not rospy.is_shutdown():
        pose2d_msg = Pose2D()
        
        pose2d_msg.x = pose_x
        pose2d_msg.y = pose_y
        pose2d_msg.theta = pose_theta_rad 
        
        pub.publish(pose2d_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_2d_pose_2d_msg()
    except rospy.ROSInterruptException:
        pass