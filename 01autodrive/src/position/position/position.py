#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import pandas as pd
import time
import os
import math
from datetime import datetime

# 导入ROS2消息接口
from car_interfaces.msg import FusionInterface

class FusionDataLogger(Node):
    def __init__(self):
        super().__init__('fusion_data_logger')
        
        # 创建订阅者
        self.subFusion = self.create_subscription(
            FusionInterface, 
            "fusion_data", 
            self.sub_callback_fusion, 
            1
        )
        
        # Excel文件路径
        self.excel_file_path = "fusion_vehicle_data.xlsx"
        
        # 初始化Excel文件
        self.init_excel_file()
        
        # GPS状态判断
        self.GPS_state = False
        
        self.get_logger().info('融合数据记录节点已启动！')
        self.get_logger().info(f'数据将保存到: {os.path.abspath(self.excel_file_path)}')

    def init_excel_file(self):
        """初始化Excel文件，创建表头"""
        try:
            # 创建DataFrame with headers
            df = pd.DataFrame(columns=['时间', '经度', '纬度', '速度', '航向'])
            df.to_excel(self.excel_file_path, index=False)
            self.get_logger().info('Excel文件初始化完成')
        except Exception as e:
            self.get_logger().error(f'Excel文件初始化失败: {str(e)}')

    def sub_callback_fusion(self, msgFusion: FusionInterface):
        """订阅融合数据回调函数"""
        # 获取当前时间戳
        current_time = time.time()
        formatted_time = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # 提取数据
        nowSpeed = msgFusion.carspeed
        nowLatitude = msgFusion.latitude
        nowLongitude = msgFusion.longitude
        nowYaw = msgFusion.yaw  # 假设yaw已经是度数，如果是弧度需要转换
        
        # 打印调试信息
        print("融合数据接收:")
        print(f"速度: {nowSpeed}")
        print(f"经度: {nowLongitude}")
        print(f"纬度: {nowLatitude}")
        print(f"航向: {nowYaw}")
        
        # GPS状态判断（与您的代码逻辑一致）
        if abs(nowLatitude) < 1 and abs(nowLongitude) < 1:
            self.GPS_state = False
            self.get_logger().warn('GPS信号异常，数据可能不准确')
        else:
            self.GPS_state = True
        
        # 准备保存的数据行
        new_row = {
            '时间': formatted_time,
            '经度': nowLongitude,
            '纬度': nowLatitude,
            '速度': nowSpeed,
            '航向': nowYaw
        }
        
        # 保存数据到Excel
        self.save_data_to_excel(new_row)

    def save_data_to_excel(self, new_row):
        """保存数据到Excel文件"""
        try:
            # 读取现有数据
            if os.path.exists(self.excel_file_path):
                df = pd.read_excel(self.excel_file_path)
            else:
                df = pd.DataFrame(columns=['时间', '经度', '纬度', '速度', '航向'])
            
            # 添加新行
            df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)
            
            # 保存到Excel
            df.to_excel(self.excel_file_path, index=False)
            
            # 打印保存信息
            gps_status = "正常" if self.GPS_state else "异常"
            self.get_logger().info(f'数据已保存 [GPS:{gps_status}]: 时间={new_row["时间"]}, 经度={new_row["经度"]:.6f}, 纬度={new_row["纬度"]:.6f}, 速度={new_row["速度"]:.2f}, 航向={new_row["航向"]:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'保存数据到Excel失败: {str(e)}')

    def get_data_count(self):
        """获取已保存的数据条数"""
        try:
            if os.path.exists(self.excel_file_path):
                df = pd.read_excel(self.excel_file_path)
                return len(df)
            return 0
        except Exception as e:
            self.get_logger().error(f'读取数据条数失败: {str(e)}')
            return 0

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建数据记录节点
    fusion_logger = FusionDataLogger()
    
    try:
        # 运行节点
        rclpy.spin(fusion_logger)
    except KeyboardInterrupt:
        fusion_logger.get_logger().info('接收到退出信号...')
    finally:
        # 显示最终统计信息
        total_records = fusion_logger.get_data_count()
        fusion_logger.get_logger().info(f'程序结束，总共保存了 {total_records} 条记录到 {fusion_logger.excel_file_path}')
        
        # 清理资源
        fusion_logger.destroy_node()
        rclpy.shutdown()
        print('程序已安全退出')

if __name__ == '__main__':
    main()
