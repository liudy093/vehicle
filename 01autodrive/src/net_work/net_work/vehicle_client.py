import rclpy
import cv2
import json
import base64
import socket
import threading
import numpy as np
import os
import time
import open3d as o3d
import torch
from datetime import datetime
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2 as pc2

class VehicleClient(Node):
    def __init__(self):
        super().__init__('vehicle_client')
        # ================== 硬件参数初始化 ==================
        self.car_id = int(os.getenv('VEHICLE_ID', '0x01'), 16)
        self.server_ip = os.getenv('CLOUD_IP', '172.16.2.115')
        self.server_port = int(os.getenv('CLOUD_PORT', '5588'))
        self.video_path = os.getenv('VIDEO_DIR', os.path.expanduser('~/vehicle_data/videos'))
        
        # ================== CUDA加速配置 ==================
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
            self.get_logger().info(f"CUDA加速已启用，设备：{torch.cuda.get_device_name(0)}")
        else:
            self.get_logger().warning("CUDA不可用，将使用CPU模式")
            
        # ================== 可视化系统初始化 ==================
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window(width=1280, height=720)
        self.bridge = CvBridge()
        
        # ================== 网络连接管理 ==================
        self.sock = self.init_network()
        self.lock = threading.Lock()
        
        # ================== 视频管理系统 ==================
        self.writer = None
        self.current_video = None
        self.file_start = 0
        self.radar_frame = None
        
        # ================== 后台线程启动 ==================
        threading.Thread(target=self.video_loop, daemon=True).start()
        threading.Thread(target=self.cleanup_loop, daemon=True).start()
        
        # ================== 传感器订阅配置 ==================
        self.init_subscriptions()


    def init_network(self):
        """建立可靠TCP连接"""
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                sock.connect((self.server_ip, self.server_port))
                self.get_logger().info(f"已连接至服务器 {self.server_ip}:{self.server_port}")
                return sock
            except Exception as e:
                self.get_logger().error(f"连接失败: {str(e)}，正在重试...")
                time.sleep(2)

    def init_subscriptions(self):
        """Orin平台专用订阅配置"""
        topics = {
            '/forward/rslidar_points': (PointCloud2, self.process_lidar),
            '/obstacle_grid': (GridCells, self.process_obstacle),
            '/left_point_after_trans': (PointCloud2, self.process_lidar),
            '/left_obstacle_grid': (GridCells, self.process_obstacle),
            '/right_point_after_trans': (PointCloud2, self.process_lidar),
            '/right_obstacle_grid': (GridCells, self.process_obstacle),
            '/radar_vis': (GridCells, self.process_radar)
        }
        
        for topic, (msg_type, callback) in topics.items():
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: callback(msg, t),
                10
            )

    def process_radar(self, msg, topic_name=None):
        """毫米波雷达数据处理（车牌中央安装）"""
        try:
            obstacle_points = []
            for cell in msg.cells:
                # 坐标补偿（前移1.0m，下移0.25m）
                x = cell.x + 1.0
                y = cell.y
                z = cell.z - 0.25
                obstacle_points.append([x, y, z])
            
            self.update_visualization(
                np.array(obstacle_points),
                color=[1.0, 0.0, 0.0],
                point_size=5.0
            )
        except Exception as e:
            self.get_logger().error(f"雷达数据处理错误: {str(e)}")

    def process_obstacle(self, msg, topic_name=None):
        """通用障碍物处理"""
        try:
            grid_points = []
            for cell in msg.cells:
                grid_points.append([cell.x, cell.y, 0.5])
            
            self.update_visualization(
                np.array(grid_points),
                color=[1.0, 1.0, 0.0],
                point_size=3.0
            )
        except Exception as e:
            self.get_logger().error(f"障碍物处理错误: {str(e)}")

    def process_lidar(self, msg, topic_name):
        """多激光雷达统一处理（Orin优化版）"""
        try:
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(gen))[:, :3]
            
            # 基于话题名称的坐标补偿
            if topic_name == '/forward/rslidar_points':
                points += [1.2, -0.02, -0.3]  # 前向雷达补偿
                color = [0.2, 0.8, 1.0]
            elif 'left' in topic_name:
                points += [-0.8, 0.82, -0.25]  # 左侧后雷达
                color = [0.0, 1.0, 0.2]
            elif 'right' in topic_name:
                points += [-0.8, -0.81, -0.25]  # 右侧后雷达
                color = [1.0, 0.5, 0.0]
            
            # 点云预处理流水线
            points = self.filter_ground(points)
            points = self.remove_outliers(points)
            
            if points.shape[0] > 0:
                self.update_visualization(points, color)
                
        except Exception as e:
            self.get_logger().error(f"激光雷达处理错误: {str(e)}")

    def filter_ground(self, points, z_threshold=-0.5):
        """地面点云过滤（适配16cm离地间隙）"""
        return points[points[:,2] > z_threshold]

    def remove_outliers(self, points, radius=1.0, min_neighbors=8):
        """统计离群点去除"""
        if len(points) < 10:
            return points
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        cl, _ = pcd.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0
        )
        return np.asarray(cl.points)

    def update_visualization(self, points, color, point_size=1.0):
        """可视化系统更新接口"""
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.paint_uniform_color(color)
            self.visualizer.add_geometry(pcd)
        except Exception as e:
            self.get_logger().error(f"可视化更新失败: {str(e)}")

    def video_loop(self):
        """视频合成主循环"""
        while True:
            try:
                # 渲染3D场景
                self.visualizer.poll_events()
                self.visualizer.update_renderer()
                
                # 捕获点云画面
                frame = cv2.cvtColor(
                    np.asarray(self.visualizer.capture_screen_float_buffer()),
                    cv2.COLOR_RGB2BGR
                )
                
                # 合成雷达图像
                if hasattr(self, 'radar_frame') and self.radar_frame is not None:
                    if self.radar_frame.size != 0:
                        radar_img = cv2.resize(self.radar_frame, (320, 240))
                        frame[0:240, 0:320] = radar_img
                
                # 视频管理
                self.write_video(frame)
                self.send_frame(frame)
                
            except Exception as e:
                self.get_logger().error(f"视频循环错误: {str(e)}")
                time.sleep(1)

    def write_video(self, frame):
        """视频文件管理"""
        if self.writer is None or time.time() - self.file_start > 180:
            if self.writer is not None:
                self.writer.release()
            filename = f"kaiyi_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            self.current_video = os.path.join(self.video_path, filename)
            self.writer = cv2.VideoWriter(
                self.current_video,
                cv2.VideoWriter_fourcc(*'mp4v'),
                30,
                (1280, 720)
            )
            self.file_start = time.time()
        
        self.writer.write(frame)

    def send_frame(self, frame):
        """网络数据传输"""
        try:
            _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            data = {
                "car_id": hex(self.car_id),
                "timestamp": int(time.time()*1000),
                "frame": base64.b64encode(jpeg).decode()
            }
            self.sock.sendall((json.dumps(data)+'\n').encode())
        except Exception as e:
            self.get_logger().error(f"网络错误: {str(e)}，正在重连...")
            self.sock = self.init_network()

    def cleanup_loop(self):
        """自动清理系统"""
        while True:
            try:
                now = time.time()
                for f in os.listdir(self.video_path):
                    path = os.path.join(self.video_path, f)
                    if path != self.current_video and os.path.isfile(path):
                        if now - os.path.getmtime(path) > 3600:  # 保留1小时历史数据
                            os.remove(path)
            except Exception as e:
                self.get_logger().error(f"清理错误: {str(e)}")
            time.sleep(300)

def main():
    rclpy.init()
    client = VehicleClient()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        if client.writer is not None:
            client.writer.release()
        client.visualizer.destroy_window()
        client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

