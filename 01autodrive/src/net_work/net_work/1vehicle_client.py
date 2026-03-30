import rclpy  
import json  
import time  
import socket  
import threading  
import numpy as np  
import open3d as o3d  
import cv2  
import base64  
from rclpy.node import Node  
from nav_msgs.msg import GridCells  
from cv_bridge import CvBridge  

class VehicleClient(Node):  
    def __init__(self, car_id):  
        super().__init__(f'vehicle_client_{car_id}')  
        self.car_id = car_id  
        self.server_ip = '172.16.2.115'  
        self.server_port = 5588  
        
        # 线程安全配置（未修改）  
        self.lock = threading.Lock()  
        self.connection_lock = threading.Lock()  
        self.sock = None  
        self.connected = False  
        self.running = True  

        # 坐标系转换参数（未修改）  
        self.coord_transform = {  
            'front_lidar_point': (0.0, 0.0, 0.0),  
            'front_lidar_obstacle': (0.0, 0.0, 0.0),  
            'left_lidar_point': (-1.5, -2.8, 0.1),  
            'left_lidar_obstacle': (-1.5, -2.8, 0.1),  
            'right_lidar_point': (1.5, -2.8, 0.1),  
            'right_lidar': (1.5, -2.8, 0.1),  
            'radar': (0.0, 0.0, 0.3)  
        }  

        # 话题订阅配置（未修改）  
        self.topic_config = {  
            'front_lidar_point': '/forward/rslidar_points',  
            'front_lidar_obstacle': '/obstacle_grid',  
            'left_lidar_point': '/left_point_after_trans',  
            'left_lidar_obstacle': '/left_obstacle_grid',  
            'right_lidar_point': '/right_point_after_trans',  
            'right_lidar': '/right_obstacle_grid',  
            'radar': '/radar_vis'  
        }  

        # 点云处理参数（未修改）  
        self.voxel_size = 0.2  
        self.max_range = 30.0  
        
        # 初始化网络连接（未修改）  
        self.connect_server()  
        
        # 数据存储结构优化（新增强度存储）▼▼▼  
        self.sensor_data = {  
            k: {'points': None, 'intensity': None}  
            for k in self.topic_config.keys()  
        }  
        # ▲▲▲  
        self.bridge = CvBridge()  
        
        # ROS 2订阅初始化（未修改）  
        self.subscribers = {}  
        self.init_subscribers()  
        
        # 启动发送线程（未修改）  
        self.send_thread = threading.Thread(target=self.data_sender)  
        self.send_thread.daemon = True  
        self.send_thread.start()  

    def connect_server(self):  
        """稳健的连接管理方法（未修改）"""  
        with self.connection_lock:  
            if self.connected:  
                return  

            while not self.connected and self.running:  
                try:  
                    if self.sock:  
                        self.sock.close()  
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
                    self.sock.settimeout(10)  
                    self.sock.connect((self.server_ip, self.server_port))  
                    self.connected = True  
                    self.get_logger().info(f"成功连接到服务器 {self.server_ip}:{self.server_port}")  
                except socket.error as e:  
                    self.connected = False  
                    self.get_logger().error(f"连接失败: {str(e)}，5秒后重试...")  
                    time.sleep(5)  
                except Exception as e:  
                    self.connected = False  
                    self.get_logger().error(f"意外连接错误: {str(e)}")  
                    time.sleep(3)  

    def init_subscribers(self):  
        """初始化ROS 2订阅（未修改）"""  
        for sensor_type, topic in self.topic_config.items():  
            self.subscribers[sensor_type] = self.create_subscription(  
                GridCells,  
                topic,  
                self.create_callback(sensor_type),  
                qos_profile=10  
            )  

    def create_callback(self, sensor_type):  
        """修正强度值处理回调▼▼▼"""  
        def callback(msg):  
            try:  
                with self.lock:  
                    points = []  
                    intensities = []  
                    dx, dy, dz = self.coord_transform[sensor_type]  
                    
                    for cell in msg.cells:  
                        # 坐标转换（根据配置的XYZ转换）  
                        x = cell.x * 0.1 + dx  
                        y = cell.y * 0.1 + dy  
                        z = cell.z * 0.1 + dz  
                        
                        # 提取强度值（根据RViz配置的intensity字段）  
                        intensity = cell.z  
                        
                        points.append([x, y, z])  
                        intensities.append(float(intensity))  
                    
                    # 存储原始数据  
                    self.sensor_data[sensor_type]['points'] = np.array(points)  
                    self.sensor_data[sensor_type]['intensity'] = np.array(intensities)  
            except Exception as e:  
                self.get_logger().error(f"回调处理错误: {str(e)}")  
        return callback  
        # ▲▲▲  

    def generate_3d_voxel(self):  
        """生成带彩虹色映射的体素图▼▼▼"""  
        try:  
            with self.lock:  
                all_points = []  
                all_intensities = []  
                
                # 合并数据（保持各传感器独立）  
                for sensor in self.topic_config.keys():  
                    if self.sensor_data[sensor]['points'] is not None:  
                        all_points.extend(self.sensor_data[sensor]['points'])  
                        all_intensities.extend(self.sensor_data[sensor]['intensity'])  
                
                if len(all_points) < 10:  
                    return None  
                
                # 创建点云  
                pcd = o3d.geometry.PointCloud()  
                pcd.points = o3d.utility.Vector3dVector(np.array(all_points))  
                
                # 强度值映射（与RViz rainbow配置一致）▼▼▼  
                intensities = np.array(all_intensities)  
                if intensities.size == 0:  
                    return None  
                
                # 归一化处理  
                eps = 1e-6  # 防止除零  
                norm_intensity = (intensities - intensities.min()) / \
                    (intensities.max() - intensities.min() + eps)  
                
                # HSV颜色空间（Hue值对应强度）  
                hsv = np.zeros((len(norm_intensity), 3))  
                hsv[:, 0] = 0.8 * (1 - norm_intensity)  # 0.8对应蓝→紫→红渐变  
                hsv[:, 1] = 1.0  # 全饱和度  
                hsv[:, 2] = 1.0  # 全亮度  
                
                # 转换为RGB  
                rgb = cv2.cvtColor(hsv.reshape(1, -1, 3).astype(np.float32),   
                                 cv2.COLOR_HSV2RGB).reshape(-1, 3)  
                
                pcd.colors = o3d.utility.Vector3dVector(rgb)  
                # ▲▲▲  
                
                # 体素下采样（未修改）  
                voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(  
                    pcd, voxel_size=self.voxel_size)  
                
                # 可视化配置（优化视点）▼▼▼  
                vis = o3d.visualization.Visualizer()  
                vis.create_window(width=600, height=600, visible=False)  
                vis.add_geometry(voxel_grid)  
                ctr = vis.get_view_control()  
                ctr.set_zoom(0.25)  
                ctr.set_front([0, -1, 0.5])  # 前视角  
                ctr.set_up([0, -0.4, -0.9])  # 优化视角角度  
                # ▲▲▲  
                
                # 图像采集（未修改）  
                vis.poll_events()  
                vis.update_renderer()  
                img = vis.capture_screen_float_buffer(True)  
                vis.destroy_window()  
                
                # 颜色空间修正（未修改）  
                img_array = (np.asarray(img) * 255).astype(np.uint8)  
                return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)  
                
        except Exception as e:  
            self.get_logger().error(f"体素生成错误: {str(e)}")  
            return None  

    def data_sender(self):  
        """数据发送线程（未修改）"""  
        while self.running:  
            try:  
                if not self.connected or self.sock is None:  
                    self.connect_server()  
                    time.sleep(1)  
                    continue  
                
                voxel_img = self.generate_3d_voxel()  
                if voxel_img is None:  
                    time.sleep(0.1)  
                    continue  
                
                _, jpeg = cv2.imencode('.jpg', voxel_img,   
                                    [int(cv2.IMWRITE_JPEG_QUALITY), 80])  
                if jpeg is None:  
                    self.get_logger().warning("JPEG编码失败")  
                    continue  
                
                packet = {  
                    "type": "radar_3d",  
                    "car_id": self.car_id,  
                    "timestamp": time.time(),  
                    "data": base64.b64encode(jpeg).decode('utf-8')  
                }  
                json_str = json.dumps(packet) + '\n'  
                
                for attempt in range(3):  
                    try:  
                        self.sock.sendall(json_str.encode('utf-8'))  
                        break  
                    except (BrokenPipeError, ConnectionResetError) as e:  
                        self.get_logger().warning(f"发送失败（尝试 {attempt+1}/3）: {str(e)}")  
                        self.connected = False  
                        self.connect_server()  
                    except Exception as e:  
                        self.get_logger().error(f"发送错误: {str(e)}")  
                        time.sleep(0.5)  
                else:  
                    self.get_logger().error("连续发送失败，暂停5秒")  
                    time.sleep(5)  
                
                time.sleep(0.05)  
                
            except Exception as e:  
                self.get_logger().error(f"发送循环错误: {str(e)}")  
                time.sleep(1)  

    def shutdown(self):  
        """关闭方法（未修改）"""  
        self.running = False  
        if self.sock:  
            try:  
                self.sock.shutdown(socket.SHUT_RDWR)  
                self.sock.close()  
            except:  
                pass  
        self.connected = False  
        self.get_logger().info("连接已安全关闭")  

if __name__ == '__main__':  
    rclpy.init()  
    client = VehicleClient(car_id=0x03)  
    
    try:  
        rclpy.spin(client)  
    except KeyboardInterrupt:  
        client.get_logger().info("接收到键盘中断信号")  
    except Exception as e:  
        client.get_logger().error(f"主循环错误: {str(e)}")  
    finally:  
        client.shutdown()  
        client.destroy_node()  
        rclpy.shutdown()  