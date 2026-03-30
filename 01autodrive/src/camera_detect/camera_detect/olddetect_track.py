import os
import sys
import time
import json
import torch

ros_node_name = "camera_detect"
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "yolov8_sort"))
sys.path.append(os.path.join(os.getcwd(), "src/%s/%s/"%(ros_node_name, ros_node_name), "MvImport"))

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from car_interfaces.msg import CameraStateInterface, CameraObstacleInterface
# from MvImport.HKCameraAPI import HIKCamera
from yolov8_sort.yolo import YOLO

class CameraOri(Node):
    def __init__(self, name):
        super().__init__(name)
        #------------输出日志信息, 话题发布----------------------#
        self.get_logger().info("Node: camera_detect  publisher: camera_state_data, camera_ori, camera_detect")
        #------------------------------------------------------#

        #--------------- 从json文件读入配置 --------------------#
        with open(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name)+"/config.json", 'r') as jsonfile:
            self.config = json.load(jsonfile)
        #------------------------------------------------------#

        #--------------------配置选项---------------------------#
        # 读入的图片的的大小, 
        self.read_image_size = tuple(self.config["orin_image"]["image_size"])
        #------------------------------------------------------#

        #-------------------是否保存视频-------------------------#
        if self.config["orin_image"]["is_out_video"]:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            # 依据时间戳得到视频名称，防止顶掉原始视频
            local_time = time.localtime()
            out_video_name = f"{local_time.tm_yday:4d}-{local_time.tm_hour:2d}-{local_time.tm_min:2d}-{local_time.tm_sec:2d}.mp4"
            self.out_video = cv2.VideoWriter(self.config["orin_image"]["out_video_path"]+out_video_name, 
                fourcc, self.config["orin_image"]["image_fps"], tuple(self.config["orin_image"]["image_size"]))
        #------------------------------------------------------#

        #------------load video, 产生相机对象 self.cap----------#
        try:
            self.load_video_camera(self.config["orin_image"]["flag"])
        except:
            print("----初始化相机失败----")
        #------------------------------------------------------#

        #----------------------订阅task_id-----------------------------#
        #self.plannerVis_subscriber = self.create_subscription(PlannerVis, "planner_vis_data", self.plannerVis_callback, 10)
        #------------------------------------------------------#

        #---------------定义相机状态的发布者和定时器--------------#
        self.pubCameraState = self.create_publisher(CameraStateInterface, 'camera_state_data', 10)
        self.pubCameraObstacle = self.create_publisher(CameraObstacleInterface, "camera_detect_data", 10)
        self.timerCameraData = self.create_timer(0.05, self.pub_callback_camera_data)
        #------------------------------------------------------#

        #-----------------读入图像的定时器-----------------------#
        self.timerReadCamera = self.create_timer(0.05, self.callback_read_camera)
        #-------------------------------------------------------#

        #---------------定义相机图像的发布者和定时器--------------#
        # 可视化原始图像用
        self.pubCameraOriRviz = self.create_publisher(Image, 'camera_ori_img', 10)
        # 可视化算法结果用
        if self.config["detect_config"]["enable_vis"]:
            self.pubCameraDetectRviz = self.create_publisher(Image, 'camera_detect_img', 10)
        self.timerCameraOri = self.create_timer(0.01, self.callback_detect_image)
        
        # 可视化原始图像用
        self.pubCameraMarker = self.create_publisher(GridCells, 'camera_obstacle_marker', 10)
        #------------------------------------------------------#

        #------------------------从json文件得到内参和外参----------------------------#
        self.internal_param = np.array(self.config["camera_param"]["intrinsic_matrix"])
        self.distortion_cofficients = np.array(self.config["camera_param"]["distortion_cofficients"])

        self.external_param = np.array(self.config["camera_param"]["forward_lidar_to_camera"])
        self.external_param = np.hstack((self.external_param[:, :-1].transpose(), self.external_param[:, -1].reshape(-1,1)))
        self.external_param = np.vstack((self.external_param, np.array([0., 0., 0., 1.])))
        self.config["detect_config"]["internal_param"] = self.internal_param
        self.config["detect_config"]["external_param"] = self.external_param
        self.config["detect_config"]["camera_height"] = self.config["camera_param"]["camera_height"]
        self.config["detect_config"]["camera_angle"] = self.config["camera_param"]["camera_angle"]
        #----------------------------------------------------#

        #--------------初始化网络模型---------------------------#
        self.yolov8_sort = YOLO(self.config["detect_config"])
        #------------------------------------------------------#

        #------------------存放中间结果-------------------------#
        # 存放可视化信息
        #self.plannerVis = None
        # 初始相机状态
        self.ret = False
        # 原始图像
        self.frame = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
        # 图像检测结果
        self.image_detect = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
        self.process_time = -1.0
        self.data_detect = []
        self.out = cv2.VideoWriter("output.mp4", cv2.VideoWriter_fourcc(*'MP4V'),20.0,(640,640))
        #------------------------------------------------------#


    def update_camera_state(self):
        """
        给msg添加数据
        :parmas msg: 是实例化的话题
        :ret 是相机状态, 为 True 则显示相机正常在线
        """
        msg = CameraStateInterface()
        msg.timestamp = time.time()
        msg.id = 0x01
        if self.ret:
            msg.state = 0
            msg.error = 0
        else:
            msg.state = 1
            msg.error = 1

        self.pubCameraState.publish(msg)

    def update_camera_obstalce(self):
        cells = GridCells()
        cells.header.frame_id = "rslidar"
        cells.cell_width = 0.5
        cells.cell_height = 0.5

        camera_obstacle = CameraObstacleInterface()
        camera_obstacle.number = len(self.data_detect)

        for i in range(len(self.data_detect)):
            point = Point()
            point.x = float(self.data_detect[i, -3])
            point.y = float(self.data_detect[i, -2])
            point.z = 1.5
            cells.cells.append(point)
            camera_obstacle.process_time = time.time()
            camera_obstacle.obstacledata.extend([float(self.data_detect[i, -3]), 
                                float(self.data_detect[i, -2]), float(self.data_detect[i, -1]),
                                int(self.data_detect[i, -5]) if self.data_detect.shape[1]==9 else float(self.data_detect[i, -4])])
        
        self.pubCameraMarker.publish(cells)
        self.pubCameraObstacle.publish(camera_obstacle)


    def pub_callback_camera_data(self):
        """
        Callback of publisher (timer), publish the topic 'camera_ori_data'.
        """
        # 发送相机状态
        self.update_camera_state()

        # 发送相机检测到的障碍物信息 
        self.update_camera_obstalce()

        # 可视化结果
        if self.ret:
            bridge = CvBridge()  # 发布原始图像和检测后的图像
            img_ori = bridge.cv2_to_imgmsg(self.frame, "bgr8")
            self.pubCameraOriRviz.publish(img_ori)
            if self.config["detect_config"]["enable_vis"]:
                img_detect = bridge.cv2_to_imgmsg(self.image_detect, "rgb8")
                self.pubCameraDetectRviz.publish(img_detect)

        
    def callback_read_camera(self):
        """
        读入图像, 更新 self.ret self.frame
        """
        if self.config["detect_config"]["enable_detect"]:
            #----------------- 判断是否能读出图像 -----------------------#
            if self.config["orin_image"]["flag"] == 0 or self.config["orin_image"]["flag"] == 1:
                self.ret, self.frame = self.cap.read()
            else:
                try:
                    self.frame = self.hikcamera.get_image_BGR()
                    self.ret = True if type(self.frame) is np.ndarray else False
                except:
                    self.ret = False
            #----------------------------------------------------------#

            #----------------- 相机掉线重新启动相机 -----------------------#
            if self.ret is not True:
                print("----------------------------------------------------")
                print("--------------------Camera_down---------------------")
                print("----------------------------------------------------")
                try:
                    # 产生相机对象 self.cap, 尝试重启相机
                    self.load_video_camera(self.config["orin_image"]["flag"])
                except:
                    pass

                # 相机掉线刷新数据, 防止卡在最后一帧
                self.ret = False
                self.frame = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
                self.image_detect = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
                self.data_detect = []
            else:
                # 读到图片后的第一件事情就是改变其大小, 然后进行畸变矫正
                self.frame = cv2.resize(self.frame, self.read_image_size)
                self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
                self.out.write(self.frame)
                # self.frame = cv2.undistort(self.frame, self.internal_param, self.distortion_cofficients.squeeze())
            #-----------------------------------------------------------#
        else:
            print("----------------------------------------------------")
            print("-------------------- 未开启检测 ----------------------")
            print("----------------------------------------------------")

            # 相机掉线刷新数据, 防止卡在最后一帧
            self.ret = False
            self.frame = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
            self.image_detect = np.zeros((self.read_image_size[1], self.read_image_size[0], 3), dtype=np.uint8)
            self.data_detect = []

    def callback_detect_image(self):
        """
        Callback of publisher (timer), publish the topic 'camera_detect_data'.
        """
        print("#############################################################")
        t0 = time.time()
        #--------------- 当读到图像以后，使用检测算法 -----------------#
        if self.ret == True:
            if self.config["orin_image"]["is_out_video"]:
                self.out_video.write(self.frame)
            
            # 读入的图像格式是 RGB, 输出的 image_detect 是 RGB 格式
            with torch.no_grad():
                (self.data_detect, self.image_detect) = self.yolov8_sort.detect_image(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB))
        #-----------------------------------------------------------#
        self.process_time = time.time() - t0

        print("FPS:%.2f, enable_track:%d, enable_vis:%d, read_image_size:(%d,%d)"%
              (1/max(self.process_time,1e-2), self.config["detect_config"]["enable_track"],self.config["detect_config"]["enable_vis"],self.read_image_size[0],self.read_image_size[1]))
        print("camera_flag:%d, save_video:%d, enable_detect:%d"%(self.config["orin_image"]["flag"], self.config["orin_image"]["is_out_video"], self.config["detect_config"]["enable_detect"]))
        print("#############################################################")

    def load_video_camera(self, flag):
        """
        依据 flag 和 video_path 生成 self.cap
        """
        if flag == 0:  # 使用视频进行测试
            self.cap = cv2.VideoCapture(self.config["orin_image"]["test_video_path"])
            print("----初始化相机成功, read video----")
        
        elif flag == 1:  # 读取 usb 相机
            self.cap = cv2.VideoCapture(self.config["orin_image"]["usb_dev"])
            print("----初始化相机成功, read USB camera----")
        
        else:  # 读取网口相机
            self.hikcamera = HIKCamera()
            self.hikcamera.start_camera()
            print("----初始化相机成功, read WALAN camera----")

def main():
    rclpy.init()
    rosNode = CameraOri(name=ros_node_name)
    rclpy.spin(rosNode)
    rclpy.shutdown()
