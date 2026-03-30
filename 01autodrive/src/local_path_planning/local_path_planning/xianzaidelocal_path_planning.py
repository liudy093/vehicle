import sys
import os
import time
import copy
import yaml
from easydict import EasyDict
import numpy as np
import array
import cv2
import rclpy
import math
from rclpy.node import Node
from car_interfaces.msg import *
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
ros_node_name = "local_path_planning"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import tjitools

class LocalPathPlanning(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubLocalPathPlanning = self.create_publisher(LocalPathPlanningInterface, "local_path_planning_data", 10)
        self.timerLocalPathPlanning = self.create_timer(0.1, self.pub_callback_local_path_planning)

        # define subscribers
        self.subCarDecision = self.create_subscription(CarDecisionInterface, "car_decision_data", self.sub_callback_car_decision, 10)
        self.subGlobalPathPlanning = self.create_subscription(GlobalPathPlanningInterface, "global_path_planning_data", self.sub_callback_global_path_planning, 10)
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 10)
        self.pubLidar_LocalPathPlanning_grid = self.create_publisher(GridCells, 'LocalPathPlanning_grid', 10)
        
        root_path = os.getcwd() + "/src/local_path_planning/local_path_planning/"  # 根目录
        # ======================================================== #
        # 从config文件中读取超参数
        yaml_file = root_path + 'config.yaml'
        f = open(yaml_file)
        config = yaml.load(f)
        yaml_data = EasyDict(config)
        print(yaml_data)

        #self.platoon_state = 0 #初始情况 0表示不在编队状态
        #self.ego_platoon_ID = None #初始情况 无队列ID
        #self.front_car_state = None #编队，前车信息




        self.carDec = None
        self.glbPath = None
        self.fusData = None
        self.locPath = None
        
        ##############################################################################
        self.is_changing_line = False   #正在绕行标志位
        self.return_idx = 0             #记录绕行结束点在全局轨迹中的位置

        self.is_turning = False         #正在转弯标志位

        self.keep_change_line = yaml_data.keep_change_line                      #换道后是否返回
        self.allow_change_line = yaml_data.allow_change_line                    #是否允许绕行
        self.change_distance = yaml_data.change_distance                        #绕行距离
        self.need_change_line_distance = yaml_data.need_change_line_distance    #安全距离右侧向左距离范围内有障碍物，则需要转向
        ##############################################################################
        self.lateral_safe_distance = yaml_data.lateral_safe_distance            #车左右安全距离

        self.speed_max = 0     #最大速度
        self.speed_close_to_target = 0

        self.ACC_speed_max = yaml_data.ACC_speed_max                             #最大跟车速度3
        self.ACC_speed_min = yaml_data.ACC_speed_min                             #最小跟车速度1
        ##############################################################################
        self.change_line_speed = yaml_data.change_line_speed                     #绕行速度2
        ##############################################################################

        self.max_platoon_speed = yaml_data.max_platoon_speed                     #编队行驶最高速度4
        self.safe_platoon_distance = yaml_data.safe_platoon_distance             #编队行驶安全距离20

    def pub_callback_local_path_planning(self):
        """
        Callback of publisher (timer), publish the topic local_path_planning_data.
        :param None.
        """
        msgLocalPathPlanning = LocalPathPlanningInterface()
        now_ts = time.time()
        msgLocalPathPlanning.timestamp = now_ts

        # if self.glbPath is None or self.fusData is None:
        #     print("self.glbPath is None or self.fusData is None")
        #     return
        
        if self.glbPath is None:
            print("self.glbPath is None")
            return
        if self.fusData is None:
            print("self.fusData is None")
            return

        if len(self.glbPath) == 0:
            print("len(self.glbPath) == 0")
            return


        lon_ = self.fusData.longitude
        lat_ = self.fusData.latitude
        vel_ = self.fusData.carspeed
        yaw_ = self.fusData.yaw
        
        err_lon_ = self.glbPath[:,0] - lon_
        err_lat_ = self.glbPath[:,1] - lat_
        err_ = err_lon_**2 + err_lat_**2
        now_idx = err_.argmin()

        tjitools.set_gps_org(lon_,lat_,0) #设置GPS原点
        
        #################################################################################
        if now_idx > self.return_idx and self.is_changing_line is True:
            self.is_changing_line = False
        #################################################################################
        #print("now_idx=%d, lon=%.11f, lat=%.11f, yaw=%.2f"%(now_idx, lon_, lat_, yaw_))
        #print("return_idx", self.return_idx)

        if now_idx + 500 < len(self.glbPath):
            loc_path_end = self.glbPath[now_idx+500, 0:2]
            loc_path_lon = self.glbPath[now_idx:now_idx+500, 0]
            loc_path_lat = self.glbPath[now_idx:now_idx+500, 1]
            loc_path_vel = self.glbPath[now_idx:now_idx+500, 2]
            loc_path_ang = self.glbPath[now_idx:now_idx+500, 3]
        else:
            loc_path_end = self.glbPath[-1, 0:2]
            loc_path_lon = self.glbPath[now_idx:, 0]
            loc_path_lat = self.glbPath[now_idx:, 1]
            loc_path_vel = self.glbPath[now_idx:, 2]
            loc_path_ang = self.glbPath[now_idx:, 3]

            pad_len = 500 - len(loc_path_lon)
            loc_path_lon = np.pad(loc_path_lon, pad_width=(0, pad_len), mode='edge')
            loc_path_lat = np.pad(loc_path_lat, pad_width=(0, pad_len), mode='edge')
            loc_path_vel = np.pad(loc_path_vel, pad_width=(0, pad_len), mode='edge')
            loc_path_ang = np.pad(loc_path_ang, pad_width=(0, pad_len), mode='edge')

            loc_path_vel[-pad_len:] = 0
        
        print("global_speed = ", loc_path_vel[0])
        self.speed_max = loc_path_vel[0]
        self.speed_close_to_target = np.mean(loc_path_vel[1:31])        
        #print("speedmax !!= ", self.speed_max )

        ##########################################################################
        #60个点范围内，航向角偏差大于30度，视为转向
        error_ang = loc_path_ang[0] - loc_path_ang[60]
        if error_ang > 180:
            error_ang = error_ang -360
        if error_ang <-180:
            error_ang = error_ang +360
            
        if error_ang > 30:
            msgLocalPathPlanning.turn_signals = 1.0
        elif error_ang < -30:
            msgLocalPathPlanning.turn_signals = -1.0
        else:
            msgLocalPathPlanning.turn_signals = 0.0

        if abs(error_ang) > 60:
            self.is_turning = True
            self.speed_max = 2
        elif abs(error_ang) > 30:
            self.is_turning = True
            self.speed_max = 3
        else:
            self.is_turning = False
        ###########################################################################
        #筛选障碍物
        #if len(self.fusData.obstacledata) > 0: 
        if True:

            #局部轨迹 gps转车体
            gps_ = np.array([loc_path_lon, loc_path_lat, np.zeros_like(loc_path_lat)]).swapaxes(0,1)
            car_gps_ = np.array([lon_, lat_, 0])
            car_rot_ = np.array([0, 0, yaw_])

            path_rfu_ = tjitools.gps_to_rfu(gps_, car_gps_, car_rot_)
            # path_rfu_ = tjitools.gps_to_enu(np.array())
            #print(path_rfu_)

            obss_ = np.array(self.fusData.obstacledata).reshape(-1,9)
            obss_[:,5] = obss_[:,5] + 3.5
            obstacles_on_road = []
            
            #筛选道路上的障碍物
            whether_left_line_clear = True  #默认道路清空
            need_change_line = False        #默认直线行使

            for o_ in obss_:
                #print("o_x", o_[4])
                #print("o_y", o_[5])
                
                obs_err_x = o_[4] - path_rfu_[:,0]
                obs_err_y = o_[5] - path_rfu_[:,1]
                obs_err_d = np.sqrt(obs_err_x**2 + obs_err_y**2)
                min_distance = obs_err_d.min()
                min_distance_number = obs_err_d.argmin()
                if min_distance_number == len(path_rfu_)-1:
                    min_distance_number -= 1

                #print("min_distance", min_distance)
                min_distance_signal = min_distance #障碍物到路径距离 在左侧为负值，右侧为正值
                if (path_rfu_[min_distance_number+1,0]-path_rfu_[min_distance_number,0])*(o_[5]-path_rfu_[min_distance_number,1]) \
                    -(path_rfu_[min_distance_number+1,1]-path_rfu_[min_distance_number,1])*(o_[4]-path_rfu_[min_distance_number,0]) > 0:
                   
                    min_distance_signal = -min_distance_signal
                
                #如果传感器可视范围内，绕行路线上有障碍物，禁止绕行
                if min_distance_signal < self.lateral_safe_distance -self.need_change_line_distance and min_distance_signal > \
                    -(self.lateral_safe_distance + self.change_distance) and self.is_changing_line == False:
                    
                    whether_left_line_clear = False
                    #print("distance_do not change lane=", min_distance_signal)
                    #print("do not change lane")

                #如果前方15米内，道路右侧有障碍物，且障碍物速度小于1， 提出绕行请求
                if min_distance_signal < self.lateral_safe_distance and min_distance_signal > self.lateral_safe_distance \
                    - self.need_change_line_distance and o_[5] < 15 and (o_[7] + vel_) < 1:
                    
                    need_change_line = True

                if self.is_changing_line == True and o_[4] > 0 and o_[4] < self.lateral_safe_distance + self.change_distance \
                    and o_[5] > -5 and o_[5] < 5 or self.keep_change_line == True: 

                    self.return_idx = now_idx + 40


                #记录当前路线和绕行路线上的障碍物
                if min_distance_signal < self.lateral_safe_distance and min_distance_signal > -(self.lateral_safe_distance + self.change_distance):
                    obstacles_information = []
                    obstacles_information.append(o_[4])                 # 0横向距离
                    obstacles_information.append(o_[5])                 # 1纵向距离
                    obstacles_information.append(min_distance)          # 2到路径距离
                    obstacles_information.append(min_distance_signal)   # 3到路径距离 有符号
                    obstacles_information.append(o_[7] + vel_)          #4障碍物径向速度
                    obstacles_information.append(o_[8] + vel_)          #5障碍物横向速度
                    obstacles_information.append(min_distance_signal) 

                    obstacles_on_road.append(obstacles_information)
            self.show_obstacle(obstacles_on_road)

            #如果需要绕行，且允许绕行，且当前不处于绕行和转弯状态，则进入绕行状态
            if need_change_line == True and whether_left_line_clear == True and self.is_changing_line == False and self.is_turning == False and self.allow_change_line:  
                
                self.is_changing_line = True
                self.return_idx = now_idx + 60
            #########################################################################################################################3

            #制定轨迹
            if self.is_changing_line == True:
                #print('is changing lane')
                loc_path_vel = self.change_line_speed * np.ones_like(loc_path_vel)

                tjitools.set_gps_org(gps_[0,0], gps_[0,1], 0.0)
                path_enu_ = tjitools.gps_to_enu(gps_)  
                #print("path_enu", path_enu_.shape[0])
                path_enu_[:,0] = path_enu_[:,0] + self.change_distance * np.cos(np.deg2rad(loc_path_ang+90))
                path_enu_[:,1] = path_enu_[:,1] + self.change_distance * np.sin(np.deg2rad(loc_path_ang+90))
                gps_new_path = tjitools.enu_to_gps(path_enu_) 
                loc_path_lon = gps_new_path[:,0]
                loc_path_lat = gps_new_path[:,1]

                min_distance = 100
                for obstacle in obstacles_on_road:
                    if obstacle[3] < -self.lateral_safe_distance-self.change_distance or obstacle[3] > self.lateral_safe_distance - self.change_distance:
                        continue
                    min_distance = min(obstacle[1], min_distance)
                
                if min_distance < 10.0+3.5:
                    loc_path_vel = 0.0 * np.ones_like(loc_path_vel)

            else:              
                print('keep lane') 
                # find the min distence
                min_distance = 100
                for obstacle in obstacles_on_road:
                    if obstacle[2] > self.lateral_safe_distance:
                        continue
                    min_distance = min(obstacle[1], min_distance)
                    obstacle_speed_y = obstacle[4]
                    obstacle_speed_x = obstacle[5]

                print("min_diatance =", min_distance)
                #如果在编队模式中，则不考虑ACC
                if min_distance < 25.0: #and self.platoon_state !=1
                    if obstacle_speed_y > self.ACC_speed_min:
                        if obstacle_speed_y > self.ACC_speed_max:
                            loc_path_vel = self.ACC_speed_max * np.ones_like(loc_path_vel)
                        else:
                            loc_path_vel = obstacle_speed_y * np.ones_like(loc_path_vel)
                    else:
                        loc_path_vel = self.ACC_speed_min * np.ones_like(loc_path_vel)
                    
                if min_distance < 10.0+3.5:
                    loc_path_vel = 0.0 * np.ones_like(loc_path_vel)


        if self.carDec is not None:
            if self.carDec.speed == True:
                if self.carDec.maxspeed < self.speed_max:
                    self.speed_max = self.carDec.maxspeed

        self.speed_max = min(self.speed_max, self.speed_close_to_target)
        
        print("speed_max = ", self.speed_max)
        loc_path_vel[loc_path_vel>self.speed_max] = self.speed_max
        
        print('refspeed=', loc_path_vel[0])

        # ================================================================
        # 计算参考加速度
        # ================================================================
        loc_path_acc = np.zeros_like(loc_path_vel)  # 初始化加速度数组
        
        # 确保obstacles_on_road变量存在
        if 'obstacles_on_road' not in locals():
            obstacles_on_road = []
        
        # 方法1：基于速度差分计算加速度
        # 估算相邻点之间的时间间隔
        if len(loc_path_vel) > 1:
            # 假设车辆以当前速度行驶，计算相邻点的时间间隔
            current_speed = max(vel_, 0.1)  # 避免除零，最小速度0.1 m/s
            
            # 计算相邻路径点之间的距离
            if len(loc_path_lon) > 1:
                # 使用经纬度计算距离（简化为平面距离）
                dx = (loc_path_lon[1] - loc_path_lon[0]) * 111320 * np.cos(np.radians(loc_path_lat[0]))
                dy = (loc_path_lat[1] - loc_path_lat[0]) * 111320
                ds = np.sqrt(dx**2 + dy**2)  # 路径点间距（米）
                dt = max(ds / current_speed, 0.01)  # 时间间隔，最小0.01秒
            else:
                dt = 0.1  # 默认100ms
            
            # 计算加速度：a = (v_{k+1} - v_k) / dt
            for i in range(len(loc_path_vel) - 1):
                dv = loc_path_vel[i+1] - loc_path_vel[i]
                loc_path_acc[i] = dv / dt
            
            # 最后一个点的加速度设为前一个点的加速度
            loc_path_acc[-1] = loc_path_acc[-2] if len(loc_path_vel) > 1 else 0.0
        
        # 方法2：考虑转弯时的向心加速度约束
        if len(loc_path_ang) > 2:
            for i in range(1, len(loc_path_ang) - 1):
                # 计算曲率变化
                angle_diff = abs(loc_path_ang[i+1] - loc_path_ang[i-1])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                
                # 如果转弯角度较大，限制加速度
                if angle_diff > 10:  # 转弯角度大于10度
                    # 根据转弯半径限制加速度
                    max_lateral_acc = 2.0  # 最大横向加速度 m/s²
                    curvature_factor = min(1.0, 30.0 / angle_diff)  # 曲率因子
                    max_acc_at_curve = max_lateral_acc * curvature_factor
                    
                    # 限制纵向加速度
                    if abs(loc_path_acc[i]) > max_acc_at_curve:
                        loc_path_acc[i] = np.sign(loc_path_acc[i]) * max_acc_at_curve
        
        # 方法3：考虑前方障碍物的减速需求
        if len(obstacles_on_road) > 0:
            min_obstacle_distance = float('inf')
            for obstacle in obstacles_on_road:
                if obstacle[1] > 0 and obstacle[2] < self.lateral_safe_distance:  # 前方的障碍物
                    min_obstacle_distance = min(min_obstacle_distance, obstacle[1])
            
            # 如果前方有障碍物，计算安全减速度
            if min_obstacle_distance < 30.0:  # 30米内有障碍物
                safe_distance = 5.0  # 安全停车距离
                if min_obstacle_distance > safe_distance:
                    # 计算安全减速度：v² = v₀² + 2a(s-s₀)
                    # 假设要减速到安全速度（如障碍物速度或更低）
                    target_speed = 1.0  # 目标安全速度
                    decel_distance = min_obstacle_distance - safe_distance
                    if decel_distance > 0 and vel_ > target_speed:
                        required_decel = -(vel_**2 - target_speed**2) / (2 * decel_distance)
                        # 对前方若干个点应用减速
                        affected_points = min(int(decel_distance / 2), len(loc_path_acc))
                        for i in range(affected_points):
                            if loc_path_acc[i] > required_decel:
                                loc_path_acc[i] = required_decel
        
        # 最终限制加速度范围
        if self.is_turning:
            # 转弯时限制加速度
            max_acceleration = 1.5
            min_acceleration = -2.0
        else:
            # 直行时的正常限制
            max_acceleration = 2.5
            min_acceleration = -3.0
        
        loc_path_acc = np.clip(loc_path_acc, min_acceleration, max_acceleration)
        
        # 平滑处理：避免加速度突变
        if len(loc_path_acc) > 2:
            # 简单的移动平均滤波
            for i in range(1, len(loc_path_acc) - 1):
                loc_path_acc[i] = 0.5 * loc_path_acc[i] + 0.25 * (loc_path_acc[i-1] + loc_path_acc[i+1])
        
        print('ref_acceleration=', loc_path_acc[0])
        # ================================================================


        msgLocalPathPlanning.startpoint = array.array('d', [lon_, lat_])
        msgLocalPathPlanning.endpoint = array.array('d', loc_path_end)
        msgLocalPathPlanning.longitude = array.array('d', loc_path_lon)
        msgLocalPathPlanning.latitude = array.array('d', loc_path_lat)
        msgLocalPathPlanning.speed = array.array('f', loc_path_vel)
        msgLocalPathPlanning.acceleration = array.array('f', loc_path_acc)
        msgLocalPathPlanning.angle = array.array('f', loc_path_ang)
        msgLocalPathPlanning.process_time = time.time() - now_ts

        self.pubLocalPathPlanning.publish(msgLocalPathPlanning)

        self.locPath = np.array([loc_path_lon, loc_path_lat]).swapaxes(0,1)

    def show_obstacle(self, obstacles_on_road):
            cells = GridCells()
            cells.header.frame_id = "rslidar"
            cells.cell_width = 0.5  # edit for grid size .3 for simple map
            cells.cell_height = 0.5  # edit for grid size
        
            for o_ in obstacles_on_road:
                point = Point()
                point.x = float(o_[1])
                point.y = -float(o_[0])
                point.z = -1.0
                cells.cells.append(point)
                # x_ = round(o_[4]/img_f) + img_w//2
                # y_ = -round(o_[5]/img_f) + img_h//2
            self.pubLidar_LocalPathPlanning_grid.publish(cells)
            
            
    def sub_callback_car_decision(self, msgCarDecision:CarDecisionInterface):
        """
        Callback of subscriber, subscribe the topic car_decision_data.
        :param msgCarDecision: The message heard by the subscriber.
        """
        self.carDec = msgCarDecision
        pass

    def sub_callback_global_path_planning(self, msgGlobalPathPlanning:GlobalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic global_path_planning_data.
        :param msgGlobalPathPlanning: The message heard by the subscriber.
        """
        msgGlbPath = msgGlobalPathPlanning
        self.glbPath = np.array(msgGlbPath.routedata)
        self.glbPath = self.glbPath.reshape(-1,4)# 数组重塑
        pass

    def sub_callback_fusion(self, msgFusion:FusionInterface):
        """
        Callback of subscriber, subscribe the topic fusion_data.
        :param msgFusion: The message heard by the subscriber.
        """
        self.fusData = msgFusion
        pass


    def show_local_path(self):

        img_h = 400
        img_w = 400
        img_f = 0.5
        
        lon_ = self.locPath[:,0]
        lat_ = self.locPath[:,1]
        alt_ = np.zeros_like(lon_)
        path_gps = np.array([lon_, lat_, alt_]).swapaxes(0,1)
        path_enu = tjitools.gps_to_enu(path_gps)
        path_enu = path_enu - path_enu[0]

        px_ = img_w//2 + np.round(path_enu[:,0]/img_f)
        py_ = img_h//2 - np.round(path_enu[:,1]/img_f)
        
        validx = (px_>=0) & (px_<img_w) & (py_>=0) & (py_<img_h)
        px_ = px_[validx]
        py_ = py_[validx]

        img_ = 255 * np.ones(shape=(img_h*img_w, 3), dtype=np.uint8)
        pix_ = (py_*img_w + px_).astype(np.int32)
        img_[pix_] = (0,0,255)
        img_ = img_.reshape(img_h, img_w, 3)
        img_[img_h//2-3:img_h//2+3, img_w//2-3:img_w//2+3] = (255,0,0)
        cv2.imshow('loc_trj', cv2.cvtColor(img_, cv2.COLOR_RGB2BGR))
        cv2.waitKey(5)

def WRP_gps_to_rfu(gps,angle):

    Path_xy = tjitools.gps_to_enu(gps)
    Path_x = Path_xy[0,:].copy()
    Path_y = Path_xy[1,:].copy()
    Path_z = Path_xy[2,:].copy()
    Path_x = Path_xy[0,:].copy()*math.cos(angle) - Path_xy[1,:].copy()*math.sin(angle) + 3.5845079
    Path_y = Path_xy[0,:].copy()*math.sin(angle) + Path_xy[1,:].copy()*math.cos(angle) + 0.1
    result = np.concatenate([Path_x,Path_y,Path_z],axis=0)
    return result


def main():
    rclpy.init()
    rosNode = LocalPathPlanning()
    rclpy.spin(rosNode)
    rclpy.shutdown()
