#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: Aeolson
# @File: hmi.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

from PyQt5.QtWidgets import QInputDialog, QFileDialog
import sys
import os
import time
import copy
import json
import threading
import signal
import psutil
from jtop import jtop  # toolkit for jetson
import numpy as np
import array

ros_node_name = "hmi"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/" % (ros_node_name, ros_node_name))

import rclpy
from rclpy.node import Node
from car_interfaces.msg import *
import tjitools
import roadnet
# import qt5 tools
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import pyqtgraph as pg  # pyqtgraph, 绘图模块

pg.setConfigOption('background', 'w')
from Ui_tji_autodrive import Ui_MainWindow


class Hmi(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # define publishers
        self.pubHmiStartEndPoint = self.create_publisher(HmiStartEndPointInterface, "hmi_start_end_point_data", 10)
        self.pubHmiPlatoon = self.create_publisher(HmiPlatoonInterface, "platoon_state_data", 10)
        #self.pubHmiParking = self.create_publisher(HmiParkingInterface, "parking_data", 10)
        self.timerHmiStartEndPoint = self.create_timer(0.1, self.pub_callback_hmi_start_end_point)
        self.timerHmiPlatoon = self.create_timer(0.1, self.pub_callback_hmi_platoon)
        #self.timerHmiParking = self.create_timer(0.1, self.pub_callback_hmi_parking)

        # define subscribers
        self.subCameraState = self.create_subscription(CameraStateInterface, "camera_state_data",
                                                       self.sub_callback_camera_state, 1)
        self.subLidarState = self.create_subscription(LidarStateInterface, "lidar_state_data",
                                                      self.sub_callback_lidar_state, 1)
        self.subRadarState = self.create_subscription(RadarStateInterface, "radar_state_data",
                                                      self.sub_callback_radar_state, 1)
        self.subSonicState = self.create_subscription(SonicStateInterface, "sonic_state_data",
                                                      self.sub_callback_sonic_state, 1)
        self.subCanState = self.create_subscription(CanStateInterface, "can_state_data", self.sub_callback_can_state, 1)
        self.subGpsState = self.create_subscription(GpsStateInterface, "gps_state_data", self.sub_callback_gps_state, 1)
        self.subImuState = self.create_subscription(ImuStateInterface, "imu_state_data", self.sub_callback_imu_state, 1)
        self.subFusion = self.create_subscription(FusionInterface, "fusion_data", self.sub_callback_fusion, 1)
        self.subCloud_1 = self.create_subscription(NetReceiveData1Interface, "cloud_data_1", self.sub_callback_cloud_1, 1)
        self.subCloud_2 = self.create_subscription(NetReceiveData2Interface, "cloud_data_2", self.sub_callback_cloud_2, 1)
        self.subGlobalPathPlanning = self.create_subscription(GlobalPathPlanningInterface, "global_path_planning_data",
                                                              self.sub_callback_global_path_planning, 10)
        self.subLocalPathPlanning = self.create_subscription(LocalPathPlanningInterface, "local_path_planning_data",
                                                             self.sub_callback_local_path_planning, 10)

        self.msgCameraState = None
        self.msgLidarState = None
        self.msgRadarState = None
        self.msgSonicState = [None, None, None, None]
        self.msgCanState = None
        self.msgGpsState = None
        self.msgImuState = None
        self.msgFusion = None
        self.msgGlobalPathPlanning = None
        self.msgLocalPathPlanning = None
        self.msgCloud_1 = None
        self.msgCloud_2 = None

        self.gpsStartPoint = None
        self.gpsEndPoint = None
        self.isGlobalPathUpdated = False

        self.CloudState = None
        self.PlatoonState = None
        self.PlatoonID = None
        #self.ParkingType = 0

    def pub_callback_hmi_start_end_point(self):
        """
        Callback of publisher (timer), publish the topic 'hmi_start_end_point_data'.
        :param None.
        """
        msgHmiStartEndPoint = HmiStartEndPointInterface()  # 创建消息对象
        ts = time.time()
        msgHmiStartEndPoint.timestamp = ts
        if self.gpsStartPoint is not None and self.gpsEndPoint is not None:
            msgHmiStartEndPoint.startpoint = array.array('d', self.gpsStartPoint[:2])
            msgHmiStartEndPoint.endpoint = array.array('d', self.gpsEndPoint[:2])
        msgHmiStartEndPoint.process_time = time.time() - ts
        self.pubHmiStartEndPoint.publish(msgHmiStartEndPoint)
        # print("pub_callback_hmi_start_end_point")

    def pub_callback_hmi_platoon(self):
        """
        Callback of publisher (timer), publish the topic 'hmi_other_functions_data'.
        :param None.
        """
        msgHmiPlatoon = HmiPlatoonInterface()  # 创建消息对象
        # 获取当前时间戳
        ts = time.time()
        msgHmiPlatoon.timestamp = ts  # 设置时间戳
        if self.PlatoonState == 1:  # 如果选择了编号
            msgHmiPlatoon.platoon_state = self.PlatoonState  # 改为1
            msgHmiPlatoon.platoon_id = self.PlatoonID  # 更新为所选编号
        self.pubHmiPlatoon.publish(msgHmiPlatoon)
        # print("pub_callback_hmi_platoon")

    # def pub_callback_hmi_parking(self):
    #     """
    #     Callback of publisher (timer), publish the topic 'hmi_other_functions_data'.
    #     :param None.
    #     """
    #     msgHmiParking = HmiParkingInterface()  # 创建消息对象
    #     # 获取当前时间戳
    #     ts = time.time()
    #     msgHmiParking.timestamp = ts  # 设置时间戳
    #     msgHmiParking.parking_type = self.ParkingType  # 改为1
    #     self.pubHmiParking.publish(msgHmiParking)

    def sub_callback_camera_state(self, msgCameraState: CameraStateInterface):
        """
        Callback of subscriber, subscribe the topic 'camera_state_data'.
        :param msgCameraState: The message heard by the subscriber.
        """
        self.msgCameraState = copy.deepcopy(msgCameraState)

    def sub_callback_lidar_state(self, msgLidarState: LidarStateInterface):
        """
        Callback of subscriber, subscribe the topic 'lidar_state_data'.
        :param msgLidarState: The message heard by the subscriber.
        """
        self.msgLidarState = copy.deepcopy(msgLidarState)

    def sub_callback_radar_state(self, msgRadarState: RadarStateInterface):
        """
        Callback of subscriber, subscribe the topic 'radar_state_data'.
        :param msgRadarState: The message heard by the subscriber.
        """
        self.msgRadarState = copy.deepcopy(msgRadarState)

    def sub_callback_sonic_state(self, msgSonicState: SonicStateInterface):
        """
        Callback of subscriber, subscribe the topic 'sonic_state_data'.
        :param msgSonicState: The message heard by the subscriber.
        """
        num = msgSonicState.id - 1
        self.msgSonicState[num] = copy.deepcopy(msgSonicState)

    def sub_callback_can_state(self, msgCanState: CanStateInterface):
        """
        Callback of subscriber, subscribe the topic 'can_state_data'.
        :param msgCanState: The message heard by the subscriber.
        """
        self.msgCanState = copy.deepcopy(msgCanState)

    def sub_callback_gps_state(self, msgGpsState: GpsStateInterface):
        """
        Callback of subscriber, subscribe the topic 'gps_state_data'.
        :param msgGpsState: The message heard by the subscriber.
        """
        self.msgGpsState = copy.deepcopy(msgGpsState)

    def sub_callback_imu_state(self, msgImuState: ImuStateInterface):
        """
        Callback of subscriber, subscribe the topic 'imu_state_data'.
        :param msgImuState: The message heard by the subscriber.
        """
        self.msgImuState = copy.deepcopy(msgImuState)

    def sub_callback_fusion(self, msgFusion: FusionInterface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        self.msgFusion = copy.deepcopy(msgFusion)

    def sub_callback_cloud_1(self, msgCloud_1: NetReceiveData1Interface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        self.msgCloud_1 = copy.deepcopy(msgCloud_1)
    def sub_callback_cloud_2(self, msgCloud_2: NetReceiveData2Interface):
        """
        Callback of subscriber, subscribe the topic 'fusion_data'.
        :param msgFusion: The message heard by the subscriber.
        """
        self.msgCloud_2 = copy.deepcopy(msgCloud_2)

    def sub_callback_global_path_planning(self, msgGlobalPathPlanning: GlobalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic 'global_path_planning_data'.
        :param msgGlobalPathPlanning: The message heard by the subscriber.
        """

        if len(msgGlobalPathPlanning.startpoint) == 0 or len(msgGlobalPathPlanning.endpoint) == 0:
            self.msgGlobalPathPlanning = None
            return

        if self.msgGlobalPathPlanning is None:
            if len(msgGlobalPathPlanning.routedata) == 0:
                self.msgGlobalPathPlanning = None
                self.isGlobalPathUpdated = False
            else:
                self.msgGlobalPathPlanning = msgGlobalPathPlanning
                self.isGlobalPathUpdated = True
        else:
            if len(msgGlobalPathPlanning.routedata) == 0:
                self.msgGlobalPathPlanning = None
                self.isGlobalPathUpdated = True
            else:
                err_start = np.array(msgGlobalPathPlanning.startpoint, dtype=np.float64) - np.array(
                    self.msgGlobalPathPlanning.startpoint, dtype=np.float64)
                err_end = np.array(msgGlobalPathPlanning.endpoint, dtype=np.float64) - np.array(
                    self.msgGlobalPathPlanning.endpoint, dtype=np.float64)
                if np.max(np.abs(err_start)) > 1e-5 or np.max(np.abs(err_end)) > 1e-5:
                    self.msgGlobalPathPlanning = copy.deepcopy(msgGlobalPathPlanning)
                    self.isGlobalPathUpdated = True
                else:
                    self.isGlobalPathUpdated = False

    def sub_callback_local_path_planning(self, msgLocalPathPlanning: LocalPathPlanningInterface):
        """
        Callback of subscriber, subscribe the topic 'local_path_planning_data'.
        :param msgLocalPathPlanning: The message heard by the subscriber.
        """
        self.msgLocalPathPlanning = msgLocalPathPlanning


class MyMainWindow(QMainWindow, Ui_MainWindow, QWidget):
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent)
        self.setupUi(self)
        self.setFixedSize(self.width(), self.height())
        self.carWidget.setEnabled(False)
        self.tabWidget.setEnabled(False)
        self.tabWidget.setCurrentIndex(0)


        self.buttonConnectCar.clicked.connect(self.action_button_connect_car)
        self.buttonOpenRviz.clicked.connect(self.action_button_open_rviz)

        self.buttonCamera.clicked.connect(self.action_button_camera)
        self.buttonCameraObject.clicked.connect(self.action_button_camera_object)
        self.buttonCameraLane.clicked.connect(self.action_button_camera_lane)
        self.buttonCameraLight.clicked.connect(self.action_button_camera_light)
        self.buttonCameraSign.clicked.connect(self.action_button_camera_sign)

        self.buttonLidar.clicked.connect(self.action_button_lidar)
        self.buttonLidarObject.clicked.connect(self.action_button_lidar_object)

        self.buttonRadar.clicked.connect(self.action_button_radar)

        self.buttonLoadMap.clicked.connect(self.action_button_load_map)
        self.buttonGlobalPlanner.clicked.connect(self.action_button_global_planner)
        self.buttonReplan.clicked.connect(self.action_button_global_replan)
        self.comboDestination.currentIndexChanged.connect(self.action_combo_destination)
        self.buttonTracker.clicked.connect(self.action_button_tracker)

        self.listRecorderPoints.clicked.connect(self.action_list_recorder_point)
        self.buttonPointRecorder_Add.clicked.connect(self.action_button_recorder_point_add)
        self.buttonPointRecorder_Delete.clicked.connect(self.action_button_recorder_point_delete)
        self.buttonPointRecorder_Edit.clicked.connect(self.action_button_recorder_point_edit)
        self.buttonPointRecorder_SaveFile.clicked.connect(self.action_button_recorder_point_savefile)
        self.buttonRouteRecorder.clicked.connect(self.action_button_recorder_route)
        self.buttonTXT2JSON.clicked.connect(self.action_convert_txt_to_json)

        self.buttonCloud.clicked.connect(self.action_button_cloud)

        self.comboBoxPlatoon.currentIndexChanged.connect(self.platoon_choose)
        self.buttonPlatoon.clicked.connect(self.toggle_platoon_mode)
        self.buttonParking.clicked.connect(self.action_button_parking)
        # self.buttonParking1.clicked.connect(self.action_button_parking1)
        # self.buttonParking2.clicked.connect(self.action_button_parking2)
        # self.buttonParking3.clicked.connect(self.action_button_parking3)
        # variables
        self.stateConnected = False
        self.stateSensorCan = False
        self.stateSensorGps = False
        self.stateSensorImu = False
        self.stateSensorSonic = False
        self.stateSensorCamera = False
        self.stateSensorRadar = False
        self.stateSensorLidar = False
        self.stateCloud = False
        self.stateplatoon = False
        self.stateparking = False

        self.stateRviz = False
        self.stateCameraObject = False
        self.stateCameraLane = False
        self.stateCameraLight = False
        self.stateCameraSign = False
        self.stateLidarObject = False

        self.trackerUseTrajectory = False
        self.trackerFilePath = ""
        self.trackerRoadMap = None
        self.trackerTrajectory = None
        self.trackerStartGps = None
        self.trackerEndGps = None
        self.trackerGlobalPath = None
        self.stateTracker = False

        self.recorderPoints = []
        self.recorderRoute = []
        self.stateRouteRecorder = False

        self.percentCpu = 0
        self.percentGpu = 0
        self.percentRam = 0
        self.percentDisk = 0

        # 定时器, 500ms更新车辆和传感器状态
        t_ = QTimer(self)
        t_.timeout.connect(self.timer_update_vehicle_sensor_info)
        t_.start(500)

        # 定时器, 200ms更新传感器目标
        t_ = QTimer(self)
        t_.timeout.connect(self.timer_update_view_objects)
        t_.start(100)
        self.cnt_change_view_objects = 0

        # 定时器, 500ms更新云平台状态
        t_ = QTimer(self)
        t_.timeout.connect(self.timer_update_Cloud_info)
        t_.start(500)

        # 创建ROS线程
        self.rosHmi = Hmi()
        t_ = threading.Thread(target=self.thread_ros_hmi, args=())
        t_.setDaemon(True)
        t_.start()

        # 创建硬件线程监听Orin硬件状态
        t_ = threading.Thread(target=self.thread_orin_hardware, args=())
        t_.setDaemon(True)
        t_.start()

    # user function
    def now_ms(self):
        ms_ = round(time.time() * 1000) % 1000
        ts_ = time.localtime()
        return ":".join(
            [str(ts_.tm_hour).zfill(2), str(ts_.tm_min).zfill(2), str(ts_.tm_sec).zfill(2), str(ms_).zfill(3)])

    def check_process(self, pro_name: str):
        """
        根据进程名检查进程是否存在
        """
        pids = psutil.process_iter()
        for p_ in pids:
            if p_.name() in pro_name:
                return True
        return False

    def kill_process(self, pro_name: str):
        """
        根据进程名获取进程pid, 然后杀死进程
        """
        pids = psutil.process_iter()
        for p_ in pids:
            if p_.name() in pro_name:
                p_.kill()

    def kill_all_processes(self):
        kill_procs = [ \
            'camera_ori', 'camera_detect', 'lane_recognition', 'sign_recognition', 'light_recognition', \
            'lidar_ori', 'lidar_obstacle', 'radar_obstacle', 'sonic_obstacle', \
            'gps', 'imu', 'match_map', 'fusion', 'car_ori', \
            'car_decision', 'global_path_planning', 'local_path_planning', 'pid', 'regulator', 'car_control', \
            'net_work', 'rviz2'
        ]
        for pn_ in kill_procs:
            self.kill_process(pn_)

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.kill_all_processes()
        return super().closeEvent(a0)

    def enable_indicator(self, indicator: QtWidgets.QLabel):
        stysh = indicator.styleSheet().split(';')
        for k_, s_ in enumerate(stysh):
            if 'background-color' in s_:
                stysh[k_] = 'background-color:rgb(138, 226, 52)'
        indicator.setStyleSheet(';'.join(stysh))

    def disable_indicator(self, indicator: QtWidgets.QLabel):
        stysh = indicator.styleSheet().split(';')
        for k_, s_ in enumerate(stysh):
            if 'background-color' in s_:
                stysh[k_] = 'background-color:rgb(136, 138, 133)'
        indicator.setStyleSheet(';'.join(stysh))

    def error_indicator(self, indicator: QtWidgets.QLabel):
        stysh = indicator.styleSheet().split(';')
        for k_, s_ in enumerate(stysh):
            if 'background-color' in s_:
                stysh[k_] = 'background-color:rgb(239, 71, 71)'
        indicator.setStyleSheet(';'.join(stysh))

    def update_view_map(self):
        self.viewMap.plotItem.clear()

        if self.trackerUseTrajectory:
            if self.trackerTrajectory is not None and len(self.trackerTrajectory) > 0:
                self.viewMap.plotItem.plot(
                    self.trackerTrajectory[:, 0], self.trackerTrajectory[:, 1],
                    pen=pg.mkPen(color=(150, 150, 150), width=2)
                )

        else:
            if self.trackerRoadMap is not None and len(self.trackerRoadMap.all_paths) > 0:
                for ph in self.trackerRoadMap.all_paths:
                    self.viewMap.plotItem.plot(
                        ph.longitudes, ph.latitudes,
                        pen=pg.mkPen(color=(150, 150, 150), width=2)
                    )

            # scatter all sites
            if self.trackerRoadMap is not None and len(self.trackerRoadMap.all_sites) > 0:
                st_lonlat = np.array([[st.loc.longitude, st.loc.latitude] for st in self.trackerRoadMap.all_sites],
                                     dtype=np.float64)
                self.viewMap.plotItem.scatterPlot(
                    st_lonlat[:, 0], st_lonlat[:, 1],
                    size=10,
                    brush=pg.mkBrush(color=(237, 212, 0, 204))
                )

            if self.trackerGlobalPath is None:
                # scatter selected destination
                idx_ = self.comboDestination.currentIndex()
                if idx_ != -1:
                    st = self.trackerRoadMap.all_sites[idx_]
                    self.viewMap.plotItem.scatterPlot(
                        [st.loc.longitude], [st.loc.latitude],
                        size=15,
                        brush=pg.mkBrush(color=(237, 212, 0, 204)),
                        pen=pg.mkPen(color=(239, 41, 41, 204), width=5)
                    )

        # plot global path & scatter start and end points
        if self.trackerGlobalPath is not None:
            self.viewMap.plotItem.plot(
                self.trackerGlobalPath[:, 0], self.trackerGlobalPath[:, 1],
                pen=pg.mkPen(color=(52, 101, 164, 204), width=5)
            )

        if self.trackerStartGps is not None:
            self.viewMap.plotItem.scatterPlot(
                [self.trackerStartGps[0]], [self.trackerStartGps[1]],
                size=8,
                brush=pg.mkBrush(color=(0, 180, 0))
            )
        if self.trackerEndGps is not None:
            self.viewMap.plotItem.scatterPlot(
                [self.trackerEndGps[0]], [self.trackerEndGps[1]],
                size=8,
                brush=pg.mkBrush(color=(180, 0, 0))
            )

        if self.rosHmi is not None and self.rosHmi.msgFusion is not None:
            lon_ = self.rosHmi.msgFusion.longitude
            lat_ = self.rosHmi.msgFusion.latitude
            self.viewMap.plotItem.scatterPlot(
                [lon_], [lat_],
                size=15,
                brush=pg.mkBrush(color=(237, 212, 0, 204)),
                pen=pg.mkPen(color=(115, 210, 22, 204), width=5)
            )

        self.viewMap.setAspectLocked()

    def update_list_reorder_points(self):
        idx_ = self.listRecorderPoints.currentIndex().row()

        pt_list = [pt["name"] for pt in self.recorderPoints]
        mdl = QStringListModel()
        mdl.setStringList(pt_list)
        self.listRecorderPoints.setModel(mdl)

        if idx_ != -1 and idx_ < mdl.rowCount():
            cidx_ = mdl.index(idx_)
            self.listRecorderPoints.setCurrentIndex(cidx_)

        self.action_list_recorder_point()

    # timers and threds
    def timer_update_vehicle_sensor_info(self):

        nowMs = self.now_ms()
        if not self.stateConnected:
            return

        # hardware state
        self.barCpu.setValue(round(self.percentCpu))
        self.barGpu.setValue(round(self.percentGpu))
        self.barRam.setValue(round(self.percentRam))
        self.barDisk.setValue(round(self.percentDisk))

        # update the vehicle state
        if self.rosHmi.msgFusion is not None:
            soc_ = self.rosHmi.msgFusion.soc
            thr_ = self.rosHmi.msgFusion.throttle_percentage
            brk_ = self.rosHmi.msgFusion.braking_percentage
            str_ = self.rosHmi.msgFusion.steerangle
            ger_ = self.rosHmi.msgFusion.gearpos
            mod_ = self.rosHmi.msgFusion.car_run_mode
            lon_ = self.rosHmi.msgFusion.longitude
            lat_ = self.rosHmi.msgFusion.latitude
            hda_ = self.rosHmi.msgFusion.yaw
            vel_ = self.rosHmi.msgFusion.carspeed * 3.6

            self.barSoc.setValue(soc_)
            self.barThrottle.setValue(thr_)
            self.barBrake.setValue(brk_)

            if str_ >= 0:
                self.barSteerLeft.setValue(round(str_ / 540 * 100))
                self.barSteerRight.setValue(0)
            else:
                self.barSteerLeft.setValue(0)
                self.barSteerRight.setValue(round(-str_ / 540 * 100))
            self.dispSteerAngle.setText(str(round(str_)))

            if self.rosHmi.msgFusion.left_light:
                self.enable_indicator(self.indicatorSteerLeft)
            else:
                self.disable_indicator(self.indicatorSteerLeft)

            if self.rosHmi.msgFusion.right_light:
                self.enable_indicator(self.indicatorSteerRight)
            else:
                self.disable_indicator(self.indicatorSteerRight)

            if ger_ == 1:
                self.dispGear.setText('P')
            elif ger_ == 2:
                self.dispGear.setText('N')
            elif ger_ == 3:
                self.dispGear.setText('D')
            elif ger_ == 4:
                self.dispGear.setText('R')
            else:
                self.dispGear.setText('Unknown')

            if mod_ == 1:
                self.dispMode.setText('Auto')
                self.enable_indicator(self.indicatorAutoMode)
            else:
                self.dispMode.setText('Manual')
                self.disable_indicator(self.indicatorAutoMode)

            self.dispLongitude.setText(str(round(lon_, 11)))
            self.dispLatitude.setText(str(round(lat_, 11)))
            self.dispHeading.setText(str(round(hda_, 2)))
            self.dispSpeed.setText(str(round(vel_, 2)))

            if self.rosHmi.msgFusion.state == 1:
                self.error_indicator(self.indicatorVehicleState)
                if self.rosHmi.msgFusion.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电池箱报警--单体过压或欠压" % nowMs)
                elif self.rosHmi.msgFusion.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电池箱报警--放电电流异常" % nowMs)
                elif self.rosHmi.msgFusion.error == 3:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电池箱报警--电压报警" % nowMs)
                elif self.rosHmi.msgFusion.error == 4:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电池箱报警--电池温度报警" % nowMs)
                elif self.rosHmi.msgFusion.error == 5:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电池箱报警--电池SOC过低" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] 车辆状态错误: 电池箱报警--未知错误 %d" % (nowMs, self.rosHmi.msgFusion.error))
            elif self.rosHmi.msgFusion.state == 2:
                self.error_indicator(self.indicatorVehicleState)
                if self.rosHmi.msgFusion.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电机控制报警--转向电机控制器故障" % nowMs)
                elif self.rosHmi.msgFusion.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] 车辆状态错误: 电机控制报警--驱动电机控制器故障" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] 车辆状态错误: 电机控制报警--未知错误 %d" % (nowMs, self.rosHmi.msgFusion.error))
            else:
                self.enable_indicator(self.indicatorVehicleState)

        # CAN state
        if self.stateSensorCan:
            if self.rosHmi.msgCanState is not None and self.rosHmi.msgCanState.state == 1:
                self.error_indicator(self.indicatorCanState)
                if self.rosHmi.msgCanState.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 库函数错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 打开设备错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 3:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 初始化CAN0通道错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 4:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 清空CAN0缓冲区错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 5:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 启动CAN0通道错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 6:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 初始化CAN1通道错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 7:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 清空CAN1缓冲区错误" % nowMs)
                elif self.rosHmi.msgCanState.error == 8:
                    self.dispSensorInfo.append("[ERROR] [%s] CAN总线错误: 启动CAN1通道错误" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] CAN总线错误: 未知错误 %d" % (nowMs, self.rosHmi.msgCanState.error))
            else:
                self.enable_indicator(self.indicatorCanState)
        else:
            self.disable_indicator(self.indicatorCanState)

        # GPS state
        if self.stateSensorGps:
            if self.rosHmi.msgGpsState is not None and self.rosHmi.msgGpsState.state == 1:
                self.error_indicator(self.indicatorGpsState)
                if self.rosHmi.msgGpsState.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 打开串口错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 读取共享内存错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 3:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 写入共享内存错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 4:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 读取串口数据错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 5:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: GPS数据校验错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 6:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 非组合导航模式" % nowMs)
                elif self.rosHmi.msgGpsState.error == 7:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 无GPS消息" % nowMs)
                elif self.rosHmi.msgGpsState.error == 8:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 无车辆消息" % nowMs)
                elif self.rosHmi.msgGpsState.error == 9:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 陀螺错误" % nowMs)
                elif self.rosHmi.msgGpsState.error == 10:
                    self.dispSensorInfo.append("[ERROR] [%s] GPS设备错误: 加表错误" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] GPS设备错误: 未知错误 %d" % (nowMs, self.rosHmi.msgGpsState.error))
            else:
                self.enable_indicator(self.indicatorGpsState)
        else:
            self.disable_indicator(self.indicatorGpsState)

        # IMU state
        if self.stateSensorImu:
            if self.rosHmi.msgImuState is not None and self.rosHmi.msgImuState.state == 1:
                self.error_indicator(self.indicatorImuState)
                self.dispSensorInfo.append(
                    "[ERROR] [%s] IMU设备错误: 未知错误 %d" % (nowMs, self.rosHmi.msgImuState.error))
            else:
                self.enable_indicator(self.indicatorImuState)
        else:
            self.disable_indicator(self.indicatorImuState)

        # Sonic state
        if self.stateSensorSonic:

            clear_flag = True
            for msg in self.rosHmi.msgSonicState:
                if msg is not None:
                    if abs(time.time() - msg.timestamp) > 1.0:
                        msg = None
                    else:
                        clear_flag = False
                        break

            if clear_flag:
                self.stateSensorSonic = False
                self.disable_indicator(self.indicatorSonicState)

            else:
                self.enable_indicator(self.indicatorSonicState)
                for msg in self.rosHmi.msgSonicState:
                    if msg is not None and msg.state == 1:
                        self.error_indicator(self.indicatorSonicState)
                        if msg.error == 1:
                            # self.dispSensorInfo.append("[ERROR] [%s] Sonic %d 设备错误: 数据传输错误"%(nowMs, msg.id))
                            pass
                        else:
                            self.dispSensorInfo.append(
                                "[ERROR] [%s] Sonic %d 设备错误: 未知错误 %d" % (nowMs, msg.id, msg.error))

        else:
            for msg in self.rosHmi.msgSonicState:
                if msg is not None:
                    self.stateSensorSonic = True

            if not self.stateSensorSonic:
                self.disable_indicator(self.indicatorSonicState)

        # Camera state
        if self.stateSensorCamera:
            if self.rosHmi.msgCameraState is not None and self.rosHmi.msgCameraState.state == 1:
                self.error_indicator(self.indicatorCameraState)
                if self.rosHmi.msgCameraState.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] Camera设备错误: 未连接设备" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] Camera设备错误: 未知错误 %d" % (nowMs, self.rosHmi.msgCameraState.error))
            else:
                self.enable_indicator(self.indicatorCameraState)
        else:
            self.disable_indicator(self.indicatorCameraState)

        # Radar state
        if self.stateSensorRadar:
            if self.rosHmi.msgRadarState is not None and self.rosHmi.msgRadarState.state == 1:
                self.error_indicator(self.indicatorRadarState)
                if self.rosHmi.msgRadarState.error == 1:
                    self.dispSensorInfo.append("[ERROR] [%s] Radar设备错误: 通信错误" % nowMs)
                elif self.rosHmi.msgRadarState.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] Radar设备错误: 内部错误" % nowMs)
                elif self.rosHmi.msgRadarState.error == 3:
                    self.dispSensorInfo.append("[ERROR] [%s] Radar设备错误: 阻塞错误" % nowMs)
                elif self.rosHmi.msgRadarState.error == 4:
                    self.dispSensorInfo.append("[ERROR] [%s] Radar设备错误: 过热错误" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] Radar设备错误: 未知错误 %d" % (nowMs, self.rosHmi.msgRadarState.error))
            else:
                self.enable_indicator(self.indicatorRadarState)
        else:
            self.disable_indicator(self.indicatorRadarState)

        # Lidar state
        if self.stateSensorLidar:
            if self.rosHmi.msgLidarState is not None and self.rosHmi.msgLidarState.state == 1:
                self.error_indicator(self.indicatorLidarState)
                if self.rosHmi.msgLidarState.error == 1:
                    # self.dispSensorInfo.append("[ERROR] [%s] Lidar设备错误: 通讯错误"%nowMs)
                    pass
                elif self.rosHmi.msgLidarState.error == 2:
                    self.dispSensorInfo.append("[ERROR] [%s] Lidar设备错误: 通讯接收异常" % nowMs)
                elif self.rosHmi.msgLidarState.error == 3:
                    self.dispSensorInfo.append("[ERROR] [%s] Lidar设备错误: 校验错误" % nowMs)
                else:
                    self.dispSensorInfo.append(
                        "[ERROR] [%s] Lidar设备错误: 未知错误 %d" % (nowMs, self.rosHmi.msgLidarState.error))
            else:
                self.enable_indicator(self.indicatorLidarState)
        else:
            self.disable_indicator(self.indicatorLidarState)

        # update map view
        self.update_view_map()

    def timer_update_view_objects(self):
        self.viewObjects.plotItem.clear()
        self.viewObjects.showGrid(x=True, y=True)

        # scatter objects
        if self.checkboxViewObstacles.isChecked():
            if self.rosHmi is not None and self.rosHmi.msgFusion is not None:
                all_obss = np.array(self.rosHmi.msgFusion.obstacledata).reshape(-1, 9)

                # points
                if self.checkboxViewPoints.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 0]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(65, 105, 225, 153)),
                            symbol='o'
                        )

                # cars
                if self.checkboxViewCars.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 1]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(0, 139, 0, 153)),
                            symbol='o'
                        )

                # trucks
                if self.checkboxViewTrucks.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 2]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(255, 193, 37, 153)),
                            symbol='o'
                        )

                # pedestrians
                if self.checkboxViewPedestrians.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 3]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(178, 34, 34, 153)),
                            symbol='o'
                        )

                # motorcycles
                if self.checkboxViewMotorcycles.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 4]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(255, 0, 255, 153)),
                            symbol='o'
                        )

                # cycles
                if self.checkboxViewCycles.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 5]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(0, 191, 255, 153)),
                            symbol='o'
                        )

                # walls
                if self.checkboxViewWalls.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 6]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(130, 130, 130, 153)),
                            symbol='o'
                        )

                # reserved
                if self.checkboxViewReserved.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 7]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(238, 220, 130, 153)),
                            symbol='o'
                        )

                # traffic lights
                if self.checkboxViewTrafficlights.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 8]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(127, 255, 0, 153)),
                            symbol='o'
                        )

                # traffic signs
                if self.checkboxViewTrafficsigns.isChecked():
                    o_ = all_obss[all_obss[:, 0] == 9]
                    if len(o_) > 0:
                        self.viewObjects.plotItem.scatterPlot(
                            o_[:, 4], o_[:, 5],
                            size=10,
                            brush=pg.mkBrush(color=(160, 32, 240, 153)),
                            symbol='o'
                        )

        # plot local planning
        if self.checkboxViewTrajectory.isChecked():
            if self.rosHmi is not None and self.rosHmi.msgFusion is not None and self.rosHmi.msgLocalPathPlanning is not None:
                if len(self.rosHmi.msgLocalPathPlanning.longitude) > 0:
                    lons_ = np.array(self.rosHmi.msgLocalPathPlanning.longitude, dtype=np.float64)
                    lats_ = np.array(self.rosHmi.msgLocalPathPlanning.latitude, dtype=np.float64)

                    gps_path = np.array([lons_, lats_, np.zeros_like(lons_)], dtype=np.float64).swapaxes(0, 1)
                    gps_ego = np.array([self.rosHmi.msgFusion.longitude, self.rosHmi.msgFusion.latitude, 0],
                                       dtype=np.float64)
                    rot_ego = np.array([0, 0, self.rosHmi.msgFusion.yaw])

                    xyz_path = tjitools.gps_to_rfu(gps_path, gps_ego, rot_ego)

                    self.viewObjects.plotItem.plot(
                        xyz_path[:, 0], xyz_path[:, 1],
                        pen=pg.mkPen(color=(69, 139, 0), width=8)
                    )

        # scatter ego vehicle
        self.viewObjects.plotItem.scatterPlot(
            [0], [0],
            size=20,
            brush=pg.mkBrush(color=(238, 99, 99)),
            symbol='t1'
        )

        if self.rosHmi is not None and self.rosHmi.msgFusion is not None and self.rosHmi.msgLocalPathPlanning is not None:
            v_ref = self.rosHmi.msgLocalPathPlanning.speed[10] * 3.6
            v_ego = self.rosHmi.msgFusion.carspeed * 3.6
            t_ = pg.TextItem("Vr= %.1f km/h\nVe= %.1f km/h" % (v_ref, v_ego), color=QColor(0, 0, 0))
        else:
            t_ = pg.TextItem("Vr= ? km/h\nVe= ? km/h", color=QColor(0, 0, 0))
        t_.setPos(1.0, 1.0)
        self.viewObjects.plotItem.addItem(t_)

        ticks = [[np.float32(k), str(k)] for k in range(-20, 41, 5)]
        self.viewObjects.plotItem.getAxis('left').setTicks([ticks])
        self.viewObjects.setYRange(-20.0, 40.0)

        ticks = [[np.float32(k), str(k)] for k in range(-15, 16, 5)]
        self.viewObjects.plotItem.getAxis('bottom').setTicks([ticks])
        self.viewObjects.setXRange(-15.0, 15.0)

    def timer_update_Cloud_info(self):
        '''if self.rosHmi.msgFusion is not None:
            soc_ = self.rosHmi.msgFusion.soc
            thr_ = self.rosHmi.msgFusion.throttle_percentage
            brk_ = self.rosHmi.msgFusion.braking_percentage
            str_ = self.rosHmi.msgFusion.steerangle
            ger_ = self.rosHmi.msgFusion.gearpos
            mod_ = self.rosHmi.msgFusion.car_run_mode
            lon_ = self.rosHmi.msgFusion.longitude
            lat_ = self.rosHmi.msgFusion.latitude
            hda_ = self.rosHmi.msgFusion.yaw
            vel_ = self.rosHmi.msgFusion.carspeed * 3.6'''
        nowMs = self.now_ms()
        if not self.stateCloud:
            return

        # update the vehicle state
        if self.rosHmi.msgCloud_1 is not None or self.rosHmi.msgCloud_2 is not None:
            # soc_ = self.rosHmi.msgCloud.soc
            # thr_ = self.rosHmi.msgCloud.throttle_percentage
            # brk_ = self.rosHmi.msgCloud.braking_percentage
            # str_ = self.rosHmi.msgCloud.steerangle
            # ger_ = self.rosHmi.msgCloud.gearpos
            # mod_ = self.rosHmi.msgCloud.car_run_mode
            lon_1 = self.rosHmi.msgCloud_1.lon
            lat_1 = self.rosHmi.msgCloud_1.lat
            hda_1 = self.rosHmi.msgCloud_1.yaw
            vel_1 = self.rosHmi.msgCloud_1.car_speed * 3.6

            self.Cloud_dispLongitude_1.setText(str(round(lon_1, 11)))
            self.Cloud_dispLatitude_1.setText(str(round(lat_1, 11)))
            self.Cloud_dispHeading_1.setText(str(round(hda_1, 2)))
            self.Cloud_dispSpeed_1.setText(str(round(vel_1, 2)))

            lon_2 = self.rosHmi.msgCloud_2.lon
            lat_2 = self.rosHmi.msgCloud_2.lat
            hda_2 = self.rosHmi.msgCloud_2.yaw
            vel_2 = self.rosHmi.msgCloud_2.car_speed * 3.6

            self.Cloud_dispLongitude_2.setText(str(round(lon_2, 11)))
            self.Cloud_dispLatitude_2.setText(str(round(lat_2, 11)))
            self.Cloud_dispHeading_2.setText(str(round(hda_2, 2)))
            self.Cloud_dispSpeed_2.setText(str(round(vel_2, 2)))
            
            car_id_1 = self.rosHmi.msgCloud_1.car_id
            car_id_2 = self.rosHmi.msgCloud_2.car_id
            
            self.Cloud_Car_Num_1.setText(str(car_id_1))
            self.Cloud_Car_Num_2.setText(str(car_id_2))
            
    def thread_ros_hmi(self):
        rclpy.spin(self.rosHmi)
        self.rosHmi.destroy_node()
        rclpy.shutdown()

    def thread_orin_hardware(self):
        while True:
            if self.stateConnected:
                # cpu
                self.percentCpu = psutil.cpu_percent(interval=1, percpu=False)

                # gpu (using jtop)
                self.percentGpu = 0.0
                try:
                    with jtop() as jetson:
                        if jetson.ok():
                            self.percentGpu = jetson.gpu['val']
                except:
                    pass

                # ram
                self.percentRam = psutil.virtual_memory().percent

                # disk
                disk_ = psutil.disk_usage("/")
                self.percentDisk = disk_.used / disk_.total * 100.0

            time.sleep(1)

    def thread_recorde_route(self):
        self.recorderRoute = []
        while self.stateRouteRecorder:
            time.sleep(0.1)

            if self.rosHmi is None or self.rosHmi.msgFusion is None:
                continue

            lon_ = round(self.rosHmi.msgFusion.longitude, 8)
            lat_ = round(self.rosHmi.msgFusion.latitude, 8)
            alt_ = round(self.rosHmi.msgFusion.height, 2)
            hed_ = round(self.rosHmi.msgFusion.yaw, 2)
            vel_ = round(self.rosHmi.msgFusion.carspeed, 2)

            if len(self.recorderRoute) == 0:
                d_ = np.array([lon_, lat_, alt_, hed_, vel_], dtype=np.float64)
                self.recorderRoute.append(d_)
            else:
                dlon_ = lon_ - self.recorderRoute[-1][0]
                dlat_ = lat_ - self.recorderRoute[-1][1]
                if abs(dlon_) >= 1e-6 or abs(dlat_) >= 1e-6:
                    d_ = np.array([lon_, lat_, alt_, hed_, vel_], dtype=np.float64)
                    self.recorderRoute.append(d_)

    def thread_update_global_path(self):
        self.trackerGlobalPath = None
        cnt_ = 0
        while True:
            cnt_ += 1
            if self.rosHmi.msgGlobalPathPlanning is not None:
                err_start = self.trackerStartGps - np.array(self.rosHmi.msgGlobalPathPlanning.startpoint,
                                                            dtype=np.float64)
                err_end = self.trackerEndGps - np.array(self.rosHmi.msgGlobalPathPlanning.endpoint, dtype=np.float64)
                if np.max(np.abs(err_start)) < 5e-5 and np.max(np.abs(err_end)) < 5e-5:
                    self.trackerGlobalPath = np.array(self.rosHmi.msgGlobalPathPlanning.routedata,
                                                      dtype=np.float64).reshape(-1, 4)
                    break

            if cnt_ >= 10:
                self.rosHmi.get_logger().info("not found !!!! ")
                break

            time.sleep(0.5)

    # actions on main page
    def action_button_connect_car(self):

        # launch default ros nodes
        proc = QProcess(self)
        proc.start('ros2 launch ./launch/launch_defaults.py')

        self.carWidget.setEnabled(True)
        self.tabWidget.setEnabled(True)
        self.stateConnected = True
        self.stateSensorCan = True
        self.stateSensorGps = True
        self.stateSensorImu = True
        self.stateSensorSonic = True
        self.figCar.setStyleSheet("image: url(\'./src/hmi/hmi/figs/car_connected.png\');")
        self.buttonConnectCar.setEnabled(False)
        time.sleep(1)

        # only for test
        self.buttonTracker.setEnabled(True)

    def action_button_cloud(self):
        if not self.stateCloud:
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_cloud.py')
            self.stateCloud = True
            self.enable_indicator(self.indicatorCloud)
            self.label_cloud_1.setEnabled(True)
            self.label_cloud_2.setEnabled(True)
            self.label_cloud_3.setEnabled(True)
            self.label_cloud_4.setEnabled(True)
            self.label_cloud_5.setEnabled(True)
            self.label_cloud_6.setEnabled(True)
            self.label_cloud_7.setEnabled(True)
            self.label_cloud_8.setEnabled(True)
            self.label_cloud_9.setEnabled(True)
            self.label_cloud_10.setEnabled(True)
            self.label_cloud_11.setEnabled(True)
            self.label_cloud_12.setEnabled(True)
            self.label_cloud_13.setEnabled(True)
            self.label_cloud_14.setEnabled(True)
            self.label_cloud_15.setEnabled(True)
            self.label_cloud_16.setEnabled(True)
            self.Cloud_dispLongitude_1.setEnabled(True)
            self.Cloud_dispLatitude_1.setEnabled(True)
            self.Cloud_dispHeading_1.setEnabled(True)
            self.Cloud_dispSpeed_1.setEnabled(True)
            self.Cloud_dispLongitude_2.setEnabled(True)
            self.Cloud_dispLatitude_2.setEnabled(True)
            self.Cloud_dispHeading_2.setEnabled(True)
            self.Cloud_dispSpeed_2.setEnabled(True)
            self.Cloud_Car_Num_1.setEnabled(True)
            self.Cloud_Car_Num_2.setEnabled(True)
            self.Cloud_figCar_1.setStyleSheet("image: url(\'./src/hmi/hmi/figs/car_connected.png\');")
            self.Cloud_figCar_2.setStyleSheet("image: url(\'./src/hmi/hmi/figs/car_connected.png\');")
        else:
            self.kill_process('net_work')
            self.stateCloud = False
            self.disable_indicator(self.indicatorCloud)
            self.label_cloud_1.setEnabled(False)
            self.label_cloud_2.setEnabled(False)
            self.label_cloud_3.setEnabled(False)
            self.label_cloud_4.setEnabled(False)
            self.label_cloud_5.setEnabled(False)
            self.label_cloud_6.setEnabled(False)
            self.label_cloud_7.setEnabled(False)
            self.label_cloud_8.setEnabled(False)
            self.label_cloud_9.setEnabled(False)
            self.label_cloud_10.setEnabled(False)
            self.label_cloud_11.setEnabled(False)
            self.label_cloud_12.setEnabled(False)
            self.label_cloud_13.setEnabled(False)
            self.label_cloud_14.setEnabled(False)
            self.label_cloud_15.setEnabled(False)
            self.label_cloud_16.setEnabled(False)
            self.Cloud_dispLongitude_1.setEnabled(False)
            self.Cloud_dispLatitude_1.setEnabled(False)
            self.Cloud_dispHeading_1.setEnabled(False)
            self.Cloud_dispSpeed_1.setEnabled(False)
            self.Cloud_dispLongitude_2.setEnabled(False)
            self.Cloud_dispLatitude_2.setEnabled(False)
            self.Cloud_dispHeading_2.setEnabled(False)
            self.Cloud_dispSpeed_2.setEnabled(False)
            self.Cloud_Car_Num_1.setEnabled(False)
            self.Cloud_Car_Num_2.setEnabled(False)

            self.Cloud_figCar_1.setStyleSheet("image: url(\'./src/hmi/hmi/figs/car_disconnected.png\');")
            self.Cloud_figCar_2.setStyleSheet("image: url(\'./src/hmi/hmi/figs/car_disconnected.png\');")
        time.sleep(0.5)
    def action_button_open_rviz(self):
        if not self.check_process('rviz2'):
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_rviz.py')
        pass

    # actions on tab_sensor
    def action_button_camera(self):
        nowMs = self.now_ms()
        if self.stateSensorCamera == 0:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_camera.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Camera 打开失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('camera_ori'):
                    break
                time.sleep(0.1)

            self.buttonCameraObject.setEnabled(True)
            self.buttonCameraLane.setEnabled(True)
            self.buttonCameraLight.setEnabled(True)
            self.buttonCameraSign.setEnabled(True)
            self.dispSensorInfo.append("[INFO] [%s] Camera 已打开 !!!" % nowMs)
            self.buttonCamera.setText("Close Camera")
            self.stateSensorCamera = True
        else:

            # kill nodes
            for pn_ in ['camera_ori', 'camera_detect', 'lane_recognition', 'sign_recognition', 'light_recognition']:
                self.kill_process(pn_)
            self.rosHmi.msgCameraState = None
            self.buttonCameraObject.setEnabled(False)
            self.buttonCameraLane.setEnabled(False)
            self.buttonCameraLight.setEnabled(False)
            self.buttonCameraSign.setEnabled(False)
            self.disable_indicator(self.indicatorCameraObject)
            self.disable_indicator(self.indicatorCameraLane)
            self.disable_indicator(self.indicatorCameraLight)
            self.disable_indicator(self.indicatorCameraSign)
            self.dispSensorInfo.append("[INFO] [%s] Camera 已关闭 !!!" % nowMs)
            self.buttonCamera.setText("Open Camera")
            self.stateSensorCamera = False
            self.stateCameraObject = False
            self.stateCameraLane = False
            self.stateCameraLight = False
            self.stateCameraSign = False

    def action_button_camera_object(self):
        nowMs = self.now_ms()
        if not self.stateCameraObject:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            self.kill_process('camera_ori')
            proc.start('ros2 launch ./launch/launch_camera_object.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Camera-目标检测 启动失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('camera_detect'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Camera-目标检测 已启动 !!!" % nowMs)
            self.enable_indicator(self.indicatorCameraObject)
            self.stateCameraObject = True
        else:
            # kill nodes
            self.kill_process('camera_detect')
            self.dispSensorInfo.append("[INFO] [%s] Camera-目标检测 已停止 !!!" % nowMs)
            self.disable_indicator(self.indicatorCameraObject)
            self.stateCameraObject = False
        pass

    def action_button_camera_lane(self):
        nowMs = self.now_ms()
        if not self.stateCameraLane:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_camera_lane.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Camera-车道线检测 启动失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('lane_recognition'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Camera-车道线检测 已启动 !!!" % nowMs)
            self.enable_indicator(self.indicatorCameraLane)
            self.stateCameraLane = True
        else:
            # kill nodes
            self.kill_process('lane_recognition')
            self.dispSensorInfo.append("[INFO] [%s] Camera-车道线检测 已停止 !!!" % nowMs)
            self.disable_indicator(self.indicatorCameraLane)
            self.stateCameraLane = False
        pass

    def action_button_camera_light(self):
        nowMs = self.now_ms()
        if not self.stateCameraLight:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_camera_light.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Camera-信号灯检测 启动失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('light_recognition'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Camera-信号灯检测 已启动 !!!" % nowMs)
            self.enable_indicator(self.indicatorCameraLight)
            self.stateCameraLight = True
        else:
            # kill nodes
            self.kill_process('lane_recognition')
            self.dispSensorInfo.append("[INFO] [%s] Camera-信号灯检测 已停止 !!!" % nowMs)
            self.disable_indicator(self.indicatorCameraLight)
            self.stateCameraLight = False
        pass

    def action_button_camera_sign(self):
        nowMs = self.now_ms()
        if not self.stateCameraSign:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_camera_sign.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Camera-标识牌检测 启动失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('sign_recognition'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Camera-标识牌检测 已启动 !!!" % nowMs)
            self.enable_indicator(self.indicatorCameraSign)
            self.stateCameraSign = True
        else:
            # kill nodes
            self.kill_process('sign_recognition')
            self.dispSensorInfo.append("[INFO] [%s] Camera-标识牌检测 已停止 !!!" % nowMs)
            self.disable_indicator(self.indicatorCameraSign)
            self.stateCameraSign = False
        pass

    def action_button_lidar(self):
        nowMs = self.now_ms()
        if not self.stateSensorLidar:
            # try to open the lidar, wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_lidar.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Liar 打开失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('rslidar_sdk_node'):
                    break
                time.sleep(0.1)

            self.buttonLidarObject.setEnabled(True)
            self.dispSensorInfo.append("[INFO] [%s] Lidar 已打开 !!!" % nowMs)
            self.buttonLidar.setText("Close Lidar")
            self.stateSensorLidar = True
        else:
            # kill processes
            for pn_ in ['right_lidar_obstacle', 'left_lidar_obstacle','lidar_obstacle', 'rslidar_sdk_node']:
                self.kill_process(pn_)
            self.rosHmi.msgLidarState = None
            self.buttonLidarObject.setEnabled(False)
            self.disable_indicator(self.indicatorLidarObject)
            self.dispSensorInfo.append("[INFO] [%s] Lidar 已关闭 !!!" % nowMs)
            self.buttonLidar.setText("Open Lidar")
            self.stateSensorLidar = False
            self.stateLidarObject = False
        pass

    def action_button_lidar_object(self):
        nowMs = self.now_ms()
        if not self.stateLidarObject:
            # try to open nodes & wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_lidar_object.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Lidar-目标检测 启动失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('lidar_obstacle'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Lidar-目标检测 已启动 !!!" % nowMs)
            self.enable_indicator(self.indicatorLidarObject)
            self.stateLidarObject = True
        else:
            # kill nodes
            for pn_ in ['right_lidar_obstacle', 'left_lidar_obstacle','lidar_obstacle']:
                self.kill_process(pn_)
            self.dispSensorInfo.append("[INFO] [%s] Lidar-目标检测 已停止 !!!" % nowMs)
            self.disable_indicator(self.indicatorLidarObject)
            self.stateLidarObject = False
        pass

    def action_button_radar(self):
        nowMs = self.now_ms()
        if not self.stateSensorRadar:
            # try to open the radar, wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_radar.py')
            for cnt_ in range(50, -1, -1):
                if cnt_ == 0:  # time out
                    self.dispSensorInfo.append("[ERROR] [%s] Radar 打开失败, 请重新尝试 !!!" % nowMs)
                    return
                if self.check_process('radar_obstacle'):
                    break
                time.sleep(0.1)

            self.dispSensorInfo.append("[INFO] [%s] Radar 已打开 !!!" % nowMs)
            self.buttonRadar.setText("Close Radar")
            self.stateSensorRadar = True
        else:
            # kill processes
            for pn_ in ['radar_obstacle']:
                self.kill_process(pn_)
            self.rosHmi.msgRadarState = None
            self.dispSensorInfo.append("[INFO] [%s] Radar 已关闭 !!!" % nowMs)
            self.buttonRadar.setText("Open Radar")
            self.stateSensorRadar = False
        pass

    # actions on tab_tracker
    def action_button_load_map(self):
        nowMs = self.now_ms()

        if self.checkboxUseTrajectory.isChecked():
            file_path, _ = QFileDialog.getOpenFileName(self, "open file", filter='*.txt')
            if file_path == "":
                return
            try:
                self.trackerTrajectory = np.loadtxt(file_path, dtype=np.float64, delimiter=',', encoding='utf-8')
                self.trackerRoadMap = None
                self.trackerFilePath = file_path
                self.trackerUseTrajectory = True
            except:
                QMessageBox.information(self, "Error", "轨迹文件损坏 !!!")
                return

        else:
            file_path, _ = QFileDialog.getOpenFileName(self, "open file", filter='*.json')
            if file_path == "":
                return
            try:
                with open(file_path, mode='r', encoding='utf-8') as f_:
                    self.trackerRoadMap = roadnet.RoadNet()
                    self.trackerRoadMap.from_dict(json.load(f_))
                self.trackerTrajectory = None
                self.trackerFilePath = file_path
                self.trackerUseTrajectory = False
            except:
                QMessageBox.information(self, "Error", "路网文件损坏 !!!")
                return

        self.stateGlobalPlanner = False
        self.buttonGlobalPlanner.setEnabled(True)
        self.disable_indicator(self.indicatorGlobalPlanner)
        self.trackerStartGps = None
        self.trackerEndGps = None
        self.trackerGlobalPath = None

        self.comboDestination.clear()
        self.comboDestination.setEnabled(False)
        self.buttonReplan.setEnabled(False)
        self.buttonReplan.setText('Plan')

        self.stateTracker = False
        self.buttonTracker.setEnabled(True)
        self.disable_indicator(self.indicatorTracker)

        self.dispTrackerInfo.append("[INFO] [%s] 地图已加载 !!!" % nowMs)
        self.update_view_map()

    def action_button_global_planner(self):
        if self.buttonGlobalPlanner.text() == 'Start Planner':

            if not self.check_process('global_path_planning'):
                nowMs = self.now_ms()
                proc = QProcess(self)
                proc.start('ros2 launch ./launch/launch_global_planner.py use_trajectory:=%s file_path:=%s' % (
                str(self.trackerUseTrajectory), self.trackerFilePath))
                for cnt_ in range(20, -1, -1):
                    if cnt_ == 0:  # time out
                        self.dispSensorInfo.append("[ERROR] [%s] Global Planner 打开失败, 请重新尝试 !!!" % nowMs)
                        self.stateGlobalPlanner = False
                        return
                    if self.checks_process('global_path_planning'):
                        break
                    time.sleep(0.1)
                self.dispTrackerInfo.append("[INFO] [%s] Global Planner 已启动 !!!" % (nowMs))
                self.rosHmi.get_logger().info(
                    'use_trajectory = %s, file = %s' % (str(self.trackerUseTrajectory), self.trackerFilePath))

            self.checkboxUseTrajectory.setEnabled(False)
            self.buttonLoadMap.setEnabled(False)

            self.stateGlobalPlanner = True
            self.buttonGlobalPlanner.setText('Stop Planner')
            self.enable_indicator(self.indicatorGlobalPlanner)

            if self.trackerUseTrajectory:
                self.comboDestination.clear()
                self.comboDestination.setEnabled(False)
            else:
                site_list = [st.name for st in self.trackerRoadMap.all_sites]
                self.comboDestination.clear()
                self.comboDestination.addItems(site_list)
                self.comboDestination.setCurrentIndex(-1)
                self.comboDestination.setEnabled(True)

            self.buttonReplan.setEnabled(True)

        else:
            for pn_ in ['global_path_planning']:
                self.kill_process(pn_)

            self.checkboxUseTrajectory.setEnabled(True)
            self.buttonLoadMap.setEnabled(True)

            self.stateGlobalPlanner = False
            self.buttonGlobalPlanner.setText('Start Planner')
            self.disable_indicator(self.indicatorGlobalPlanner)

            self.comboDestination.clear()
            self.comboDestination.setEnabled(False)

            self.buttonReplan.setEnabled(False)
            self.buttonReplan.setText('Plan')

            self.trackerStartGps = None
            self.trackerEndGps = None
            self.trackerGlobalPath = None

    def action_combo_destination(self):
        self.update_view_map()

    def action_button_global_replan(self):
        nowMs = self.now_ms()
        self.buttonReplan.setText('Replan')

        if self.trackerUseTrajectory:
            trackerEndGps = self.trackerTrajectory[-1, 0:2]
        else:
            end_idx = self.comboDestination.currentIndex()
            if end_idx == -1:
                QMessageBox.information(self, "Error", "请选择目标点 !!!")
                return
            else:
                st = self.trackerRoadMap.all_sites[end_idx]
                trackerEndGps = np.array([st.loc.longitude, st.loc.latitude], dtype=np.float64)

        trackerStartGps = np.array([self.rosHmi.msgFusion.longitude, self.rosHmi.msgFusion.latitude], dtype=np.float64)

        if abs(trackerStartGps[0] - trackerEndGps[0]) < 1e-5 and abs(trackerStartGps[1] - trackerEndGps[1]) < 1e-5:
            QMessageBox.information(self, "Info", "已到达目标点 !!!")
            return

        self.trackerStartGps = trackerStartGps
        self.trackerEndGps = trackerEndGps
        self.rosHmi.gpsStartPoint = trackerStartGps
        self.rosHmi.gpsEndPoint = trackerEndGps

        self.dispTrackerInfo.append(
            "[INFO] [%s] 全局起始点: lon=%.5f, lat=%.5f" % (nowMs, self.trackerStartGps[0], self.trackerStartGps[1]))
        self.dispTrackerInfo.append(
            "[INFO] [%s] 全局目标点: lon=%.5f, lat=%.5f" % (nowMs, self.trackerEndGps[0], self.trackerEndGps[1]))

        t_ = threading.Thread(target=self.thread_update_global_path, args=())
        t_.setDaemon(True)
        t_.start()

    def action_button_tracker(self):
        nowMs = self.now_ms()
        if not self.stateTracker:
            # check if the global planner has started
            if not self.stateGlobalPlanner:
                QMessageBox.information(self, "Error", "请启动规划器 !!!")
                return

            # check if the path is empty
            if self.trackerGlobalPath is None:
                QMessageBox.information(self, "Error", "未找到全局轨迹 !!!")
                return

            # try to start the tracker, wait for 5 seconds
            proc = QProcess(self)
            proc.start('ros2 launch ./launch/launch_tracker.py')

            self.buttonGlobalPlanner.setEnabled(False)
            self.comboDestination.setEnabled(False)
            self.buttonReplan.setEnabled(False)

            self.stateTracker = True
            self.buttonTracker.setText("Stop Tracker")
            self.enable_indicator(self.indicatorTracker)
            self.dispTrackerInfo.append("[INFO] [%s] 路径跟踪已启动 !!!" % nowMs)

            self.buttonParking.setEnabled(False)

        else:
            # kill processes
            for pn_ in ['car_decision', 'pid', 'car_control', 'local_path_planning']:
                self.kill_process(pn_)

            self.buttonGlobalPlanner.setEnabled(True)
            self.comboDestination.setEnabled(True)
            self.buttonReplan.setEnabled(True)

            self.stateTracker = False
            self.buttonTracker.setText("Start Tracker")
            self.disable_indicator(self.indicatorTracker)
            self.dispTrackerInfo.append("[INFO] [%s] 路径跟踪已停止 !!!" % nowMs)

            self.buttonParking.setEnabled(True)

    # action on tab_recorder
    def action_button_recorder_point_add(self):
        if self.rosHmi is None or self.rosHmi.msgFusion is None:
            return

        new_pt = {}
        new_pt["name"] = "new"
        new_pt["longitude"] = round(self.rosHmi.msgFusion.longitude, 8)
        new_pt["latitude"] = round(self.rosHmi.msgFusion.latitude, 8)
        new_pt["altitude"] = round(self.rosHmi.msgFusion.height, 2)
        new_pt["heading"] = round(self.rosHmi.msgFusion.yaw, 2)

        self.recorderPoints.append(new_pt)
        self.update_list_reorder_points()

    def action_button_recorder_point_delete(self):
        idx_ = self.listRecorderPoints.currentIndex().row()
        if idx_ == -1:
            return
        else:
            self.recorderPoints.pop(idx_)
            self.update_list_reorder_points()

    def action_button_recorder_point_edit(self):
        idx_ = self.listRecorderPoints.currentIndex().row()
        if idx_ == -1:
            return
        else:
            if self.buttonPointRecorder_Edit.text() == "Edit":
                self.widgetRecorderInfo.setEnabled(True)
                self.buttonPointRecorder_Edit.setText("Save")
            else:
                pt = {}
                pt["name"] = self.inputRecorderName.text()
                pt["longitude"] = float(self.inputRecorderLon.text())
                pt["latitude"] = float(self.inputRecorderLat.text())
                pt["altitude"] = float(self.inputRecorderAlt.text())
                pt["heading"] = float(self.inputRecorderHed.text())
                self.recorderPoints[idx_] = copy.deepcopy(pt)

                self.widgetRecorderInfo.setEnabled(False)
                self.buttonPointRecorder_Edit.setText("Edit")
                self.update_list_reorder_points()

    def action_button_recorder_point_savefile(self):
        # 弹出输入框获取文件名
        file_name, ok = QInputDialog.getText(self, "保存文件", "输入文件名:")
        if not ok or file_name.strip() == "":
            QMessageBox.warning(self, "输入错误", "文件名不能为空!")
            return

        # 指定固定文件夹路径
        directory_path = '/home/nvidia/AutoDrive/maps'

        # 拼接完整的文件路径

        file_path = os.path.join(directory_path, f"{file_name}_sites.txt")

        # 保存文件
        try:
            with open(file_path, mode='w', encoding='utf-8') as f_:
                for pt in self.recorderPoints:
                    s_ = ",".join([
                        pt["name"],
                        str(pt["longitude"]),
                        str(pt["latitude"]),
                        str(pt["altitude"]),
                        str(pt["heading"])
                    ])
                    f_.write(s_ + '\n')
            QMessageBox.information(self, "保存成功", f"文件已保存至: {file_path}")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"保存文件时出错: {e}")

    def action_list_recorder_point(self):
        idx_ = self.listRecorderPoints.currentIndex().row()
        if idx_ == -1:
            self.inputRecorderName.clear()
            self.inputRecorderLon.clear()
            self.inputRecorderLat.clear()
            self.inputRecorderAlt.clear()
            self.inputRecorderHed.clear()
        else:
            pt = self.recorderPoints[idx_]
            self.inputRecorderName.setText(pt["name"])
            self.inputRecorderLon.setText(str(pt["longitude"]))
            self.inputRecorderLat.setText(str(pt["latitude"]))
            self.inputRecorderAlt.setText(str(pt["altitude"]))
            self.inputRecorderHed.setText(str(pt["heading"]))

        self.widgetRecorderInfo.setEnabled(False)
        self.buttonPointRecorder_Edit.setText("Edit")

    def action_button_recorder_route(self):
        nowMs = self.now_ms()
        if not self.stateRouteRecorder:
            self.enable_indicator(self.indicatorRouteRecoder)
            self.dispRecorderInfo.append("[INFO] [%s] 录制轨迹 !!!" % nowMs)
            self.buttonRouteRecorder.setText("Stop Recording")
            self.stateRouteRecorder = True

            # create recording threading
            t_ = threading.Thread(target=self.thread_recorde_route, args=())
            t_.setDaemon(True)
            t_.start()
        else:
            self.disable_indicator(self.indicatorRouteRecoder)
            self.dispRecorderInfo.append("[INFO] [%s] 录制结束 !!!" % nowMs)
            self.buttonRouteRecorder.setText("Record Route")
            self.stateRouteRecorder = False

            # save route
            # 弹出输入框获取文件名
            file_name, ok = QInputDialog.getText(self, "save file", "输入文件名:")
            if not ok or file_name.strip() == "":
                QMessageBox.warning(self, "输入错误", "文件名不能为空!")
                return

                # 指定固定文件夹路径
            directory_path = '/home/nvidia/AutoDrive/maps'
            file_path = os.path.join(directory_path, f"{file_name}_trj.txt")

            # 保存文件
            try:
                np.savetxt(file_path, np.array(self.recorderRoute, dtype=np.float64),
                           fmt=['%.8f', '%.8f', '%.2f', '%.2f', '%.2f'], delimiter=',')
                QMessageBox.information(self, "保存成功", f"文件已保存至: {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "保存失败", f"保存文件时出错: {e}")

    def action_convert_txt_to_json(self):
        path_high_speed = 10.0 # 10 km/h

        # 弹出输入框获取地图名称
        map_name, ok = QInputDialog.getText(self, "输入地图名称", "请输入地图名称:")

        if not ok or not map_name:
            return

        # 指定地图文件夹路径
        map_folder = '/home/nvidia/AutoDrive/maps'  # 请根据实际路径修改
        site_file = os.path.join(map_folder, map_name + '_sites.txt')
        trj_file = os.path.join(map_folder, map_name + '_trj.txt')

        if not os.path.exists(site_file) or not os.path.exists(trj_file):
            self.show_message_box("Error", "无法找到指定地图的资源文件。")
            return

        my_roadnet = roadnet.RoadNet()

        # 读取站点文件
        with open(site_file, mode='r', encoding='utf-8') as fr_:
            for s_ in fr_:
                data_ = s_.strip("\n").split(',')
                st = roadnet.RoadNet_Site(
                    name_=data_[0],
                    lon_=np.float64(data_[1]),
                    lat_=np.float64(data_[2]),
                    alt_=np.float64(data_[3])
                )
                my_roadnet.all_sites.append(st)

        # 读取轨迹文件
        trj = np.loadtxt(trj_file, dtype=np.float64, delimiter=',', encoding='utf-8')

        print("已加载所有轨迹文件！")

        def uag180(a):
            while a > 180:
                a -= 360
            while a < -180:
                a += 360
            return a

        all_wps = []
        trj = trj[:, 0:4]
        st_lon, st_lat, st_alt, st_hed = trj[0]
        st_id = None
        for i in range(len(all_wps)):
            lon_, lat_, alt_, hed_ = all_wps[i]
            if (abs(lon_ - st_lon) < 1e-4 and
                    abs(lat_ - st_lat) < 1e-4 and
                    abs(alt_ - st_alt) < 1.0 and
                    abs(uag180(hed_ - st_hed)) < 15.0):
                st_id = i
                break
        if st_id is None:
            wp = np.array([st_lon, st_lat, st_alt, st_hed])
            all_wps.append(wp)
            st_id = len(all_wps) - 1

        ed_lon, ed_lat, ed_alt, ed_hed = trj[-1]
        ed_id = None
        for i in range(len(all_wps)):
            lon_, lat_, alt_, hed_ = all_wps[i]
            if (abs(lon_ - ed_lon) < 1e-4 and
                    abs(lat_ - ed_lat) < 1e-4 and
                    abs(alt_ - ed_alt) < 1.0 and
                    abs(uag180(hed_ - ed_hed)) < 15.0):
                ed_id = i
                break
        if ed_id is None:
            wp = np.array([ed_lon, ed_lat, ed_alt, ed_hed])
            all_wps.append(wp)
            ed_id = len(all_wps) - 1

        ph = roadnet.RoadNet_Path(
            name_='-'.join([str(st_id + 1), str(ed_id + 1)]),
            start_=str(st_id + 1),
            end_=str(ed_id + 1),
            highspeed_=path_high_speed,
            lowspeed_=0,
            lons_=trj[:, 0],
            lats_=trj[:, 1],
            alts_=trj[:, 2],
            heds_=trj[:, 3]
        )
        my_roadnet.all_paths.append(ph)

        print("已处理所有轨迹信息！")

        for i, wp_data in enumerate(all_wps):
            lon_, lat_, alt_, hed_ = wp_data
            wp = roadnet.RoadNet_Waypoint(
                name_=str(i + 1),
                lon_=lon_,
                lat_=lat_,
                alt_=alt_,
                hed_=hed_
            )
            my_roadnet.all_waypoints.append(wp)

        # 保存为JSON
        file_path, _ = QFileDialog.getSaveFileName(self, "另存为JSON", filter='*.json')
        if file_path != "":
            with open(file_path + '.json', mode="w", encoding="utf-8") as fw_:
                json.dump(my_roadnet.to_dict(), fw_, ensure_ascii=False, indent=4)
        
        # 保存文件
        try:
            np.savetxt(file_path, np.array(self.recorderRoute, dtype=np.float64),
                        fmt=['%.8f', '%.8f', '%.2f', '%.2f', '%.2f'], delimiter=',')
            QMessageBox.information(self, "保存成功", f"文件已保存至: {file_path}")
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"保存文件时出错: {e}")

    # 编队下拉选项
    def platoon_choose(self):
        platoon_id = self.comboBoxPlatoon.currentText()
        if platoon_id:
            self.buttonPlatoon.setEnabled(True)  # 启用按钮
            self.label_platoon.setText("Self-car_id is " + str(platoon_id))
        elif not self.stateplatoon:  # 没有选择时，禁用按钮
            self.buttonPlatoon.setEnabled(False)

    def toggle_platoon_mode(self):
        # 切换多车编队状态
        platoon_id = self.comboBoxPlatoon.currentText()
        if not self.stateplatoon:
            self.stateplatoon = True
            self.rosHmi.PlatoonID = int(platoon_id)
            self.rosHmi.PlatoonState = 1
            self.comboBoxPlatoon.setEnabled(False)
            self.comboBoxPlatoon.clear()
            self.enable_indicator(self.indicatorPlatoon)
        else:
            self.stateplatoon = False
            self.rosHmi.PlatoonID = None
            self.rosHmi.PlatoonState = None
            self.disable_indicator(self.indicatorPlatoon)
            self.comboBoxPlatoon.addItems(["", "1", "2", "3"])
            self.comboBoxPlatoon.setEnabled(True)
            self.buttonPlatoon.setEnabled(False)
            self.label_platoon.setText("Please choose self-car_id first!")

    def action_button_parking(self):
        if not self.stateparking:
            self.stateparking = True
            # self.buttonParking1.setEnabled(True)
            # self.buttonParking2.setEnabled(True)
            # self.buttonParking3.setEnabled(True)
            # launch default ros nodes
            proc1 = QProcess(self)
            proc2 = QProcess(self)
            proc3 = QProcess(self)
            proc1.start('ros2 run parking_planning parking_planning')
            if not self.check_process('parking_planning'):
                proc1.start('ros2 run parking_planning parking_planning')
                time.sleep(1)
            self.label_parking_1.setText("Start parking_planning success!")
            if not self.check_process('park_control'):
                proc2.start('ros2 run park_control park_control')
                time.sleep(1)
            self.label_parking_2.setText("Start park_control success!")
            if not self.check_process('car_control'):
                proc3.start('ros2 run car_control car_control')
                time.sleep(1)
            self.label_parking_3.setText("Start car_control success!")
            
            
            self.enable_indicator(self.indicatorParking)
        else:
            for pn_ in ['parking_planning', 'park_control', 'car_control']:
                self.kill_process(pn_)
            self.stateparking = False
            # self.buttonParking1.setEnabled(False)
            # self.buttonParking2.setEnabled(False)
            # self.buttonParking3.setEnabled(False)
            self.label_parking_1.setText("")
            self.label_parking_2.setText("")
            self.label_parking_3.setText("")
            self.disable_indicator(self.indicatorParking)
            self.rosHmi.ParkingType = 0
        time.sleep(0.5)
    # def action_button_parking1(self):
    #     self.rosHmi.ParkingType = 1
    # def action_button_parking2(self):
    #     self.rosHmi.ParkingType = 2
    # def action_button_parking3(self):
    #     self.rosHmi.ParkingType = 3
def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    myWin = MyMainWindow()
    myWin.show()
    sys.exit(app.exec_())
