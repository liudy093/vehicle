#!/usr/bin/env python3
## -*- coding: utf-8 -*-
# @Copyright: TIAN-JI Intell. Driv. Lab.
# @Author: Aeolson
# @File: global_path_planning.py
# @Project: Auto-Driving System
# @CreateTime: 2022/11/01
# @Description: ***

import sys
import os
import time
import copy
import json
import numpy as np
import cv2
import array
import rclpy
from rclpy.node import Node
from car_interfaces.msg import *

ros_node_name = "global_path_planning"
sys.path.append(os.getcwd() + "/src/utils/")
sys.path.append(os.getcwd() + "/src/%s/%s/"%(ros_node_name, ros_node_name))

import tjitools
import roadnet


class GlobalPathPlanning(Node):
    def __init__(self):
        super().__init__(ros_node_name)

        # degine params
        self.declare_parameter('use_trajectory', False)
        self.declare_parameter('file_path', '')

        self.use_trajectory = self.get_parameter('use_trajectory').get_parameter_value().bool_value
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        self.get_logger().info('use_trajectory = %s, file_path = %s'%(str(self.use_trajectory), self.file_path))

        # define publishers
        self.pubGlobalPathPlanning = self.create_publisher(GlobalPathPlanningInterface, "global_path_planning_data", 10)
        self.timerGlobalPathPlanning = self.create_timer(1, self.pub_callback_global_path_planning)

        # define subscribers
        self.subHmiStartEndPoint = self.create_subscription(HmiStartEndPointInterface, "hmi_start_end_point_data", self.sub_callback_hmi_start_end_point, 1)
        self.subNetStartEndPoint = self.create_subscription(NetStartEndPointInterface, "net_start_end_point_data", self.sub_callback_net_start_end_point, 1)

        self.use_alg = 'astar'
        # self.use_alg = 'dijkstra'
        self.glbMap = None
        self.glbTrack = None
        self.glbStartGps = None
        self.glbEndGps = None
        self.plannedPath = np.array([]) # [lon, lat, alt, hed, vel]

        if self.use_trajectory: # load recorded trajectory
            if not os.path.exists(self.file_path):
                self.get_logger().error("File '%s' is not existed !!!!"%(self.file_path))
                return
            try:
                self.glbTrack = np.loadtxt(fname=self.file_path, dtype=np.float64, delimiter=',', encoding='utf-8')
            except:
                self.get_logger().error("Fail to open the recorded trajectory file '%s' !!!!"%(self.file_path))
                return
            
            if self.glbTrack.shape[1] == 4: # add velocity
                vels_ = 3.0 * np.ones(shape=(len(self.glbTrack),1), dtype=np.float64)
                self.glbTrack = np.concatenate([self.glbTrack.copy(), vels_], axis=1)
                self.get_logger().info('Add vels: shape=(%d, %d)'%(self.glbTrack.shape[0], self.glbTrack.shape[1]))

        else: # load map
            if not os.path.exists(self.file_path):
                self.get_logger().error("Map '%s' is not existed !!!!"%(self.file_path))
                return
            try:
                self.glbMap = roadnet.RoadNet()
                with open(self.file_path, mode='r', encoding='utf-8') as f_:
                    self.glbMap.from_dict(json.load(f_))
            except:
                self.get_logger().error("Fail to open the map '%s' !!!!"%(self.file_path))
                return

        tjitools.ros_log(self.get_name(), 'Start Node:%s'%self.get_name())

    def pub_callback_global_path_planning(self):
        """
        Callback of publisher (timer), publish the topic global_path_planning_data.
        :param None.
        """
        msgGlobalPathPlanning = GlobalPathPlanningInterface()
        now_ts = time.time()
        msgGlobalPathPlanning.timestamp = now_ts
        
        if self.glbStartGps is None:
            msgGlobalPathPlanning.startpoint = []
        else:
            msgGlobalPathPlanning.startpoint = array.array('d', self.glbStartGps[0:2])
        
        if self.glbEndGps is None:
            msgGlobalPathPlanning.endpoint = []
        else:
            msgGlobalPathPlanning.endpoint = array.array('d', self.glbEndGps[0:2])

        # self.get_logger().info("shape = %s"%str(self.plannedPath.shape))

        if len(self.plannedPath) == 0:
            msgGlobalPathPlanning.routedata = []
        else:
            lons_ = self.plannedPath[:,0]
            lats_ = self.plannedPath[:,1]
            heds_ = self.plannedPath[:,3]
            #vels_ = self.plannedPath[:,4]
            vels_ = 3.0 * np.ones_like(lons_)
            outs_ = np.array([lons_, lats_, vels_, heds_]).swapaxes(0,1)
            msgGlobalPathPlanning.routedata = array.array('d', outs_.reshape(-1))

        msgGlobalPathPlanning.process_time = time.time() - now_ts
        self.pubGlobalPathPlanning.publish(msgGlobalPathPlanning)

    def sub_callback_hmi_start_end_point(self, msgHmiStartEndPoint:HmiStartEndPointInterface):
        """
        Callback of subscriber, subscribe the topic hmi_start_end_point_data.
        :param msgHmiStartEndPoint: The message heard by the subscriber.
        """
        if len(msgHmiStartEndPoint.startpoint) < 2 or len(msgHmiStartEndPoint.endpoint) < 2:
            self.glbStartGps = None
            self.glbEndGps = None
            self.plannedPath = np.array([])
            # self.get_logger().info('No Start or End !!!')
            return

        start_lon = msgHmiStartEndPoint.startpoint[0]
        start_lat = msgHmiStartEndPoint.startpoint[1]
        end_lon = msgHmiStartEndPoint.endpoint[0]
        end_lat = msgHmiStartEndPoint.endpoint[1]

        flag_replan = False
        if self.glbStartGps is None or self.glbEndGps is None: # new plan
            flag_replan = True
        elif abs(self.glbStartGps[0]-start_lon) > 1e-5 or abs(self.glbStartGps[1]-start_lat) > 1e-5: # start changed
            flag_replan = True
        elif abs(self.glbEndGps[0]-end_lon) > 1e-5 or abs(self.glbEndGps[1]-end_lat) > 1e-5: # end changed
            flag_replan = True
        
        glbStartGps = np.array([start_lon, start_lat, 0])
        glbEndGps = np.array([end_lon, end_lat, 0])

        if not flag_replan:
            return

        self.get_logger().info('Replan !!! From (%.8f, %.8f) To (%.8f, %.8f)'%(glbStartGps[0], glbStartGps[1], glbEndGps[0], glbEndGps[1]))

        err_lon = glbStartGps[0] - glbEndGps[0]
        err_lat = glbStartGps[1] - glbEndGps[1]
        if abs(err_lon) < 1e-6 and abs(err_lat) < 1e-6: # achieved at end point
            self.plannedPath = np.array([])
            self.glbStartGps = glbStartGps
            self.glbEndGps = glbEndGps
            self.get_logger().info('Achieved at (%.8f, %.8f)'%(glbEndGps[0], glbEndGps[1]))
            return 

        if self.use_trajectory: # track recorded trajectory
            err_ = (self.glbTrack[:,0]-glbStartGps[0])**2 + (self.glbTrack[:,1]-glbStartGps[1])**2
            start_idx = np.argmin(err_)
            err_ = (self.glbTrack[:,0]-glbEndGps[0])**2 + (self.glbTrack[:,1]-glbEndGps[1])**2
            end_idx = np.argmin(err_)
            if end_idx <= start_idx:
                self.plannedPath = np.array([])
            else:
                self.plannedPath = self.glbTrack[start_idx:end_idx+1,:]
                self.plannedPath[:,-1] = 3.0 # set the constant speed as 3m/s
        else: # search global path from map
            # self.get_logger().info("Begin Searching !!!!")
            self.plannedPath = self.search_global_path_from_map(glbStartGps, glbEndGps)
            # self.get_logger().info("End Searching !!!! %d, %d"%(self.plannedPath.shape[0], self.plannedPath.shape[1]))

        
        self.glbStartGps = glbStartGps
        self.glbEndGps = glbEndGps

    def sub_callback_net_start_end_point(self, msgNetStartEndPoint:NetStartEndPointInterface):
        """
        Callback of subscriber, subscribe the topic net_start_end_point_data.
        :param msgNetStartEndPoint: The message heard by the subscriber.
        """
        pass

    def show_global_path(self):

        img_h = 800
        img_w = 800
        img_f = 1e-5
        
        if len(self.plannedPath) == 0:
            return
        lon_ = self.plannedPath[:,0]
        lat_ = self.plannedPath[:,1]

        org_x = (lon_.max() + lon_.min())/2
        org_y = (lat_.max() + lat_.min())/2

        px_ = img_w//2 + np.round((lon_-org_x)/img_f)
        py_ = img_h//2 - np.round((lat_-org_y)/img_f)
        
        validx = (px_>=0) & (px_<img_w) & (py_>=0) & (py_<img_h)
        px_ = px_[validx]
        py_ = py_[validx]

        img_ = 255 * np.ones(shape=(img_h*img_w, 3), dtype=np.uint8)
        pix_ = (py_*img_w + px_).astype(np.int32)
        img_[pix_] = (0,0,255)

        px_ = img_w//2 + np.round((self.glbStartGps[0]-org_x)/img_f)
        py_ = img_h//2 - np.round((self.glbStartGps[1]-org_y)/img_f)
        pix_ = (py_*img_w + px_).astype(np.int32)
        img_[pix_] = (0,255,0)

        px_ = img_w//2 + np.round((self.glbEndGps[0]-org_x)/img_f)
        py_ = img_h//2 - np.round((self.glbEndGps[1]-org_y)/img_f)
        pix_ = (py_*img_w + px_).astype(np.int32)
        img_[pix_] = (255,0,0)

        img_ = img_.reshape(img_h, img_w, 3)
        cv2.imshow('glb_trj', cv2.cvtColor(img_, cv2.COLOR_RGB2BGR))
        cv2.waitKey(5)

    def search_global_path_from_map(self, start_gps:np.ndarray, end_gps:np.ndarray):
        if self.glbMap is None or len(self.glbMap.all_paths) == 0:
            return np.array([])
        
        start_errs, start_idxs, end_errs, end_idxs = [], [], [], []
        for k, ph in enumerate(self.glbMap.all_paths):
            err_ = (ph.longitudes-start_gps[0])**2 + (ph.latitudes-start_gps[1])**2
            p_idx = np.argmin(err_)
            start_errs.append(err_[p_idx])
            start_idxs.append([k, p_idx])

            err_ = (ph.longitudes-end_gps[0])**2 + (ph.latitudes-end_gps[1])**2
            p_idx = np.argmin(err_)
            end_errs.append(err_[p_idx])
            end_idxs.append([k, p_idx])

        if len(start_errs) == 0 or len(end_errs) == 0:
            return np.array([])

        
        k_ = np.argsort(start_errs)[0]
        start_point = {"r_idx": start_idxs[k_][0], "p_idx":start_idxs[k_][1], "error":start_errs[k_]}
        if len(end_errs) == 1:
            end_point_1 = {"r_idx":end_idxs[0][0], "p_idx":end_idxs[0][1], "error":end_errs[0]}
            end_point_2 = None
        else:
            k1_, k2_ = np.argsort(end_errs)[:2]
            end_point_1 = {"r_idx":end_idxs[k1_][0], "p_idx":end_idxs[k1_][1], "error":end_errs[k1_]}
            end_point_2 = {"r_idx":end_idxs[k2_][0], "p_idx":end_idxs[k2_][1], "error":end_errs[k2_]}

        path_1 = self.alg_trajectory_search(start_point, end_point_1)
        
        if end_point_2 is not None:
            path_2 = self.alg_trajectory_search(start_point, end_point_2)
        else:
            path_2 = np.array([])
        
        if len(path_1) > 0 and len(path_2) > 0:
            if len(path_1) <= len(path_2):
                return path_1
            else:
                return path_2
        elif len(path_1) > 0:
            return path_1
        else:
            return path_2
        
    def alg_trajectory_search(self, start_point:dict, end_point:dict):
        if start_point is None or end_point is None:
            return np.array([])
        
        if end_point["r_idx"] == start_point["r_idx"]:
            # the end point is on the same path with the start point
            if end_point["p_idx"] == start_point["p_idx"]:
                # the end point is same to the start point
                return np.array([])
            elif end_point["p_idx"] > start_point["p_idx"]:
                # the end point is ahead of the start point
                ph = self.glbMap.all_paths[end_point["r_idx"]]
                lons_ = ph.longitudes[start_point["p_idx"]:end_point["p_idx"]+1]
                lats_ = ph.latitudes[start_point["p_idx"]:end_point["p_idx"]+1]
                alts_ = ph.altitudes[start_point["p_idx"]:end_point["p_idx"]+1]
                heds_ = ph.headings[start_point["p_idx"]:end_point["p_idx"]+1]
                vels_ = round(ph.high_speed/3.6, 2) * np.ones_like(lons_)
                return np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)
            else:
                # the end point is behind the start point
                ph = self.glbMap.all_paths[end_point["r_idx"]]
                
                lons_ = ph.longitudes[start_point["p_idx"]:]
                lats_ = ph.latitudes[start_point["p_idx"]:]
                alts_ = ph.altitudes[start_point["p_idx"]:]
                heds_ = ph.headings[start_point["p_idx"]:]
                vels_ = round(ph.high_speed/3.6, 2) * np.ones_like(lons_)
                gps_path_1 = np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)

                lons_ = ph.longitudes[:end_point["p_idx"]]
                lats_ = ph.latitudes[:end_point["p_idx"]]
                alts_ = ph.altitudes[:end_point["p_idx"]]
                heds_ = ph.headings[:end_point["p_idx"]]
                vels_ = round(ph.high_speed/3.6, 2) * np.ones_like(lons_)
                gps_path_3 = np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)

                if self.use_alg == 'dijkstra':
                    node_list = self.alg_Dijkstra(start_name=ph.end, end_name=ph.start)
                else:
                    node_list = self.alg_Astar(start_name=ph.end, end_name=ph.start)
                
                if len(node_list) == 0:
                    return np.array([])
                else:
                    gps_path_2 = self.generate_path_from_nodes(node_list)
                    return np.concatenate([gps_path_1, gps_path_2, gps_path_3], axis=0)
        else:
            # the end point is on the different path
            ph_1 = self.glbMap.all_paths[start_point["r_idx"]]
            lons_ = ph_1.longitudes[start_point["p_idx"]:]
            lats_ = ph_1.latitudes[start_point["p_idx"]:]
            alts_ = ph_1.altitudes[start_point["p_idx"]:]
            heds_ = ph_1.headings[start_point["p_idx"]:]
            vels_ = round(ph_1.high_speed/3.6, 2) * np.ones_like(lons_)
            gps_path_1 = np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)

            ph_3 = self.glbMap.all_paths[end_point["r_idx"]]
            lons_ = ph_3.longitudes[:end_point["p_idx"]]
            lats_ = ph_3.latitudes[:end_point["p_idx"]]
            alts_ = ph_3.altitudes[:end_point["p_idx"]]
            heds_ = ph_3.headings[:end_point["p_idx"]]
            vels_ = round(ph_3.high_speed/3.6, 2) * np.ones_like(lons_)
            gps_path_3 = np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)

            if ph_1.end == ph_3.start:
                return np.concatenate([gps_path_1, gps_path_3], axis=0)
            else:
                if self.use_alg == 'dijkstra':
                    node_list = self.alg_Dijkstra(start_name=ph_1.end, end_name=ph_3.start)
                else:
                    node_list = self.alg_Astar(start_name=ph_1.end, end_name=ph_3.start)
                
                if len(node_list) == 0:
                    return np.array([])
                else:
                    gps_path_2 = self.generate_path_from_nodes(node_list)
                    return np.concatenate([gps_path_1, gps_path_2, gps_path_3], axis=0)

    def alg_Astar(self, start_name:str, end_name:str):

        node_graph = self.generate_nodegraph()

        def hcost(cnd):
            dx_ = node_graph[cnd]["x"] - node_graph[end_name]["x"]
            dy_ = node_graph[cnd]["y"] - node_graph[end_name]["y"]
            return np.sqrt(dx_**2 + dy_**2)

        flag_find_end = False
        node_graph[start_name]["cost"] = 0
        node_graph[start_name]["is_inclosed"] = True
        node_graph[start_name]["father"] = None
        now_node = start_name

        for i in range(len(self.glbMap.all_waypoints)):
            # expand from now_node
            for nd_ in node_graph[now_node]["adjacents"].keys():
                if not node_graph[nd_]["is_inclosed"]:
                    new_cost = node_graph[now_node]["cost"] + node_graph[now_node]["adjacents"][nd_]
                    if new_cost < node_graph[nd_]["cost"]:
                        node_graph[nd_]["cost"] = new_cost
                        node_graph[nd_]["father"] = now_node
            
            # add the now_node into closed set & judge if achieved goal
            node_graph[now_node]["is_inclosed"] = True
            if now_node == end_name:
                flag_find_end = True
                break
            
            # search the minimum cost node in closed list
            min_cost = np.Inf
            now_node = None
            for nd_ in node_graph.keys():
                if not node_graph[nd_]["is_inclosed"]:
                    exp_cost = node_graph[nd_]["cost"] + hcost(nd_)
                    if exp_cost < min_cost:
                        min_cost = exp_cost
                        now_node = nd_
            
            # check if the open is empty, search failed
            if now_node is None:
                return []

        # search stop    
        if not flag_find_end:
            return [] # search failed
        
        # search succeess & collect nodelist
        now_node = end_name
        node_list = []
        while now_node is not None:
            node_list.insert(0, now_node)
            now_node = node_graph[now_node]["father"]
        return node_list

    def alg_Dijkstra(self, start_name:str, end_name:str):

        node_graph = self.generate_nodegraph()

        flag_find_end = False
        node_graph[start_name]["cost"] = 0
        node_graph[start_name]["is_inclosed"] = True
        node_graph[start_name]["father"] = None
        now_node = start_name

        for i in range(len(self.glbMap.all_waypoints)):
            
            # expand from now_node
            for nd_ in node_graph[now_node]["adjacents"].keys():
                if not node_graph[nd_]["is_inclosed"]:
                    new_cost = node_graph[now_node]["cost"] + node_graph[now_node]["adjacents"][nd_]
                    if new_cost < node_graph[nd_]["cost"]:
                        node_graph[nd_]["cost"] = new_cost
                        node_graph[nd_]["father"] = now_node

            # add the now_node into closed set & judge if achieved goal
            node_graph[now_node]["is_inclosed"] = True
            if now_node == end_name:
                flag_find_end = True
                break

            # search the minimum cost node in closed list
            min_cost = np.Inf
            now_node = None
            for nd_ in node_graph.keys():
                if not node_graph[nd_]["is_inclosed"]:
                    exp_cost = node_graph[nd_]["cost"]
                    if exp_cost < min_cost:
                        min_cost = exp_cost
                        now_node = nd_
            
            # check if the open is empty, search failed
            if now_node is None:
                return []
            
            
        # search stop & failed
        if not flag_find_end:
            return []
        
        # search succeess & collect nodelist
        now_node = end_name
        node_list = []
        while now_node is not None:
            node_list.insert(0, now_node)
            now_node = node_graph[now_node]["father"]
        return node_list

    def generate_path_from_nodes(self, node_list):

        path_list = []
        for k in range(len(node_list)-1):
            st_ = node_list[k]
            ed_ = node_list[k+1]
            idx_ = self.glbMap.index_path(name_='-'.join([st_, ed_]))
            p_ = self.glbMap.all_paths[idx_]
            lons_ = p_.longitudes
            lats_ = p_.latitudes
            alts_ = p_.altitudes
            heds_ = p_.headings
            vels_ = round(p_.high_speed/3.6, 2) * np.ones_like(lons_)
            data_ = np.array([lons_, lats_, alts_, heds_, vels_]).swapaxes(0,1)
            path_list.append(data_)
        
        return np.concatenate(path_list, axis=0)

    def generate_nodegraph(self):

        node_graph = {}
        for wp in self.glbMap.all_waypoints:
            node_graph[wp.name] = {
                "x": wp.loc.longitude,
                "y": wp.loc.latitude,
                "adjacents": {},
                "father":None,
                "cost":np.Inf,
                "is_inclosed":False
            }
        for ph in self.glbMap.all_paths:
            node_graph[ph.start]["adjacents"][ph.end] = ph.cost
        
        return node_graph

def main():
    rclpy.init()
    rosNode = GlobalPathPlanning()
    rclpy.spin(rosNode)
    rclpy.shutdown()
