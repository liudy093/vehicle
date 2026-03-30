"""
RoadNet
=====

Provides
  1. User defined roadnet file, including waypoint, site, path, traffic light, traffic sign
  2. Load from & save into '*.json' map file

Author
  Aelson @ TJI Intell. Driv. Lab.

"""

import sys
import copy
import operator
import tjitools
import numpy as np

import sys
import copy
import operator
import tjitools
from typing import *
import numpy as np

class RoadNet_GPSPoint(object):    
    def __init__(self, lon_=0, lat_=0, alt_=0, hed_=0):
        self.longitude = round(lon_,8)
        self.latitude = round(lat_,8)
        self.altitude = round(alt_,2)
        self.heading = round(hed_,2)
    
    def set(self, lon_, lat_, alt_, hed_):
        self.longitude = round(lon_,8)
        self.latitude = round(lat_,8)
        self.altitude = round(alt_,2)
        self.heading = round(hed_,2)

class RoadNet_Waypoint(object):
    def __init__(self, name_="", lon_=0, lat_=0, alt_=0, hed_=0):
        self.name = name_
        self.loc = RoadNet_GPSPoint(lon_, lat_, alt_, hed_)
    
    def set(self, lon_, lat_, alt_, hed_):
        self.loc.set(lon_, lat_, alt_, hed_)

class RoadNet_Site(object):
    def __init__(self, name_="", lon_=0, lat_=0, alt_=0):
        self.name = name_
        self.loc = RoadNet_GPSPoint(lon_, lat_, alt_, 0)
    
    def set(self, lon_, lat_, alt_):
        self.loc.set(lon_, lat_, alt_, 0)

class RoadNet_TrafficLight(object):
    def __init__(self, name_="", lon_=0, lat_=0, alt_=0, hed_=0, stop_lon_=0, stop_lat_=0):
        self.name = name_
        self.loc = RoadNet_GPSPoint(lon_, lat_, alt_, hed_)
        self.stopline = RoadNet_GPSPoint(stop_lon_, stop_lat_, 0, hed_)
    
    def set(self, lon_, lat_, alt_, hed_, stop_lon_, stop_lat_):
        self.loc.set(lon_, lat_, alt_, hed_)
        self.stopline.set(stop_lon_, stop_lat_, 0, hed_)        
    
class RoadNet_Path(object):
    def __init__(self, name_="", start_="", end_="", highspeed_=0, lowspeed_=0, lons_=[], lats_=[], alts_=[], heds_=[]):
        self.name = name_
        self.start = start_
        self.end = end_
        self.high_speed = highspeed_
        self.low_speed = lowspeed_

        if min([len(lons_), len(lats_), len(alts_), len(heds_)]) == 0:
            self.longitudes = np.array([])
            self.latitudes = np.array([])
            self.altitudes = np.array([])
            self.headings = np.array([])
            self.cost = 0
        else:
            self.longitudes = np.round(np.array(lons_), 8)
            self.latitudes = np.round(np.array(lats_), 8)
            self.altitudes = np.round(np.array(alts_), 2)
            self.headings = np.round(np.array(heds_), 2)

            tjitools.set_gps_org(lons_[0], lats_[0], alts_[0])
            xyz_ = tjitools.gps_to_enu(np.array([lons_, lats_, alts_]).swapaxes(0,1))
            delta_xyz_ = xyz_[1:,:] - xyz_[:-1,:]
            self.cost = np.round(np.sum(np.sqrt(np.sum(delta_xyz_**2, axis=-1))), 2)
    
    def set(self, start_, end_, highspeed_, lowspeed_, lons_, lats_, alts_, heds_):
        self.start = start_
        self.end = end_
        self.high_speed = highspeed_
        self.low_speed = lowspeed_

        self.longitudes = np.round(np.array(lons_), 8)
        self.latitudes = np.round(np.array(lats_), 8)
        self.altitudes = np.round(np.array(alts_), 2)
        self.headings = np.round(np.array(heds_), 2)

        tjitools.set_gps_org(lons_[0], lats_[0], alts_[0])
        xyz_ = tjitools.gps_to_enu(np.array([lons_, lats_, alts_]).swapaxes(0,1))
        delta_xyz_ = xyz_[1:,:] - xyz_[:-1,:]
        self.cost = np.round(np.sum(np.sqrt(np.sum(delta_xyz_**2, axis=-1))), 2)

class RoadNet(object):
    def __init__(self):

        self.all_sites = []
        self.all_waypoints = []
        self.all_paths = []
        self.all_trafficlights = []

    def index_waypoint(self, name_):
        if name_ == "":
            return -1
        for idx_, d_ in enumerate(self.all_waypoints):
            if d_.name == name_:
                return idx_
        return -1
    
    def index_site(self, name_):
        for idx_, d_ in enumerate(self.all_sites):
            if d_.name == name_:
                return idx_
        return -1

    def index_path(self, name_):
        if name_ == "":
            return -1
        for idx_, d_ in enumerate(self.all_paths):
            if d_.name == name_:
                return idx_
        return -1

    def index_light(self, name_):
        for idx_, d_ in enumerate(self.all_trafficlights):
            if d_.name == name_:
                return idx_
        return -1

    def to_dict(self):
        rn = {}

        rn["sites"] = []
        for st in self.all_sites:
            # st = RoadNet_Site()
            st_dict = {
                "name": st.name,
                "location": {
                    "longitude": st.loc.longitude,
                    "latitude": st.loc.latitude,
                    "altitude": st.loc.altitude
                }
            }
            rn["sites"].append(st_dict)

        rn["waypoints"] = []
        for wp in self.all_waypoints:
            # wp = RoadNet_Waypoint()
            wp_dict = {
                "name": wp.name,
                "location": {
                    "longitude": wp.loc.longitude,
                    "latitude": wp.loc.latitude,
                    "altitude": wp.loc.altitude,
                    "heading": wp.loc.heading
                }
            }
            rn["waypoints"].append(wp_dict)
        
        rn["trafficlights"] = []
        for tl in self.all_trafficlights:
            # tl = RoadNet_TrafficLight()
            tl_dict = {
                "name": tl.name,
                "location": {
                    "longitude": tl.loc.longitude,
                    "latitude": tl.loc.latitude,
                    "altitude": tl.loc.altitude,
                    "heading": tl.loc.heading
                },
                "stopline": {
                    "longitude": tl.stopline.longitude,
                    "latitude": tl.stopline.latitude
                }
            }
            rn["trafficlights"].append(tl_dict)

        rn["paths"] = []
        for ph in self.all_paths:
            # ph = RoadNet_Path()
            ph_dict = {
                "name": ph.name,
                "start": ph.start,
                "end": ph.end,
                "highspeed": ph.high_speed,
                "lowspeed": ph.low_speed,
                "cost": ph.cost,
                "truepath": [{
                    "longitude": lon_,
                    "latitude": lat_,
                    "altitude": alt_,
                    "heading": hed_
                } for (lon_, lat_, alt_, hed_) in zip(ph.longitudes, ph.latitudes, ph.altitudes, ph.headings)]
            }
            rn["paths"].append(ph_dict)
        
        return rn

    def from_dict(self, rn_dict):

        self.all_sites = []
        for d_ in rn_dict["sites"]:
            st = RoadNet_Site(
                name_   = d_["name"],
                lon_    = d_["location"]["longitude"],
                lat_    = d_["location"]["latitude"],
                alt_    = d_["location"]["altitude"]
            )
            self.all_sites.append(st)

        self.all_waypoints = []
        for d_ in rn_dict["waypoints"]:
            wp = RoadNet_Waypoint(
                name_   = d_["name"],
                lon_    = d_["location"]["longitude"],
                lat_    = d_["location"]["latitude"],
                alt_    = d_["location"]["altitude"],
                hed_    = d_["location"]["heading"]
            )
            self.all_waypoints.append(wp)
        
        self.all_trafficlights = []
        for d_ in rn_dict["trafficlights"]:
            tl = RoadNet_TrafficLight(
                name_       = d_["name"],
                lon_        = d_["location"]["longitude"],
                lat_        = d_["location"]["latitude"],
                alt_        = d_["location"]["altitude"],
                hed_        = d_["location"]["heading"],
                stop_lon_   = d_["stopline"]["longitude"],
                stop_lat_   = d_["stopline"]["latitude"]
            )
            self.all_trafficlights.append(tl)
        
        self.all_paths = []
        for d_ in rn_dict["paths"]:
            gps_ = [[p_["longitude"], p_["latitude"], p_["altitude"], p_["heading"]] for p_ in d_["truepath"]]
            lons_, lats_, alts_, heds_ = np.array(gps_).swapaxes(0,1)
            ph = RoadNet_Path(
                name_       = d_["name"],
                start_      = d_["start"],
                end_        = d_["end"],
                highspeed_  = d_["highspeed"],
                lowspeed_   = d_["lowspeed"],
                lons_       = lons_,
                lats_       = lats_,
                alts_       = alts_,
                heds_       = heds_
            )
            self.all_paths.append(ph)
