
import sys
import os
import threading
import json
import numpy as np
import copy
import roadnet

site_files = (
    './ZUT_demo1_sites.txt',
)

trj_files = (
    './ZUT_demo1_trj.txt',
)

path_high_speed = 10.0 # 10 km/h

save_name = 'map'

if __name__ == '__main__':

    
    my_roadnet = roadnet.RoadNet()
    for i, f_ in enumerate(site_files):
        with open(f_, mode='r', encoding='utf-8') as fr_:
            for s_ in fr_:
                data_ = s_.strip("\n").split(',')
                st = roadnet.RoadNet_Site(name_=data_[0], lon_=np.float64(data_[1]), lat_=np.float64(data_[2]), alt_=np.float64(data_[3]))
                my_roadnet.all_sites.append(st)

    print("Finished processing all site files !!!!")
    
    all_trjs = []
    for i, f_ in enumerate(trj_files):
        trj = np.loadtxt(f_, dtype=np.float64, delimiter=',', encoding='utf-8')
        all_trjs.append(trj)

    print("Finished loading all trj files !!!!")

    def uag180(a):
        # return ang in (-180, +180]
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a
    
    def uag360(a):
        # return ang in [0, 360)
        while a < 0:
            a += 360
        while a >= 360:
            a -= 360
        return a

    all_wps = []
    for trj in all_trjs:

        trj = trj[:,0:4]

        # find the start point
        st_lon, st_lat, st_alt, st_hed = trj[0]
        st_id = None
        for i in range(len(all_wps)):
            lon_, lat_, alt_, hed_ = all_wps[i]
            if abs(lon_-st_lon) < 1e-4 and abs(lat_-st_lat) < 1e-4 and abs(alt_-st_alt) < 1.0 and abs(uag180(hed_-st_hed))<15.0:
                st_id = i
                break
        if st_id is None: # new point
            wp = np.array([st_lon, st_lat, st_alt, st_hed])
            all_wps.append(wp)
            st_id = len(all_wps)-1
        
        # find the end point
        ed_lon, ed_lat, ed_alt, ed_hed = trj[-1]
        ed_id = None
        for i in range(len(all_wps)):
            lon_, lat_, alt_, hed_ = all_wps[i]
            if abs(lon_-ed_lon) < 1e-4 and abs(lat_-ed_lat) < 1e-4 and abs(alt_-ed_alt) < 1.0 and abs(uag180(hed_-ed_hed))<15.0:
                ed_id = i
                break
        if ed_id is None: # new point
            wp = np.array([ed_lon, ed_lat, ed_alt, ed_hed])
            all_wps.append(wp)
            ed_id = len(all_wps)-1
        
        # st_lon, st_lat, st_alt, st_hed = all_wps[st_id]
        # ed_lon, ed_lat, ed_alt, ed_hed = all_wps[ed_id]
        # trj = np.concatenate([np.array([[st_lon, st_lat, st_alt, st_hed]]), trj], axis=0)
        # trj = np.concatenate([trj, np.array([[ed_lon, ed_lat, ed_alt, ed_hed]])], axis=0)
        
        ph = roadnet.RoadNet_Path(
            name_='-'.join([str(st_id+1), str(ed_id+1)]),
            start_=str(st_id+1),
            end_=str(ed_id+1),
            highspeed_=path_high_speed,
            lowspeed_=0,
            lons_=trj[:,0],
            lats_=trj[:,1],
            alts_=trj[:,2],
            heds_=trj[:,3]
        )
        my_roadnet.all_paths.append(ph)
    
    print("Finished processing all trjs !!!!")

    for i in range(len(all_wps)):
        lon_, lat_, alt_, hed_ = all_wps[i]
        wp = roadnet.RoadNet_Waypoint(
            name_=str(i+1),
            lon_=lon_,
            lat_=lat_,
            alt_=alt_,
            hed_=hed_
        )
        my_roadnet.all_waypoints.append(wp)

    f_ = './' + save_name + '.json'
    with open(f_, mode="w", encoding="utf-8") as fw_:
        json.dump(my_roadnet.to_dict(), fw_, ensure_ascii=False, indent=4)



        

    

