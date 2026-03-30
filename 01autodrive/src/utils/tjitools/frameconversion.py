"""
FrameConversion
=====

Provides
  1. Coordinate conversion between the GPS frame and the ENU frame
  2. Coordinate conversion between the ENU frame and the RFU frame
  3. Coordinate conversion between the GPS frame and the RFU frame

Author
  Aelson @ TJI Intell. Driv. Lab.

Attention
  The conversion is based on approximation with the assumption of flatten local ground
  rather than sphere. If the area is large, please switch the GPS origin ``GPS_ORG'' with
  the function set_gps_org().
"""

import math
import numpy as np

WGS84_A     = 6378137.0
WGS84_B     = 6356752.31424518
WGS84_ER    = 6371000.0
D2R         = math.pi / 180.0
R2D         = 180.0 / math.pi


# ZUT
GPS_ORG_LON = 113.6791186
GPS_ORG_LAT = 34.58765069

# TJU
# GPS_ORG_LON = 117.1645294
# GPS_ORG_LAT = 39.1066726
GPS_ORG_ALT = 0
GPS_ORG = np.array([GPS_ORG_LON, GPS_ORG_LAT, GPS_ORG_ALT])

def check_input_data(data):
    if data.ndim != 1 and data.ndim != 2:
        raise ValueError("the input must be with dimension 1 or 2")

def calc_trans_matrix_rfu(rot=(0,0,0)):
    """
    rot = [pitch, roll, yaw]; unit is 'degree'; value ranges [0.0, 359.99]
    'pitch' is along x-axis; follow right-hand rule; positive direction is anticlockwise;
    'roll'  is along y-axis; follow right-hand rule; positive direction is anticlockwise;
    'yaw    is along z-axis; follow right-hand rule; positive direction is anticlockwise; yaw=0 means heading towards 'East'
    """

    pitch_, roll_, yaw_ = rot
    yaw_ = np.deg2rad(90 - yaw_) # transite the yaw from east-north (anticlockwise) to north-east (clockwise), degree to radian
    # yaw_ = math.pi/2 - yaw_

    tm_z = np.matrix([
        [math.cos(yaw_),    -math.sin(yaw_),    0                   ],
        [math.sin(yaw_),    math.cos(yaw_),     0                   ],
        [0,                 0,                  1                   ]
    ], dtype=np.float64)
    tm_x = np.matrix([
        [1,                 0,                  0                   ],
        [0,                 math.cos(pitch_),   math.sin(pitch_)    ],
        [0,                 -math.sin(pitch_),  math.cos(pitch_)    ]
    ], dtype=np.float64)
    tm_y = np.matrix([
        [math.cos(roll_),   0,                  -math.sin(roll_)    ],
        [0,                 1,                  0                   ],
        [math.sin(roll_),   0,                  math.cos(roll_)     ]
    ], dtype=np.float64)
    
    tm_enu_rfu = np.matmul(tm_y, np.matmul(tm_x, tm_z))
    return tm_enu_rfu

def set_gps_org(org_lon, org_lat, org_alt):
    global GPS_ORG_LON, GPS_ORG_LAT, GPS_ORG_ALT, GPS_ORG

    GPS_ORG_LON = org_lon
    GPS_ORG_LAT = org_lat
    GPS_ORG_ALT = org_alt
    GPS_ORG = np.array([GPS_ORG_LON, GPS_ORG_LAT, GPS_ORG_ALT])

def get_gps_org():
    return GPS_ORG

def gps_to_enu(pt_gps):
    """
    Transform the coordinate GPS (Longitude, Latitude, Altitude) to the coordinate ENU (East, North, Up)
    :param pt_gps: Input GPS points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :return: ENU points
    """

    check_input_data(pt_gps)
    
    if pt_gps.ndim == 1:
        lon_, lat_, alt_ = pt_gps[:, np.newaxis]
    else:
        lon_, lat_, alt_ = pt_gps.swapaxes(0,1)
    
    e_ = WGS84_A * np.cos(lat_*D2R) * (lon_ - GPS_ORG_LON)*D2R
    n_ = WGS84_B * (lat_ - GPS_ORG_LAT)*D2R
    u_ = alt_ - GPS_ORG_ALT
    pt_enu = np.array([e_, n_, u_]).swapaxes(0,1)
    if pt_gps.ndim == 1:
        pt_enu = pt_enu.squeeze()
    
    return pt_enu

def enu_to_gps(pt_enu):
    """
    Transform the coordinate ENU (East, North, Up) to the coordinate GPS (Longitude, Latitude, Altitude)
    :param pt_enu: Input ENU points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :return: GPS points
    """
    
    check_input_data(pt_enu)
    
    if pt_enu.ndim == 1:
        e_, n_, u_ = pt_enu[:, np.newaxis]
    else:
        e_, n_, u_ = pt_enu.swapaxes(0,1)
    
    lat_ = n_ / WGS84_B * R2D + GPS_ORG_LAT
    lon_ = e_ / WGS84_A / np.cos(lat_*D2R) * R2D + GPS_ORG_LON
    alt_ = u_ + GPS_ORG_ALT
    pt_gps = np.array([lon_, lat_, alt_]).swapaxes(0,1)
    if pt_enu.ndim == 1:
        pt_gps = pt_gps.squeeze()
    
    return pt_gps

def enu_to_rfu(pt_enu, og_enu, rot=(0,0,0)):
    """
    Transform the coordinate ENU (East, North, Up) to the vehicle-coordinate RFU (Right, Front, Up)
    :param pt_enu: Input ENU points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :param og_enu: ENU coordinate of the RFU origin (vehicluar IMU center)
    :param rot: rotation of the vehicle along R-F-U (Pitch, Roll, Yaw), the unit is 'degree', values [0, 359.99]
    :return: RFU points
    """
    check_input_data(pt_enu)

    tm_enu2rfu = calc_trans_matrix_rfu(rot)

    ndim_ = pt_enu.ndim
    if ndim_ == 1:
        pt_enu = pt_enu[np.newaxis,:]
    
    pt_enu = np.asmatrix(pt_enu - og_enu)
    pt_rfu = np.matmul(tm_enu2rfu, pt_enu.T)
    pt_rfu = np.asarray(pt_rfu.T)

    if ndim_ == 1:
        pt_rfu = pt_rfu.squeeze()
    
    return pt_rfu

def rfu_to_enu(pt_rfu, og_enu, rot=(0,0,0)):
    """
    Transform the coordinate vehicle-coordinate RFU (Right, Front, Up) to the ENU (East, North, Up)
    :param pt_enu: Input RFU points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :param og_enu: ENU coordinate of the RFU origin (vehicluar IMU center)
    :param rot: rotation of the vehicle along R-F-U (Pitch, Roll, Yaw), the unit is 'degree', values [0, 359.99]
    :return: ENU points
    """
    check_input_data(pt_rfu)

    tm_rfu2enu = calc_trans_matrix_rfu(rot).T

    ndim_ = pt_rfu.ndim
    if ndim_ == 1:
        pt_rfu = pt_rfu[np.newaxis,:]
    
    pt_rfu = np.asmatrix(pt_rfu)
    pt_enu = np.matmul(tm_rfu2enu, pt_rfu.T)
    pt_enu = np.asarray(pt_enu.T)
    pt_enu = pt_enu + og_enu

    if ndim_ == 1:
        pt_enu = pt_enu.squeeze()
    
    return pt_enu

def gps_to_rfu(pt_gps, og_gps, rot=(0,0,0)):
    """
    Transform the coordinate GPS (Longitude, Latitude, Altitude) to the vehicle-coordinate RFU (Right, Front, Up)
    :param pt_gps: Input GPS points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :param og_gps: GPS coordinate of the RFU origin (vehicluar IMU center)
    :param rot: rotation of the vehicle along R-F-U (Pitch, Roll, Yaw), the unit is 'degree', values [0, 359.99]
    :return: RFU points
    """
    check_input_data(pt_gps)

    og_enu = gps_to_enu(og_gps)
    pt_enu = gps_to_enu(pt_gps)
    pt_rfu = enu_to_rfu(pt_enu, og_enu, rot)

    return pt_rfu

def rfu_to_gps(pt_rfu, og_gps, rot=(0,0,0)):
    """
    Transform the coordinate GPS (Longitude, Latitude, Altitude) to the vehicle-coordinate RFU (Right, Front, Up)
    :param pt_rfu: Input RFU points, must be an 1D-ndarray with shape (3) or a 2D-ndarray with shape (n_samples, 3)
    :param og_gps: GPS coordinate of the RFU origin (vehicluar IMU center)
    :param rot: rotation of the vehicle along R-F-U (Pitch, Roll, Yaw), the unit is 'degree', values [0, 359.99]
    :return: RFU points
    """
    check_input_data(pt_rfu)

    og_enu = gps_to_enu(og_gps)
    pt_enu = rfu_to_enu(pt_rfu, og_enu, rot)
    pt_gps = enu_to_gps(pt_enu)

    return pt_gps

