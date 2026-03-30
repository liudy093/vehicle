#!/usr/bin/env python3
## -*- coding: utf-8 -*-

"""

Car model for Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""
import time
import sys
import os
import pathlib
root_dir = pathlib.Path(__file__).parent.parent.parent
sys.path.append(str(root_dir))

from math import cos, sin, tan, pi

import matplotlib.pyplot as plt
import numpy as np

from parking_path_planning.angle import rot_mat_2d

sys.path.append(os.getcwd() + "/src/utils/")
# sys.path.append(os.getcwd() + "/src/zouzhenshan/parking_path_planning/")
WB = 2.63  # rear to front wheel
W = 1.8  # width of car
LF = 3.4  # distance from rear to vehicle front end
LB = 1  # distance from rear to vehicle back end
MAX_STEER = np.deg2rad(35.0)  # [rad] maximum steering angle 
                 # 0.4 for practical experiment

BUBBLE_DIST = (LF - LB) / 2.0  # distance from rear to center of vehicle.
BUBBLE_R = np.hypot((LF + LB) / 2.0, W / 2.0)  # bubble radius

# vehicle rectangle vertices
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W / 2, -W / 2, -W / 2, W / 2, W / 2]


def check_car_collision(x_list, y_list, yaw_list, ox, oy, kd_tree):
    # T1 = time.perf_counter()

    for i_x, i_y, i_yaw in zip(x_list, y_list, yaw_list):
        cx = i_x + BUBBLE_DIST * cos(i_yaw)
        cy = i_y + BUBBLE_DIST * sin(i_yaw)

        ids = kd_tree.query_ball_point([cx, cy], BUBBLE_R+0.3)

        if not ids:
            continue

        # original version
        if not rectangle_check(i_x, i_y, i_yaw,
                               [ox[i] for i in ids], [oy[i] for i in ids]):
            return False  # no collision

    # T2 = time.perf_counter()
    # print('程序运行时间:%s毫秒' % ((T2 - T1)*1000))
    return True  # collision

def rectangle_check(x, y, yaw, ox, oy):
    # transform obstacles to base link frame
    rot = rot_mat_2d(yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        converted_xy = np.stack([tx, ty]).T @ rot
        rx, ry = converted_xy[0], converted_xy[1]
         
        #oringal version
        # if not (rx > LF or rx < -LB or ry > W / 2.0 or ry < -W / 2.0):
        if not (rx > LF+0.1 or rx < -LB-0.1 or ry > (W / 2.0)+0.1 or ry < (-W / 2.0)-0.1):
            return False  # no collision

    return True  # collision


def plot_arrow(x, y, yaw, length=0.6, width=0.2, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)


def plot_car(x, y, yaw):
    car_color = 'b'
    c, s = cos(yaw), sin(yaw)
    rot = rot_mat_2d(-yaw)
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        converted_xy = np.stack([rx, ry]).T @ rot
        car_outline_x.append(converted_xy[0]+x)
        car_outline_y.append(converted_xy[1]+y)

    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    arrow_x, arrow_y, arrow_yaw = x, y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y, car_color, linewidth=1.5)


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi


def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw += pi_2_pi(distance * tan(steer) / L)  # distance/2
    return x, y, yaw

def sign(a):
    if a == 0:
        return 0
    elif a > 0:
        return 1
    elif a < 0:
        return -1
     
def main():
    x, y, yaw = 0., 0., 1.
    plt.axis('equal')
    plot_car(x, y, yaw)
    plt.show()


if __name__ == '__main__':
    main()