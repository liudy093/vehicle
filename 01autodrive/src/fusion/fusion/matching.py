"""
Author: chen fang
CreateTime: 2022/10/25 下午9:35
File: matching.py
Description: 使用匈牙利算法进行匹配
"""

import math
import numpy as np
from scipy.optimize import linear_sum_assignment


def associate_detections_to_trackers(detections, trackers, max_dis=10, ED_threshold=0.6):
    """
    基于距离的匹配 : sqrt( (x1-x2)^2 + (y1-y2)^2 +(z1-z2)^2 )
    :param detections: numpy [[category,l,w,h,x,y,z,v,latv],[...],[...]]
    :param trackers: numpy [[category,l,w,h,x,y,z,v,latv],[...],[...]]
    :param max_dis: 最大的匹配距离
    :param ED_threshold: 匹配的阈值(0.6 = 6m)
    :return: matched, unmatched_dets, unmatched_trks
    """
    # 如果没有预测框直接输出
    if len(trackers) == 0:
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 8, 3), dtype=int)

    ED_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
    # 成本矩阵
    for d, det in enumerate(detections):
        for t, trk in enumerate(trackers):
            # 对于距离大于 max_dis 的物体不进行匹配
            if math.sqrt(math.pow(det[4] - trk[4], 2) + math.pow(det[5] - trk[5], 2)) >= max_dis:
                ED_matrix[d, t] = 0
            else:
                ED_matrix[d, t] = (10 - math.sqrt(
                    math.pow(det[4] - trk[4], 2) + math.pow(det[5] - trk[5], 2))) / max_dis

    # 匈牙利算法进行匹配
    row_ind, col_ind = linear_sum_assignment(-ED_matrix)  # 匈牙利算法
    matched_indices = np.stack((row_ind, col_ind), axis=1)

    unmatched_detections = []
    for d, det in enumerate(detections):
        if d not in matched_indices[:, 0]: unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if t not in matched_indices[:, 1]: unmatched_trackers.append(t)

    # 过滤掉低IOU匹配
    matches = []
    for m in matched_indices:
        if ED_matrix[m[0], m[1]] < ED_threshold:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))

    if len(matches) == 0:
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)
