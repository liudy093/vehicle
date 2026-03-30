"""
Author: chen fang
CreateTime: 2022/10/25 下午9:35
File: mot_dis.py
Description: 基于距离的多目标跟踪
"""
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


def associate_detections_to_trackers(detections, trackers, max_dis=10, ED_threshold=0.6):
    """
    基于距离的匹配 : sqrt( (x1-x2)^2 + (y1-y2)^2 +(z1-z2)^2 )
    :param detections: list array [[x,y,z],[...],[...]]
    :param trackers: list array [[x,y,z],[...],[...]]
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
            if math.sqrt(
                    math.pow(det[0] - trk[0], 2) + math.pow(det[1] - trk[1], 2) + math.pow(det[2] - trk[2],
                                                                                           2)) >= max_dis:
                ED_matrix[d, t] = 0
            else:
                ED_matrix[d, t] = (10 - math.sqrt(
                    math.pow(det[0] - trk[0], 2) + math.pow(det[1] - trk[1], 2) + math.pow(det[2] - trk[2],
                                                                                           2))) / max_dis

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


class MOT_dis(object):
    """
    基于距离的多目标跟踪
    """

    def __init__(self, min_hits=3, max_age=3, max_dis=10, ED_threshold=0.6):  # 默认 min_hits=3,max_age=3
        self.min_hits = min_hits
        self.max_age = max_age
        self.max_dis = max_dis
        self.ED_threshold = ED_threshold
        self.trackers = []
        self.frame_count = 0
        self.id = 0

    def run(self, dection_result, frame):
        """
        跟踪
        :param dection_result: numpy array [[category,l,w,h,x,y,z,v,latv],[...],[...]]
        :param frame: int array
        :return: tracking_result： list array [[category,id,l,w,h,x,y,z,v,latv],[...],[...]]
        """
        dets = []
        for dect in dection_result:
            a = [dect[4], dect[5], dect[6]]
            dets.append(a)

        trks = []
        for trk in self.trackers:
            trk_pos = trk['pos']
            a = [trk_pos[4], trk_pos[5], trk_pos[6]]
            trks.append(a)

        # 前后帧匹配
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks,
                                                                                   max_dis=self.max_dis,
                                                                                   ED_threshold=self.ED_threshold)

        # 为跟踪更新状态
        for t, trk in enumerate(self.trackers):
            if t in unmatched_trks:  # 如果不匹配， unmatch_age +1
                trk_pos = trk['pos']
                trk_pos[12] += 1
            else:  # 如果匹配，match_age + 1 , unmatch_age 清零
                trk_pos = trk['pos']
                trk_pos[11] += 1
                trk_pos[12] = 0

        # 为不匹配的检测创建和初始化新的跟踪器
        for i in unmatched_dets:
            trk = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            a = {}
            det = dection_result[i]

            trk[0:9] = det[0:9]
            trk[9] = frame  # frame
            trk[10] = self.id  # id
            trk[11] = 1  # match_age
            trk[12] = 0  # unmatch_age

            # trk[0] = det[0]  # class
            # trk[1] = det[1]  # l
            # trk[2] = det[2]  # w
            # trk[3] = det[3]  # h
            # trk[4] = det[4]  # x
            # trk[5] = det[5]  # y
            # trk[6] = det[6]  # z
            # trk[7] = det[7]  # v
            # trk[8] = det[8]  # latv

            a['pos'] = trk

            self.trackers.append(a)
            self.id += 1

        # 为匹配的跟踪更新状态
        for i in matched:
            trk = self.trackers[i[1]]
            trk_pos = trk['pos']
            det = dection_result[i[0]]

            trk_pos[0:9] = det[0:9]
            trk_pos[9] = frame  # frame
            # trk_pos[0] = det[0]  # class
            # trk_pos[1] = det[1]  # l
            # trk_pos[2] = det[2]  # w
            # trk_pos[3] = det[3]  # h
            # trk_pos[4] = det[4]  # x
            # trk_pos[5] = det[5]  # y
            # trk_pos[6] = det[6]  # z
            # trk_pos[7] = det[7]  # v
            # trk_pos[8] = det[8]  # latv

        # 删除轨迹
        for i, trk in enumerate(self.trackers):
            trk_pos = trk['pos']
            if trk_pos[12] >= self.max_age:
                self.trackers.pop(i)

        # 返回跟踪结果
        tracking_result = []
        for t, trk in enumerate(self.trackers):
            trk_pos = trk['pos']
            a = [0, 0, 0, 0, 0, 0, 0, 0, 0]

            # 对于 帧数 == frame，匹配次数 >= 2,未匹配次数 == 0 的目标，输出
            if (trk_pos[9] == frame) and (trk_pos[11] >= self.min_hits) and (trk_pos[12] == 0):
                a[:] = trk_pos[:9]
                tracking_result.append(a)

        if len(tracking_result) > 0:
            print("tracking_result 不为0")
            return tracking_result
        print("tracking_result为0")
        return []
