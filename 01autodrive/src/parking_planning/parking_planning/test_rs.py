import pybind11
import numpy as np
import math
import os
import sys
import matplotlib.pyplot as plt
from rs import rs_func

if __name__ == '__main__':
    paremeters = np.zeros(8).astype(np.float32)
    paremeters[0] = 69.84210491825463  # 起点x坐标
    paremeters[1] = 222.62476053512654  # 起点y坐标
    paremeters[2] = 179.25999450683594  # 起点航向角（角度）
    paremeters[3] = 43.246196054379205  # 终点x坐标
    paremeters[4] = 222.54931708001106  # 终点y坐标
    paremeters[5] = 180.92999267578125  # 终点航向角（角度）
    paremeters[6] = 0.5  # 轨迹步长
    paremeters[7] = 0  # 轨迹类型

    path = rs_func.get_rs_track(paremeters)  # [N,3]
    print(path)

    f = plt.figure()
    ax = f.add_subplot(111)
    ax.set(xlim=[30, 80], ylim=[200,250 ])

    plt.plot(path[0, 0], path[0, 1], 'ro')
    plt.arrow(path[0, 0], path[0, 1], 1.0 * math.cos(path[0, 2]), 1.0 * math.sin(path[0, 2]),
              fc="r", ec="k", head_width=0.5, head_length=0.5)
    plt.plot(path[len(path) - 1, 0], path[len(path) - 1, 1], 'yo')
    plt.arrow(path[len(path) - 1, 0], path[len(path) - 1, 1], 1.0 * math.cos(path[len(path) - 1, 2]),
              1.0 * math.sin(path[len(path) - 1, 2]),
              fc="r", ec="k", head_width=0.5, head_length=0.5)
    plt.plot(path[:, 0], path[:, 1], '-')
    plt.show()

    print('aaaaa')

    pass
