import math
import numpy as np
from scipy.optimize import curve_fit
import cv2

def object_point_world_position(u, v):
    # k = np.array([[582.488, 0.000, 310.280], [
    #     0.000, 579.370, 245.140], [0.000, 0.000, 1.000]])

    # p = np.array([[0, 1, 0, 0], [0, 0, 1, -1], [1, 0, 0, 0], [0, 0, 0, 1]])
    #480p
    k = np.array([[646.47800032  , 0.000     ,   284.36701713],
        [  0.000     ,    648.09336409 ,250.63763335],
        [  0.00       ,    0.00     ,     1.00        ]])
    # p = np.array([[0, -1, 0, 0], [0, 0, 1, -1], [1, 0, 0, 0], [0, 0, 0, 1]])

    #1080p
    # k = np.array([[1.65941273e+03, 0.00000000e+00 ,1.22492270e+03],
    #             [0.00000000e+00 ,1.62358465e+03 ,3.74930467e+02],
    #             [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])
    p = np.array([[ 0.02443498 ,-0.99944833 , 0.02249361,0.0439391],
        [-0.07909819, -0.02436264 ,-0.99656908,1.57088057],
        [ 0.99656731 , 0.02257195 ,-0.07964986,2.02810838],
        [0,0,0,1]])
    # 焦距和相机安装高度
    fx = k[0, 0]
    fy = k[1, 1]
    H = 1.65
    angle_a = 0.05
    angle_b = math.atan((v - 596/2) / fy)
    angle_c = angle_b + angle_a
    if angle_c != 0:
        depth = (H / np.sin(angle_c)) * math.cos(angle_b)
    else:
        depth = 150.00
    k_inv = np.linalg.inv(k)
    p_inv = np.linalg.inv(p)
    # 写入图像坐标
    point_c = np.array([u, v, 1])
    point_c = np.transpose(point_c)
    # 计算距离
    c_position = np.matmul(k_inv, depth * point_c)
    c_position = np.append(c_position, 1)
    c_position = np.transpose(c_position)
    c_position = np.matmul(p_inv, c_position)
    if c_position[0] < 0 or c_position[0]>=150:
        c_position[0] = 150
    d1 = np.array([c_position[0], -c_position[1], 0], dtype=float)
    return d1

# 定义拟合函数形式
def Fun(x, a3, a2, a1, a0):
    return a3*x**3+a2*x**2+a1*x+a0


# 拟合残差
def error(p, x, y):
    return Fun(p, x)-y


def fitting_main(x, y,img):
    a3, a2, a1, a0 = [0, -0, 3, 300]              # 原始数据的参数
    para, pcov = curve_fit(Fun, x, y)
    y_fitted = Fun(x, para[0], para[1], para[2], para[3])
    # 将y坐标变为整型
    for i in range(0, len(y_fitted)):
        y_fitted[i] = int(y_fitted[i])
    line = np.vstack((y_fitted, x))
    line = np.transpose(line)
    # print(line)
    # huabu = np.ones((480,640,3), dtype=np.uint8)*255
    for i in range(0, len(y_fitted)):
        cv2.circle(img, (int(line[i][0]),int(line[i][1])), 10, (0,0,0), 2)
    #     cv2.circle(huabu, (int(y[i]),int(x[i])), 5, (255,255,255), 2)

    # cv2.imshow("3",img)
    # cv2.waitKey(0)
    # plt.figure
    # plt.plot(x, y, 'r', label='Original curve')
    # plt.plot(x, y_fitted, '-b', label='Fitted curve')
    # plt.legend()
    # plt.show()
    # print(para)

    return line


def dis_lane(result, img):
    """
    检测到两条车道时, 用左右两端车道距离计算偏移中线距离, 检测到一条车道线时使用默认宽度计算; 未检测到车道线时, 默认输出0
    DEFAULT_WIDTH_LANE: 默认车道宽度
    threshold_point: 最低点个数，认为可以形成车道线
    """
    threshold_point = 4
    # 分成两条线
    line_left = result[:, 0:2]
    line_right = result[:, 2:4]
    # 删除全零行
    line_left = line_left[~np.all(line_left == 0, axis=1)]
    line_right = line_right[~np.all(line_right == 0, axis=1)]
    # 计算左边车道线距离车辆横向距离
    dis_left = 0.0
    if line_left.shape[0] >= threshold_point:
        line_left = fitting_main(line_left[:, 1].reshape(1, -1)[0],
                                 line_left[:, 0].reshape(1, -1)[0],img)
        #求均值距离
        for i in line_left:
            dis = object_point_world_position(i[0], i[1])
            dis_left = dis[1] + dis_left
        dis_left = dis_left / line_left.shape[0]
    else:
        dis_left = 0.0

    # 计算右边车道线距离车辆横向距离
    dis_right = 0.0
    if line_right.shape[0] >= threshold_point:
        line_right = fitting_main(line_right[:, 1].reshape(1, -1)[0],
                                  line_right[:, 0].reshape(1, -1)[0],img)
        #求均值距离
        for i in line_right:
            dis = object_point_world_position(i[0], i[1])
            dis_right = dis[1] + dis_right
        dis_right = dis_right / line_right.shape[0]
    else:
        dis_right = 0.0
    # 计算中心偏移
    if dis_right != 0 and dis_left!=0:
        dis = -(dis_right+dis_left)/2
    elif dis_left ==0 and dis_right == 0:
        dis = 0.0
    else:
        dis = 0.0
    return dis