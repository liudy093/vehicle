import numpy as np
import math

u = 320
v = 200
w = 20
h = 10
W = 640
H = 480
a = np.array([[600, 280, 20, 0, 1, 3]])

def object_point_world_position(u, v, w, h):
    #内参   
    k = np.array([[769.50412258, 0.000, 270.45676359], [
    0.000, 772.92576942, 219.68535813], [0.000, 0.000, 1.000]])
    # p = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    p = np.array([[0, -1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    #外参
    # p = np.array([[ 0.07233302 ,-0.99712603  ,0.02253053,0.07407906], [ 0.03081982 ,-0.02034434 ,-0.99931789,0.73848666], [ 0.99690424,  0.07297806,  0.02925968,0.88268976], [0, 0, 0, 1]])
    # 得到框最下面的中点
    u1 = u
    v1 = v + h / 2
    # 焦距和相机安装高度
    fx = k[0, 0]
    fy = k[1, 1]
    H = 0.75
    angle_a = 0
    angle_b = math.atan((v1 - 480/2) / fy)
    angle_c = angle_b + angle_a
    if angle_c != 0:
        depth = (H / np.sin(angle_c)) * math.cos(angle_b)
    else:
        depth = 200.00
    k_inv = np.linalg.inv(k)
    p_inv = np.linalg.inv(p)
    #写入图像坐标
    point_c = np.array([u1, v1, 1])
    point_c = np.transpose(point_c)
    #计算距离
    c_position = np.matmul(k_inv, depth * point_c)
    c_position = np.append(c_position, 1)
    c_position = np.transpose(c_position)
    c_position = np.matmul(p_inv, c_position)
    print(c_position)
    if c_position[0] < 0:
        c_position[0] = 150
    d1 = np.array([c_position[0], c_position[1], c_position[2]], dtype=float)
    return d1
def tran_category(a):
    #针对每一个类别设计不同物体大小
    if a[0] == 0 or a[0] == 1:
        a[0] = 3
        a=np.append(a,[0.20,0.30,2.00])
    elif a[0] == 2:
        a[0] = 1
        a=np.append(a,[4.00,1.80,1.5])
    elif a[0] == 3 or a[0] == 4:
        a[0] = 2
        a=np.append(a,[10.00,2.50,3.00])
    elif a[0] == 5:
        a[0] = 5
        a=np.append(a,[1.50,0.10,0.80])
    elif a[0] == 6:
        a[0] = 4
        a=np.append(a,[1.50,0.10,0.80])
    elif a[0] == 7 or a[0] == 8 or a[0] == 9 or a[0] == 10:
        a[0] = 8
        a=np.append(a,[0.20 ,0.60,0.22])
    elif a[0] == 11:
        a[0] = 9
        a=np.append(a,[0.10,0.60,0.60])
    return a
def result_to_msg(result):
    #先排序
    result = result[:, [5, 0, 1, 2, 3]]
    e = []
    for i in result:
        i = tran_category(i)
        d = object_point_world_position(i[1], i[2], i[3], i[4])
        f = np.concatenate([i, d])
        e.append(f)
    e=np.array(e)
    print(e)
    e = e[:,[0,5,6,7,8,9,10,1,2]]
    e[:,-1].fill(0)
    e[:,-2].fill(0)
    result_ = sum(e.tolist(),[])
    return result_

if __name__ == "__main__":
    result = result_to_msg(a)
    print(result)
