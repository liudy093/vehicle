import numpy as np
import math
import sys

####注意单位，fx像素，x_pixel像素，know_weight单位m。
def dis(fx,x_pixel,know_weight):
    dis = fx*know_weight/x_pixel
    return dis

def object_point_world_position(image, u, v, w, h):
        #框图参数转化成像素值
        u = int(u*image.shape[1])
        v = int(v*image.shape[0])
        w = int(w*image.shape[1])
        h = int(h*image.shape[0])
        
        # #k 为相机内参
        # #p 为相机坐标系到世界坐标系下的外参
        # k = np.array([[582.488, 0.000, 310.280], [
        #             0.000, 579.370, 245.140], [0.000, 0.000, 1.000]])
        # p = np.array([[0, -1, 0, 0], [0, 0, 1, -1], [1, 0, 0, 0], [0, 0, 0, 1]])
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
        # 得到框最下面的中点
        u1 = u
        v1 = v + h / 2

        # 焦距和相机安装高度
        fx = k[0, 0]
        fy = k[1, 1]

        #设置车辆高度和角度
        H = 1.65
        angle_a = 0.05

        angle_b = math.atan((v1 - 480/2) / fy)
        angle_c = angle_b + angle_a
        if angle_c != 0:
            depth = (H / np.sin(angle_c)) * math.cos(angle_b)
        else:
            depth = 150.00
    
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


        c_position[0] = dis(fx,w,0.5)
        if c_position[0] < 0 or c_position[0]>=150:
            c_position[0] = 150
    
        d1 = np.array([c_position[0], c_position[1], c_position[2]], dtype=float)

        return d1


def tran_category(a):
    #针对每一个类别设计不同物体大小

    if a[0] == 0:
        a[0] == 2
    elif a[0] > 0 and a[0] < 18:
        a[0] == 1
    elif a[0] >= 18:
        a[0] == 0
    else:
        raise NotImplementedError

    a=np.append(a,[0.10,0.60,0.60])

    return a
    

def get_speed(image, a):
    #计算初始距离
    d1 = object_point_world_position(image, a[1],a[2],a[3],a[4])
    #利用像素速度计算距离
    d2 = object_point_world_position(image, a[1]+(a[5]/image.shape[1]),a[2]+(a[6]/image.shape[0]),a[3],a[4])
    v = d2 - d1
    a = np.append(a,v)
    return a


def result_to_msg(image, result):
    """
    :params image: 是从 camera_ori 接收到的图像
    :params result_msg:(cls,x,y,z,w,h,l,v_x,v_y)二维
    :params result_msg_nocls:(cls[这里的类别没有变过],pix_x,pix_y,pix_w,pix_h,u_dot,v_dot,x,y,z,w,h,l,v_x,v_y)二维    
    :params result_: 一维的e
    """
    #先排序
    result = result[:, [4, 0, 1, 2, 3, 5, 6]]

    result_msg = []

    #将数组转为一维数组
    for i in result:
        #对应类别
        i = tran_category(i)

        #测距
        d = object_point_world_position(image, i[1], i[2], i[3], i[4])
        f = np.concatenate([i, d])
        #测速
        f = get_speed(image, f)
  
        result_msg.append(f)
    
    result_msg=np.array(result_msg)

    try:
        result_msg = result_msg[:,[0,7,8,9,10,11,12,13,14]]
        result_ = sum(result_msg.tolist(),[])
    except:
        result_ = []
    
    return result_