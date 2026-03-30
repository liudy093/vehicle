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
        # p = np.array([[0, -1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
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
    if a[0] == 7:
        a[0] = 3
    if a[0] == 8:
        a[0] = 1
    if a[0] == 9:
        a[0] = 2
    if a[0] == 10:
        a[0] = 4
    a=np.append(a,[0.20 ,0.60,0.22])
    return a

def get_speed(image, a):
    #计算初始距离
    d1 = object_point_world_position(image, a[1],a[2],a[3],a[4])
    #利用像素速度计算距离
    d2 = object_point_world_position(image, a[1]+(a[5]/image.shape[1]),a[2]+(a[6]/image.shape[0]),a[3],a[4])
    v = d2 - d1
    a = np.append(a,v)
    return a

def light_detect(light_info,left_Threshold=0.15,right_Threshold=0.85,top_Threshold=0,bottle_Threshold=0.45):
    #输入交通灯信息，输出[X,Y,Z,left,middle,right]01:red,02:yellow,03:green,04:None
    #输入信息[cls,x,y,w,h,X,Y,Z]
    ##定义输出
        light_msg = [0.0,0.0,0.0,0,0,0]
        #参数
        height_Threshold = 0.05 #判断两个交通灯是否并排的高度阈值
        weight_Threshold = 0.15 #判断两个交通灯是否并排的宽度阈值
        #切割相机区域
        light_info = light_info[light_info[:,1]<=right_Threshold]
        light_info = light_info[light_info[:,1]>=left_Threshold]
        light_info = light_info[light_info[:,2]<=bottle_Threshold]
        light_info = light_info[light_info[:,2]>=top_Threshold]
        #超过三个,取高的三个
        if light_info.shape[0] >= 4:
            light_info = light_info[np.argsort(light_info[:,2])]
            light_info = light_info[[-1,-2,-3],:]
        #对图像x轴的红绿灯进行排序(从小到大)
        light_info = light_info[np.argsort(light_info[:,1])]
        #检测到三个红绿灯
        if light_info.shape[0] == 3:
            light_info_three = light_info[abs(light_info[:,2]-np.mean(light_info[:,2]))<=height_Threshold]
            #三个红绿灯不是一组
            if light_info_three.shape[0] != 3:
                #三个红绿灯有两个是一组
                light_info_ = np.array([])
                for i in range (0,light_info.shape[0]-1):
                    for j in range(i+1,light_info.shape[0]):
                        if abs(light_info[i][1] - light_info[j][1])<weight_Threshold and abs(light_info[i][2] - light_info[j][2])<height_Threshold:
                            light_info_ = light_info[[i,j],:]
                if light_info_.shape[0] == 2:
                    light_msg[0:3] = np.sum(light_info_[:,5:8],axis=0)/light_info_.shape[0]
                    light_msg[3] = light_info_[0][0]
                    light_msg[4] = light_info_[1][0]
                    light_msg[5] = 4
                #三个相机都不是一组
                else :
                    max_anchor = 0
                    I = 0
                    for i in range(0,light_info.shape[0]):
                        if light_info[i][3]*light_info[i][4] > max_anchor:
                            max_anchor = light_info[i][3]*light_info[i][4]
                            I = i
                    light_msg[0:3] = light_info[I,5:8]
                    light_msg[3] = 4
                    light_msg[4] = light_info[I][0]
                    light_msg[5] = 4
            #三个红绿灯一组
            elif light_info_three.shape[0] == 3:
                light_msg[0:3] = np.sum(light_info[:,5:8],axis=0)/light_info.shape[0]
                light_msg[3] = light_info[0][0]
                light_msg[4] = light_info[1][0]
                light_msg[5] = light_info[2][0]
        #检测到两个红绿灯
        elif light_info.shape[0] == 2:
            #两个红绿灯为一组，默认两个灯是直行和右转，一般直行和左转是一个灯
            if abs(light_info[0][1] - light_info[1][1])<weight_Threshold and abs(light_info[0][2] - light_info[1][2])<height_Threshold:
                light_info = light_info
                light_msg[0:3] = np.sum(light_info[:,5:8],axis=0)/light_info.shape[0]
                light_msg[3] = light_info[0][0]
                light_msg[4] = light_info[1][0]
                light_msg[5] = 4
            #两个不为一组
            #!此处用的是框面积大的为路口红绿灯
            else:
                max_anchor = 0
                I = 0
                for i in range(0,light_info.shape[0]):
                    if light_info[i][3]*light_info[i][4] > max_anchor:
                        max_anchor = light_info[i][3]*light_info[i][4]
                        I = i
                light_msg[0:3] = light_info[I,5:8]
                light_msg[3] = 4
                light_msg[4] = light_info[I][0]
                light_msg[5] = 4
        #检测到一个红绿灯
        elif light_info.shape[0] == 1:
            light_msg[0:3] = np.sum(light_info[:,5:8],axis=0)/light_info.shape[0]
            light_msg[3] = 4
            light_msg[4] = light_info[0][0]
            light_msg[5] = 4
        light_msg[0:3] = [float(i) for i in light_msg[0:3]]
        light_msg[3:6] = [int(i) for i in light_msg[3:6]]
        return light_msg

def result_to_msg(image, result):
    """
    :params image: 是从 camera_ori 接收到的图像
    :params result_msg:(cls[这里的类别没有变过],pix_x,pix_y,pix_w,pix_h,u_dot,v_dot,x,y,z,w,h,l,v_x,v_y)二维
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

    result_msg = result_msg[:,[0,1,2,3,4,10,11,12]]
    # print(result_msg,"!!!!!!!!!!!!!!!!!!")
    result_ = light_detect(result_msg,left_Threshold=0.25,right_Threshold=0.75,top_Threshold=0,bottle_Threshold=0.6)


    return result_