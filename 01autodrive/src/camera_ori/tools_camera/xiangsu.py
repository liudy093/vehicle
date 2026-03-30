import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
 
'''
重点问题：
1、图像矩阵记录不要直接赋值，否则像当于将多个变量指向一个地址图像，等同np.view()
2、记录图像矩阵数据，使用img.copy() #使用id（）,可查看是不是一个地址
3、cv2.setMouseCallback的调用函数中event是阶跃信号，flags是状态信号
4、若想保存回调函数中的某些状态，需要使用全局变量，否则无法记录
5、最终显示的都是一个图像变量矩阵
'''

# 图片路径
img_path = '/home/warren/tools_camera/picture1/img2.jpg'
img = cv2.imread(img_path,-1)

cv2.namedWindow("image")
create_new_flag = 1
img_rem = img.copy() # 存储一个图像矩阵
img_mid = img.copy()  # 存储一次完成点击和松开动作的图像矩阵
# print(id(img_rem),id(img_mid))

init_point = (0,0)
lbutton_state = 0  # 左键状态标志

def my_def(event, x, y, flags, param=None):
    global create_new_flag
    global img_rem
    global img_mid
    global init_point
    global lbutton_state
    # 按下左键，瞬间触发一次事件
    if event == cv2.EVENT_LBUTTONDOWN:  
        if create_new_flag == 1: # 点完右键后第一次点左键
            #重新显示矩阵
            img_rem = cv2.imread(img_path,-1).copy()
        init_point = (x,y)     # 记录瞬间点击右键的位置
        #记录原始点位
        # 位置信息p，和像素值信息p_v
        xy = f"{init_point}"  if param else f"{init_point},{str(img[x][y])}" 
        cv2.circle(img_rem, (x, y), 5, ( 0,255), thickness=-1)
        cv2.putText(img_rem, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.5, (0,255, 0), thickness=2)
        img_mid  = img_rem.copy()   # 记录想要保存的中间状态
        create_new_flag = 0  # 不进行图片的再次刷新
        lbutton_state = 1

    # 按下左键并滑动，不松开就持续触发
    elif lbutton_state == 1 and flags == cv2.EVENT_FLAG_LBUTTON :
        cur_point = (x,y)
        if not (event == cv2.EVENT_LBUTTONUP): # 左键未松开，一直被清除
            #重新显示矩阵
            img_rem = img_mid.copy() # 不能直接赋值操作，会直接认为是同一地址的数据
            # print(id(img_rem),id(img_mid))
            # cv2.imshow("mid", img_mid) 

        if lbutton_state == 1:
            cv2.rectangle(img_rem, init_point, cur_point, (0,0,255), 2)
            cv2.putText(img_rem,str(cur_point), (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.5, (0,255, 0), thickness=2)
        if event == cv2.EVENT_LBUTTONUP:  # 松开左键
            lbutton_state = 0
            img_mid = img_rem

    elif event == cv2.EVENT_RBUTTONDOWN:  # 右键按下执行的动作
        create_new_flag = 1   # 判断是否重新打开一个图片矩阵
        img_rem = cv2.imread(img_path,-1).copy()

    cv2.imshow("image", img_rem)  # 显示的是图片矩阵



cv2.setMouseCallback("image", my_def,1)
cv2.imshow("image", img)  #只是用于开始图片的展示
cv2.waitKey(0)
