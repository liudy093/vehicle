import cv2
 
# 使用OpenCV打开视频流
cap = cv2.VideoCapture("/dev/video2")
 

while True:
    # 读取视频帧
    ret, frame = cap.read()
 
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
 
    # 显示视频帧
    cv2.imshow('Video Stream', frame)
 
    # 按 'q' 退出循环
    if cv2.waitKey(1) == ord('q'):
        break
 
# 释放资源和关闭窗口
cap.release()
cv2.destroyAllWindows()