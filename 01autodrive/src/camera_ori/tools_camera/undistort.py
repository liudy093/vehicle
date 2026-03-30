import cv2 
import numpy as np
DIM=(640, 480)
K=np.array([[210.8474277007198, 0.0, 324.5663784214445], [0.0, 213.37980599910375, 210.26426189456396], [0.0, 0.0, 1.0]])
D=np.array([[0.09614167852695576], [-0.023859828464255536], [0.00992317373781036], [-0.010796907624139316]])
def undistort(img):
    img = cv2.resize(img, DIM)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM,cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)    
    return undistorted_img

cap=cv2.VideoCapture(0)# 换成要打开的摄像头编号
cap.set(cv2.CAP_PROP_FPS, 30)
ret,frame=cap.read()
while ret:
	cv2.imshow('later',frame)
	cv2.imshow('img',undistort(frame))
	ret,frame=cap.read()
	if cv2.waitKey(1)&0xff==27:
		break

cap.release()
cv2.destroyAllWindows()

