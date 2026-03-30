import cv2 
import numpy as np

k=np.array( [[249.40021034 ,  0.        , 346.22028796],
		[0.  ,       248.06291615, 214.8187893],
       [0.        ,   0.         ,  1.]])

d=np.array([-0.24419889 , 0.05199839 ,-0.00136252 , 0.00067625 ,-0.00504383])

def undistort(frame):
	h,w=frame.shape[:2]
	mapx,mapy=cv2.initUndistortRectifyMap(k,d,None,k,(w,h),5)
	return cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)

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

