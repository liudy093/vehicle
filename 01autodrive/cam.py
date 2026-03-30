import cv2 

cap = cv2.VideoCapture("/dev/video2")

out = cv2.VideoWriter("output.mp4", cv2.VideoWriter_fourcc(*'MP4V'),20.0,(1920,1080))


while(1):
    ret,frame = cap.read()
    out.write(frame)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1)== ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
