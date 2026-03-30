import cv2 

def captureVideoFromCamera():
    cap = cv2.VideoCapture(-1)
    #初始化视频参数
    WIDTH = 1920
    HEIGHT = 1080
    FILENAME = r'utils/camera_videos/1.avi'

    FPS = 24
    cap.set(cv2.CAP_PROP_FPS, 24)
    # 建议使用XVID编码,图像质量和文件大小比较都兼顾的方案
    fourcc = cv2.VideoWriter_fourcc(*'XVID')

    out = cv2.VideoWriter(FILENAME, fourcc=fourcc, fps=FPS,frameSize=(WIDTH,HEIGHT))

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # 逐帧捕获
        ret, frame = cap.read()
        # 如果正确读取帧，ret为True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        #看情况是否进行水平翻转
        # frame = cv2.flip(frame, 1)  # 水平翻转
        ret = out.write(frame)
        # 显示结果帧e
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):  break
    # 完成所有操作后，释放捕获器
    out.release()
    cap.release()
    cv2.destroyAllWindows()

captureVideoFromCamera()
