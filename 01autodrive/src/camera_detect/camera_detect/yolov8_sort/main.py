import json
import cv2
import time
import torch
from yolo import YOLO
from PIL import Image
import numpy as np

if __name__ == "__main__":
    img = "/home/wyz/WIDC2024/data/camera_video/18.6_0.24_25.7_-1.0.png"
    # img = "/home/wyz/WIDC2024/data/camera_video/12.6_1.5_18.1_5.6.png"
    json_file = "/home/wyz/WIDC2024/src/camera_detect/camera_detect/config.json"

    #--------------- 从json文件读入配置 --------------------#
    with open(json_file, 'r') as jsonfile:
        config = json.load(jsonfile)
    #------------------------------------------------------#
    
    internal_param = np.array(config["camera_param"]["intrinsic_matrix"])
    distortion_cofficients = np.array(config["camera_param"]["distortion_cofficients"])

    external_param = np.array(config["camera_param"]["forward_lidar_to_camera"])
    external_param = np.hstack((external_param[:, :-1].transpose(), external_param[:, -1].reshape(-1,1)))
    external_param = np.vstack((external_param, np.array([0., 0., 0., 1.])))
    config["detect_config"]["internal_param"] = internal_param
    config["detect_config"]["external_param"] = external_param
    config["detect_config"]["camera_height"] = config["camera_param"]["camera_height"]
    config["detect_config"]["camera_angle"] = config["camera_param"]["camera_angle"]

    yolov8 = YOLO(config["detect_config"])
    image = cv2.resize(cv2.imread(img, cv2.COLOR_BGR2RGB), tuple(config["orin_image"]["image_size"]))
    # 读入的图像格式是 RGB
    with torch.no_grad():
        t1 = time.time()
        (result, r_image) = yolov8.detect_image(image)
        print(result)
        cv2.imshow("ok", r_image)
        cv2.waitKey(0)
        k = cv2.waitKey(0)&0xFF
        #按下ESC时关闭图片窗口
        if k == 27:
            cv2.destroyAllWindows()
    
    # enable_detect, enable_track, enable_vis 均为 true 的时候, result 为
    # num_object * 9    xmin, ymin, xmax, ymax, class, id, lx, ly, lz
    # [[ 3.7000000e+02  3.1000000e+02  3.8000000e+02  3.3900000e+02
    # 0.0000000e+00  2.0000000e+00  2.6408831e+01 -1.0226922e+00
    # -4.8594362e-01]
    # [ 3.3500000e+02  3.2400000e+02  3.4800000e+02  3.6200000e+02
    # 0.0000000e+00  1.0000000e+00  1.9674515e+01  2.3138981e-01
    # -4.7872674e-01]]

    # enable_detect, enable_vis 均为 true 的时候, enable_track 为 false 的时候, result 为
    # num_object * 8    xmin, ymin, xmax, ymax, class, lx, ly, lz
    # [[ 3.3500000e+02  3.2400000e+02  3.4800000e+02  3.6200000e+02
    #    0.0000000e+00  1.9674515e+01  2.3138981e-01 -4.7872674e-01]
    #  [ 3.7000000e+02  3.1000000e+02  3.8000000e+02  3.3900000e+02
    #    0.0000000e+00  2.6408831e+01 -1.0226922e+00 -4.8594362e-01]]

    # enable_detect 为 false  
