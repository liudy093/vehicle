import torch, cv2
from easydict import EasyDict as edict
import yaml
import os
from model.model import parsingNet
import scipy.special
import numpy as np
import torchvision.transforms as transforms
import time
import matplotlib.pyplot as plt
from utils.calculate_dis import dis_lane


def load_yaml(path):
    with open(path) as f:
        cfg = edict(yaml.safe_load(f))
    return cfg


cfg = load_yaml(os.path.join(os.getcwd(), "src/%s/%s/"%("lane_recognition", "lane_recognition"), "detectLane", "configs/culane.yaml"))
cfg.test_model = os.path.join(os.getcwd(), "src/%s/%s/"%("lane_recognition", "lane_recognition"), "detectLane", "weights/culane_18.pth")

class ZeroOneNormalize(object):
    """
    对图像像素位于[0, 255]之间, 缩放到 [0, 1] 之间
    """
    def __call__(self, img):
        return img.float().div(255)



def lane_detect_init():
    """
    通过配置字典导入模型
    :params cfg 模型配置的 easydict 形式
    :return net 是依据配置文件进行网络模型的实例化对象, img_transforms 是对图像进行预处理的类的实例化对象
    """
    global cfg
    net = parsingNet(pretrained = False, backbone=cfg.backbone,cls_dim = (cfg.griding_num+1,cfg.cls_num_per_lane,4),
                    use_aux=False).cuda().half()

    state_dict = torch.load(cfg.test_model, map_location='cuda:0')['model']
    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v

    net.load_state_dict(compatible_state_dict, strict=False)

    img_transforms = transforms.Compose([
        transforms.Resize((288, 800), antialias=True),
        ZeroOneNormalize(),
        transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))])

    return net, img_transforms


def lane_detect_main(net, imgs, img_transforms):
    """
    车道线检测的主函数, 车道线检测后的图片(目前刚实现到这里)
    :param cfg 是配置文件的 easydict 
    :param net 是网络模型的实例化
    :param imgs 是传入的图片, 可以是图片的绝对路径, 也可以是 cv2.imread 读取后的 BGR 格式
    :img_transforms 是对图像进行预变换的实例化对象
    :return img_origin 是图片进行车道线标注后的结果, 仍然是 BGR 的形式
    :return lane_point 是车前的两条车道线的点的坐标, 是一个18*4大小的array数组 (x1, y1, x2, y2), 对应的图像的坐标原点在左上角
    lane_point 中的 y 坐标是递减的, 但不是均匀间隔
    """
    global cfg

    if isinstance(imgs, str):
        img_origin = cv2.imread(imgs)  # BGR
        assert img_origin is not None, 'Image Not Found ' + imgs
    else:
        img_origin = imgs
    
    img_h, img_w = img_origin.shape[:-1]  # 返回图像的原始长度

    img0 = np.ascontiguousarray(img_origin[:, :, ::-1].transpose(2,0,1))  # BGR -> RGB 并且 h,w,c -> c, h, w
    img0 = torch.tensor(img0, dtype=torch.uint8, device=torch.device('cuda:0'))

    img = img_transforms(img0)
    img = img.unsqueeze(0).half()

    out = net(img)

    col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
    col_sample_w = col_sample[1] - col_sample[0]

    out_j = out[0].data.cpu().numpy()
    out_j = out_j[:, ::-1, :]
    prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)  # 不取第一个二维矩阵
    idx = np.arange(cfg.griding_num) + 1
    idx = idx.reshape(-1, 1, 1)
    loc = np.sum(prob * idx, axis=0)
    out_j = np.argmax(out_j, axis=0)
    loc[out_j == cfg.griding_num] = 0
    out_j = loc

    color_list = [[255,0,0],[0,255,0],[0,0,255],[255,255,255]]
    lane_point = np.zeros((18, 4))  # 存放车前的两条车道线的点的坐标
    for i in range(out_j.shape[1]):
        if i ==0 or i==3: continue
        if np.sum(out_j[:, i] != 0) > 2:  # 检测出的这条车道线上必须至少有两个点才行
            for k in range(out_j.shape[0]):
                if out_j[k, i] > 0:
                    ppp = [int(out_j[k, i] * col_sample_w * img_w / 800) - 1, int(img_h * (cfg.culane_row_anchor[cfg.cls_num_per_lane-1-k]/288)) - 1 ]
                    # cv2.circle(img_origin,ppp,5,(0,255,0),-1)
                    cv2.circle(img_origin,tuple(ppp),5,color_list[i],-1)

                    if i == 1 :
                        if ppp[1] <= 347:  # 过滤掉离车较远的点
                            ppp[0] = 0
                            ppp[1] = 0
                        lane_point[k][0] = ppp[0]
                        lane_point[k][1] = ppp[1] 
                        
                    elif i == 2:
                        if ppp[1] <= 347:  # 过滤掉离车较远的点
                            ppp[0] = 0
                            ppp[1] = 0
                        lane_point[k][2] = ppp[0]
                        lane_point[k][3] = ppp[1]
                    else:
                        pass

    return img_origin, lane_point


if __name__ == "__main__":
    cfg = load_yaml("/home/self_driving/.local/lib/python3.8/site-packages/detectLane/configs/culane.yaml")

    cap = cv2.VideoCapture("/home/self_driving/.local/lib/python3.8/site-packages/detectLane/inference/video_lane/hard.mp4")

    print('start testing...')
    net, img_transforms = lane_detect_init()

    while True:
        ret, frame = cap.read()
        imgs = cv2.resize(frame,(640,480))
        cv2.imshow("123",imgs)
        cv2.waitKey(0)
        with torch.no_grad():
            t0 = time.time()
            vis_lane, point_lane = lane_detect_main(net, imgs, img_transforms)
            dis = dis_lane(point_lane, vis_lane)
            print(vis_lane.shape, point_lane)
            t1 = time.time()
            print(t1-t0, 1/(t1-t0), "-------总时间---------")
            print("------------------------------------")