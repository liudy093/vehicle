import os
import torch
from models.sign_net import SignNet
from torchvision.transforms import transforms
from sort.sort import *
from utils.datasets import xyxy2xywh
from models.experimental import attempt_load
from utils.vis_utils import draw_boxes
from easydict import EasyDict as edict
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression, scale_boxes
import cv2
import warnings
warnings.filterwarnings("ignore")


# 裁剪图像
def cut_img(img, x0, x1, y0, y1):
    cropped = img[int(y0):int(y1), int(x0):int(x1)]
    return cropped

class ZeroOneNormalize(object):
    """
    对图像像素位于[0, 255]之间, 缩放到 [0, 1] 之间
    """
    def __call__(self, img):
        return img.float().div(255)


def detect_init(imgsz, conf_thres=0.25, iou_thres=0.45, 
                sort_max_age=5, sort_min_hits=2, sort_iou_thresh=0.3):
    """
    :params imgsz: 输入的原始图像的大小
    :params conf_thres: 是置信度阈值, 对候选框进行NMS时的阈值, 如果低于阈值, 则不进行候选
    :param iou_thres: 是iou阈值, 也是对候选框进行NMS时的阈值。
    :param sort_max_age: 跟踪的生命周期, 多个帧目标不出现, 则丢弃跟踪
    :param sort_min_hits: 当检测到多个目标后开始跟踪
    :param sort_iou_thresh: 跟踪的 iou_thresh 阈值

    依据参数, 初始化检测和跟踪模型
    """
    # Initialize
    opt = edict({"img_size": imgsz, "conf_thres": conf_thres, "iou_thres": iou_thres,
                 "weight_path":  os.path.join(os.getcwd(), 
                 "src/%s/%s/"%("sign_recognition", "sign_recognition"), "detectSign","weights/weights_2023_2_4.pt")})

    # opt = edict({"img_size": imgsz, "conf_thres": conf_thres, "iou_thres": iou_thres,
    #             "weight_path":  "weights/weights_2023_2_3.pt"})

    device = torch.device('cuda:0')
    half = device.type != 'cpu'  # half precision only supported on CUDA
    opt.device = device

    # Load model
    model = attempt_load(
        opt.weight_path, device=device)  # load FP32 model
    names = model.module.names if hasattr(
        model, 'module') else model.names  # 得到类别名称

    if half:
        model.half()  # to FP16

    # Initialize SORT, 初始化 SORT
    sort_tracker = Sort(max_age=sort_max_age,
                        min_hits=sort_min_hits,
                        iou_threshold=sort_iou_thresh)  # {plug into parser}


    return model, sort_tracker, opt, names


def detect_ros(image, model, sort_tracker, opt, class_names):
    """
    :param image: 必须是BGR格式的图片。
    :param model: 目标检测的模型
    :param sort_tracker: 跟踪模型
    :param opt: 配置参数
    :param class_names: 每个类别的名字, 可视化图像使用

    :return object_info (x,y,w,h,cls,u_dot, v_dot, s_dot, object_id) 
    是一个大列表, 其中每个元素是一个小列表, 其中包含9个数字.
    x, y , w, h都是比例

    u_dot: time derivative of x_center in pixels
    v_dot: time derivative of y_center in pixels
    s_dot: time derivative of scale (area of bbox) in pixels
    objetc_id: 是检测到的目标 id

    :return img0: 与输入图像的大小相同, 但是做了可视化处理

    输入图像, BGR格式, 进行目标检测和目标跟踪

    类别编号的对应值
    {0: 'ps', 1: 'pl5', 2: 'pl10', 3: 'pl15', 4: 'pl20', 5: 'pl25', 6: 'pl30', 
    7: 'pl35', 8: 'pl40', 9: 'pl50', 10: 'pl60', 11: 'pl65', 12: 'pl70', 13: 'pl80',
    14: "pl90", 15:"pl100", 16:"pl110", 17:"pl120", 18:"unlabel"}

    其中 0 表示停车让行, 1-17 是限速标志, 18 是其余标识牌
    """
    img0 = np.array(image, dtype="u1")  # img0 (368, 640, 3), BGR
    imgsz = check_img_size(imgsz=opt.img_size, s=model.stride.item())  # (384, 640, 3), BGR

    img = letterbox(img0, new_shape=imgsz, stride=model.stride.item(), auto=True)[0]  # (384, 640, 3), BGR

    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x384x640
    img = np.ascontiguousarray(img)

    # Run inference
    img = torch.from_numpy(img).to(opt.device)
    img = img.half()
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    pred = model(img, augment=False)[0]
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres)

    det = pred[0]
  
    object_info = []
    if det is not None and len(det):
        # Rescale boxes from img_size to im0 size
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        det[:, :4] = scale_boxes(
            img.shape[2:], det[:, :4], img0.shape).round()

        dets_to_sort = np.empty((0, 6))
        # Pass detections to SORT
        # NOTE: We send in detected object class too
        for x1, y1, x2, y2, conf, detclass in det.cpu().detach().numpy():
            dets_to_sort = np.vstack(
                (dets_to_sort, np.array([x1, y1, x2, y2, conf, detclass])))

        # Run SORT
        tracked_dets = sort_tracker.update(dets_to_sort)

        # draw boxes for visualization
        if len(tracked_dets) > 0:
            bbox_xyxy = tracked_dets[:, :4]
            identities = tracked_dets[:, 8]
            categories = tracked_dets[:, 4]
            draw_boxes(img0, bbox_xyxy, identities, categories, class_names)

        for x1, y1, x2, y2, *info in tracked_dets:
            xywh = (xyxy2xywh(torch.tensor([x1, y1, x2, y2]).view(
                1, 4)) / gn).view(-1).tolist()  # normalized xywh
            object_info.append(xywh + info)

    return object_info, img0


if __name__ == '__main__':

    img_path = "/home/yzwang/PythonProject/yolov5-master/images/1.jpg"

    model, sort_tracker, opt, names = detect_init(imgsz=(368, 640))
    image = cv2.imread(img_path)

    with torch.no_grad():
        det_result, vis_img = detect_ros(image, model, sort_tracker, opt, names)
        print(det_result)
        # cv2.imshow("1", vis_img)
        # cv2.waitKey(0)
        cv2.imwrite("ok.jpg", vis_img)
