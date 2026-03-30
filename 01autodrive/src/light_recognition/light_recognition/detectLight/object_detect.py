import os
from models.sign_net import SignNet
from torchvision.transforms import transforms
from sort.sort import *
from utils.datasets import *
from models.experimental import *
from utils.vis_utils import draw_boxes
from easydict import EasyDict as edict
from PIL import Image
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


def detect_init(imgsz=640, conf_thres=0.35, iou_thres=0.55, sort_max_age=5, sort_min_hits=2, sort_iou_thresh=0.3):
    """
    :params imgsz: 设定图片长边被缩放后的大小(图像两边都是成比例缩放)
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
                 "src/%s/%s/"%("light_recognition", "light_recognition"), "detectLight","weights/best_yolov5s_bdd_prew.pt")})

    device = torch.device('cuda:0')
    half = device.type != 'cpu'  # half precision only supported on CUDA
    opt.device = device

    # Load model
    model = attempt_load(
        opt.weight_path, map_location=device)  # load FP32 model
    names = model.module.names if hasattr(
        model, 'module') else model.names  # 得到类别名称

    names.extend(["unlabel", 'pl5', 'pl10', 'pl15', 'pl20', 'pl30', 'pl40', 'pl50',
                 'pl60', 'pl70', 'pl80', 'pl90', 'pl100', 'pl110', 'pl120', 'pl35', 'ps'])
    # colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
    # colors = [random.randint(0, 255) for _ in range(3)]

    if half:
        model.half()  # to FP16

    data_transfomrs = transforms.Compose([
        transforms.Resize((32, 32)),
        ZeroOneNormalize(),
        transforms.Normalize((0.3337, 0.3064, 0.3171),
                             (0.2672, 0.2564, 0.2629))
    ])

    # Initialize SORT, 初始化 SORT
    sort_tracker = Sort(max_age=sort_max_age,
                        min_hits=sort_min_hits,
                        iou_threshold=sort_iou_thresh)  # {plug into parser}

    signmodel = SignNet(17).cuda()
    signmodel.load_state_dict(torch.load(
        os.path.join(os.getcwd(), "src/%s/%s/"%("light_recognition", "light_recognition"), 
        "detectLight","weights/append_model_epoch23_train0.975_val0.981.pth")))

    return model, sort_tracker, opt, names, data_transfomrs, signmodel


def detect_ros(image, model, sort_tracker, opt, class_names, data_transfroms, signmodel):
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
    0:person, 1:rider, 2:car, 3:bus, 4:truck, 5:bike, 6:motor, 
    7:tl_green, 8:tl_red, 9:tl_yellow, 10:tl_none, 11:t_sign, 12:train
    """
    img0 = np.array(image, dtype="u1")

    img = letterbox(img0, new_shape=opt.img_size)[0]

    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
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
    if det is None: 
        det = None
    else:
        det = det[det[:,5]>=7]
        det = det[det[:,5]<=10]

    object_info = []
    if det is not None and len(det):
        # Rescale boxes from img_size to im0 size
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        det[:, :4] = scale_coords(
            img.shape[2:], det[:, :4], img0.shape).round()

        dets_to_sort = np.empty((0, 6))
        # Pass detections to SORT
        # NOTE: We send in detected object class too
        for x1, y1, x2, y2, conf, detclass in det.cpu().detach().numpy():
            dets_to_sort = np.vstack(
                (dets_to_sort, np.array([x1, y1, x2, y2, conf, detclass])))

        # Run SORT
        tracked_dets = sort_tracker.update(dets_to_sort)

        sign_dets = tracked_dets[tracked_dets[:, 4] == 11]
        for i in range(len(sign_dets)):
            crop = cut_img(image, sign_dets[i][0], sign_dets[i][2], sign_dets[i][1], sign_dets[i][3])
            if crop.size == 0: 
                sign_dets[i][4] =  13
                continue
            # print(crop.shape, "-------------------")
            # PIL_crop = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            PIL_crop = np.ascontiguousarray(crop[:, :, ::-1].transpose(2,0,1))  # BGR -> RGB 并且 h,w,c -> c, h, w
            PIL_crop = torch.tensor(PIL_crop, dtype=torch.uint8, device=torch.device('cuda:0'))

            sign_img = data_transfroms(PIL_crop)
            output = signmodel(sign_img.unsqueeze(0).cuda())
            sign_dets[i][4] =  13 + output.argmax().item()


        tracked_dets[tracked_dets[:, 4] == 11] = sign_dets
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

    img_path = "/home/self_driving/.local/lib/python3.8/site-packages/detectTrace/inference/test_one/stop_1.jpg"

    model, sort_tracker, opt, names, data_transfroms, signmodel = detect_init()
    image = cv2.imread(img_path)
    with torch.no_grad():
        det_result, vis_img = detect_ros(
            image, model, sort_tracker, opt, names, data_transfroms, signmodel)
        cv2.imshow("1", vis_img)
        cv2.waitKey(0)
