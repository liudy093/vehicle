import colorsys
import cv2
import numpy as np
import torch
import torch.nn as nn
import math
from PIL import ImageDraw, ImageFont, Image

from nets.yolo import YoloBody
from utils.utils import (cvtColor, get_classes, preprocess_input,
                         resize_image, show_config)
from utils.utils_bbox import DecodeBox

# -------------sort-------------------#
from sort import *
# ------------------------------------#


class DistanceEstimation:
    def __init__(self, weight_img, height_img, internal_param, external_param, camera_height, camera_angle):
        # weight_img, height_img 是图像的宽度, 高度
        self.W = weight_img
        self.H = height_img

        self.internal_param = internal_param  # 3*3
        self.external_param = external_param  # 4*4

        self.camera_height = camera_height          # 相机的安装高度
        self.camera_angle = camera_angle            # 相机相对于地面 angle

    def object_point_world_position(self, u, v, w, h, p, k):
        # 选择框的下边缘的中心点
        u1 = u + w/2
        v1 = v + h

        fx = k[0, 0]
        fy = k[1, 1]
        angle_b = math.atan((v1 - self.H / 2) / fy)
        angle_c = angle_b + (self.camera_angle*math.pi/360)
        depth = (self.camera_height / np.sin(angle_c)) * math.cos(angle_b)

        k_inv = np.linalg.inv(k)  # 3*3
        p_inv = np.linalg.inv(p)  # 4*4
        point_c = np.array([u1, v1, 1])
        point_c = np.transpose(point_c)  # 3*1
        c_position = np.matmul(k_inv, depth * point_c)
        c_position = np.append(c_position, 1)
        c_position = np.transpose(c_position)
        c_position = np.matmul(p_inv, c_position)
        lidar_xyz = np.array((-c_position[1], -c_position[2], c_position[0]), dtype=float)

        return lidar_xyz

    def get_distance(self, pred, ymin_xmin=True):
        if ymin_xmin:
            lidar_xyz = self.object_point_world_position(pred[1], pred[0], pred[3]-pred[1], pred[2]-pred[0],
                                                         self.external_param, self.internal_param)
        else:
            lidar_xyz = self.object_point_world_position(pred[0], pred[1], pred[2]-pred[0], pred[3]-pred[1],
                                                         self.external_param, self.internal_param)

        return lidar_xyz


class YOLO(object):
    _defaults = {
        # --------------------------------------------------------------------------#
        #   使用自己训练好的模型进行预测一定要修改model_path和classes_path！
        #   model_path指向logs文件夹下的权值文件，classes_path指向model_data下的txt
        #
        #   训练好后logs文件夹下存在多个权值文件，选择验证集损失较低的即可。
        #   验证集损失较低不代表mAP较高，仅代表该权值在验证集上泛化性能较好。
        #   如果出现shape不匹配，同时要注意训练时的model_path和classes_path参数的修改
        # --------------------------------------------------------------------------#
        "model_path": 'weights/ep150-loss2.623-val_loss2.689.pth',
        "classes_path": 'model_data/traffic_cone.txt',
        # ---------------------------------------------------------------------#
        #   输入图片的大小，必须为32的倍数。
        # ---------------------------------------------------------------------#
        "input_shape": [640, 640],
        # ------------------------------------------------------#
        #   所使用到的yolov8的版本：
        #   n : 对应yolov8_n
        #   s : 对应yolov8_s
        #   m : 对应yolov8_m
        #   l : 对应yolov8_l
        #   x : 对应yolov8_x
        # ------------------------------------------------------#
        "phi": 's',
        # ------------------------------------------------------#
        # 是否开启融合
        # ------------------------------------------------------#
        "model_fuse": False,
        # ---------------------------------------------------------------------#
        #   只有得分大于置信度的预测框会被保留下来
        # ---------------------------------------------------------------------#
        "confidence": 0.5,
        # ---------------------------------------------------------------------#
        #   非极大抑制所用到的nms_iou大小
        # ---------------------------------------------------------------------#
        "nms_iou": 0.3,
        # ---------------------------------------------------------------------#
        #   是否使用跟踪算法
        # ---------------------------------------------------------------------#
        "enable_track": True,
        # ---------------------------------------------------------------------#
        "enable_vis": True,
        # ---------------------------------------------------------------------#
        "enable_distance": True,
        # ---------------------------------------------------------------------#
        # ---------------------------------------------------------------------#
        #   该变量用于控制是否使用letterbox_image对输入图像进行不失真的resize，
        #   在多次测试后，发现关闭letterbox_image直接resize的效果更好
        # ---------------------------------------------------------------------#
        "letterbox_image": False,
        # -------------------------------#
        #   是否使用Cuda
        #   没有GPU可以设置成False
        # -------------------------------#
        "cuda": True,

        # ---------追踪算法的配置---------#
        "sort_max_age": 5,
        "sort_min_hits": 2,
        "sort_iou_thresh": 0.2
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    # ---------------------------------------------------#
    #   初始化YOLO
    # ---------------------------------------------------#
    def __init__(self, kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
            self._defaults[name] = value

        # ---------------------------------------------------#
        #   获得种类和先验框的数量
        # ---------------------------------------------------#
        self.class_names, self.num_classes = get_classes(
            os.path.join(os.path.dirname(__file__), self.classes_path))
        self.bbox_util = DecodeBox(
            self.num_classes, (self.input_shape[0], self.input_shape[1]))

        # ---------------------------------------------------#
        #   画框设置不同的颜色
        # ---------------------------------------------------#
        hsv_tuples = [(x / self.num_classes, 1., 1.)
                      for x in range(self.num_classes)]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))

        # --------------- 初始化模型并且导入权重 ---------------#
        self.generate()

        # ----------------跟踪代码-------------------#
        if self.enable_track:
            self.sort_tracker = Sort(max_age=self.sort_max_age,
                                     min_hits=self.sort_min_hits, iou_threshold=self.sort_iou_thresh)

        # ---------------------测距代码------------------------#
        if self.enable_distance:
            self.distanceEstimate = DistanceEstimation(640, 640, self.internal_param, self.external_param, self.camera_height, self.camera_angle)

    # ---------------------------------------------------#
    #   生成模型
    # ---------------------------------------------------#
    def generate(self):
        # ---------------------------------------------------#
        #   建立yolo模型，载入 yolo 模型的权重
        # ---------------------------------------------------#
        self.net = YoloBody(self.input_shape, self.num_classes, self.phi)

        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(os.path.join(
            os.path.dirname(__file__), self.model_path), map_location=device))
        if self.model_fuse:
            self.net = self.net.fuse().eval()
        else:
            self.net = self.net.eval()
        if self.cuda:
            self.net = self.net.cuda()

    # ---------------------------------------------------#
    #   检测图片, 返回的 detect_image 中 共有NX6, 分别为 xmin,ymin,xmax,ymax,class,id
    # ---------------------------------------------------#
    def detect_image(self, image):
        # ---------------------------------------------------#
        #   计算输入图片的高和宽
        # ---------------------------------------------------#
        image_shape = image.shape[:-1]
        # ---------------------------------------------------------#
        #   添加上batch_size维度, 并且将数据归一化
        #   h, w, 3 => 3, h, w => 1, 3, h, w
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(
            preprocess_input(image), (2, 0, 1)), 0)

        images = torch.from_numpy(image_data).float()
        if self.cuda:
            images = images.cuda()
        # ---------------------------------------------------------#
        #   将图像输入网络当中进行预测！
        # ---------------------------------------------------------#
        outputs = self.net(images)
        outputs = self.bbox_util.decode_box(outputs)
        # ---------------------------------------------------------#
        #   将预测框进行堆叠，然后进行非极大抑制
        # ---------------------------------------------------------#
        results = self.bbox_util.non_max_suppression(outputs, self.num_classes, self.input_shape,
                                                     image_shape, self.letterbox_image, conf_thres=self.confidence, nms_thres=self.nms_iou)

        if results[0] is None:
            return ([], image)

        if self.enable_track:
            # ..................USE TRACK FUNCTION....................
            # pass an empty array to sort
            dets_to_sort = np.empty((0, 6))

            # NOTE: We send in detected object class too
            for y1, x1, y2, x2, conf, detclass in results[0]:
                dets_to_sort = np.vstack((dets_to_sort,
                                          np.array([x1, y1, x2, y2, conf, int(detclass)])))

            # Run SORT
            tracked_dets = self.sort_tracker.update(dets_to_sort)
            detect_result = np.hstack(
                (tracked_dets[:, [0]], tracked_dets[:, [1]], tracked_dets[:, [2]], tracked_dets[:, [3]], 
                        tracked_dets[:, [4]], tracked_dets[:, [-1]]))

            if self.enable_distance:
                object_distance = np.zeros(
                    (len(detect_result), 3), dtype=np.float32)
                for i in range(len(detect_result)):
                    object_distance[i] = self.distanceEstimate.get_distance(detect_result[i], ymin_xmin=False)

                detect_result = np.concatenate([detect_result.astype('int32').astype(
                    "float32"), object_distance], axis=1)    # ymin, xmin, ymax, xmax, class, id, lx, ly, lz

            if self.enable_vis:
                top_boxes = np.hstack((tracked_dets[:, [1]], tracked_dets[:, [
                                      0]], tracked_dets[:, [3]], tracked_dets[:, [2]])).astype('int32')
                top_label = np.array(tracked_dets[:, 4], dtype='int32')
                top_id = tracked_dets[:, -1].astype("int32")
                
                if self.enable_distance:
                    top_distance = detect_result[:, 6:]

        else:
            if self.enable_distance:
                object_distance = np.zeros(
                    (len(results[0]), 3), dtype=np.float32)
                for i in range(len(results[0])):
                    object_distance[i] = self.distanceEstimate.get_distance(
                        results[0][i])

                results[0] = np.concatenate(
                    [results[0], object_distance], axis=1)

            det = results[0]

            if self.enable_distance:
                detect_result = np.hstack((det[:, [1]], det[:, [0]], det[:, [3]],
                                           det[:, [2]], det[:, [5]])).astype('int32').astype("float32")
                # ymin, xmin, ymax, xmax, class, lx, ly, lz
                detect_result = np.hstack(
                    (detect_result, det[:, [6]], det[:, [7]], det[:, [8]]))

            else:
                detect_result = np.hstack((det[:, [1]], det[:, [0]], det[:, [3]],
                                           det[:, [2]], det[:, [5]])).astype('int32')  # ymin, xmin, ymax, xmax, class

            if self.enable_vis:
                top_label = np.array(det[:, 5], dtype='int32')
                top_conf = det[:, 4]
                top_boxes = det[:, :4]

            if self.enable_distance:
                top_distance = det[:, 6:]

        if self.enable_vis:
            # ---------------------------------------------------------#
            #   图像绘制
            # ---------------------------------------------------------#
            # ---------------------------------------------------------#
            #   设置字体与边框厚度
            # ---------------------------------------------------------#
            image = Image.fromarray(image, mode='RGB')
            font = ImageFont.truetype(font=os.path.dirname(
                __file__)+'/model_data/simhei.ttf', size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
            thickness = int(
                max((image.size[0] + image.size[1]) // np.mean(self.input_shape), 1))
            for i, c in list(enumerate(top_label)):
                predicted_class = self.class_names[int(c)]
                box = top_boxes[i]

                top, left, bottom, right = box

                top = max(0, np.floor(top).astype('int32'))
                left = max(0, np.floor(left).astype('int32'))
                bottom = min(image.size[1], np.floor(bottom).astype('int32'))
                right = min(image.size[0], np.floor(right).astype('int32'))

                if self.enable_track:
                    if self.enable_distance:
                        label = '{} id={:d} x={:.2f} y={:.2f}'.format(
                            predicted_class, top_id[i], top_distance[i][0], top_distance[i][1])
                    else:
                        label = '{} id={}'.format(predicted_class, top_id[i])
                else:
                    if self.enable_distance:
                        label = '{} sc={:.1f} x={:.2f} y={:.2f}'.format(
                            predicted_class, top_conf[i], top_distance[i][0], top_distance[i][1])
                    else:
                        label = '{} sc={:.1f}'.format(
                            predicted_class, top_conf[i])

                draw = ImageDraw.Draw(image)
                label_size = draw.textsize(label, font)
                label = label.encode('utf-8')

                if top - label_size[1] >= 0:
                    text_origin = np.array([left, top - label_size[1]])
                else:
                    text_origin = np.array([left, top + 1])

                for i in range(thickness):
                    draw.rectangle([left + i, top + i, right - i,
                                   bottom - i], outline=self.colors[c])
                draw.rectangle([tuple(text_origin), tuple(
                    text_origin + label_size)], fill=self.colors[c])
                draw.text(text_origin, str(label, 'UTF-8'),
                          fill=(0, 0, 0), font=font)
                del draw

        if self.enable_vis:
            return (detect_result, np.array(image, dtype="uint8"))
        else:
            return (detect_result, 0)
