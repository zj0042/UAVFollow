import numpy as np
import time
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective
from models.common import DetectMultiBackend
import torch


class YoloTracker:

    def __init__(self, config):
        # construct the argument parse and parse the arguments
        MODEL_DIR = config['MODEL_DIR']
        self.device = select_device('')
        model = DetectMultiBackend(MODEL_DIR, device=self.device, dnn=False, data=None, fp16=False)
        self.stride, self.names, pt = model.stride, model.names, model.pt
        self.imgsz = check_img_size((960, 720), s=self.stride)

        model.warmup(imgsz=(1, 3, *self.imgsz))
        self.model = model

        print("model loaded... " + MODEL_DIR)
        self.rect = ()

    def get_rect(self, frame):
        t1 = time.time()
        img = letterbox(frame, self.imgsz, stride=self.stride)[0]
        img = img.transpose((2, 0, 1))[::-1]
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255
        if len(img.shape) == 3:
            img = img[None]

        pred = self.model(img)
        pred = non_max_suppression(pred, conf_thres=0.2, iou_thres=0.45, classes=None, max_det=1)
        det = pred[0]
        im0 = frame.copy()
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        annotator = Annotator(im0, line_width=3, example=str(self.names))

        rect = None
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            *xyxy, conf, cls = det[0]
            c = int(cls)  # integer class
            label = f'{self.names[c]} {conf:.2f}'
            annotator.box_label(xyxy, label, color=colors(c, True))
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()
            rect = xywh[0], xywh[1], np.sqrt(xywh[2] ** 2 + xywh[3] ** 2) / 2

        frame = annotator.result()

        t2 = time.time()
        print(f'{1 / (t2 - t1)} fps')

        return frame, rect
