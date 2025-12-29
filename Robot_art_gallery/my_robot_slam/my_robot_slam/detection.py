#!/usr/bin/env python

from glob import glob

import cv2
import numpy as np

from openvino.inference_engine import IECore

from my_robot_slam.yolox_utils import multiclass_nms, vis, preprocess, demo_postprocess, COCO_CLASSES

from ament_index_python import get_package_share_directory

import my_robot_slam.apriltag as apriltag
from copy import deepcopy


class YOLOX():
    def __init__(self):
        options = apriltag.DetectorOptions(families="tag16h5")
        self.detector = apriltag.Detector(options)

        self.model_path = get_package_share_directory('my_robot_slam') + "/weights/yolox_m.onnx"
        self.detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

        self.input_shape = (640, 640)
        self.with_p6 = False
        self.nms_th = 0.5
        self.conf = 0.3

        # ---------------------------Step 1. Initialize inference engine core--------------------------------------------------
        ie = IECore()

        # ---------------------------Step 2. Read a model in OpenVINO Intermediate Representation or ONNX format---------------
        # (.xml and .bin files) or (.onnx file)
        self.net = ie.read_network(model=self.model_path)

        if len(self.net.input_info) != 1:
            return -1
        if len(self.net.outputs) != 1:
            return -1

        # ---------------------------Step 3. Configure input & output----------------------------------------------------------
        # Get names of input and output blobs
        self.input_blob = next(iter(self.net.input_info))
        self.out_blob = next(iter(self.net.outputs))

        # Set input and output precision manually
        self.net.input_info[self.input_blob].precision = 'FP32'
        self.net.outputs[self.out_blob].precision = 'FP16'

        # Get a number of classes recognized by a model
        num_of_classes = max(self.net.outputs[self.out_blob].shape)

        # ---------------------------Step 4. Loading model to the device-------------------------------------------------------
        self.exec_net = ie.load_network(network=self.net, device_name='CPU')

    def image_detection(self, img, image_dict, result_img):
        new_image_dict = image_dict
        result, _ = apriltag.detect_tags(img,
                                            self.detector,
                                            camera_params=(1696.802685832259, 1696.802685832259, 960.5, 540.5),
                                            tag_size=0.0762,
                                            vizualization=0,
                                            verbose=0,
                                            annotation=False
                                            )

        if len(result) > 0:
            # preprocess
            origin_img, self.ratio = preprocess(img, self.input_shape)

            # ---------------------------Step 5. Create infer request--------------------------------------------------------------
            # load_network() method of the IECore class with a specified number of requests (default 1) returns an ExecutableNetwork
            # instance which stores infer requests. So you already created Infer requests in the previous step.

            # ---------------------------Step 6. Prepare input---------------------------------------------------------------------
            # _, _, h, w = self.net.input_info[self.input_blob].input_data.shape

            # ---------------------------Step 7. Do inference----------------------------------------------------------------------
            res = self.exec_net.infer(inputs={self.input_blob: origin_img})

            # ---------------------------Step 8. Process output--------------------------------------------------------------------
            res = res[self.out_blob]
            
            predictions = demo_postprocess(res, self.input_shape, p6=self.with_p6)[0]

            boxes = predictions[:, :4]
            scores = predictions[:, 4:5] * predictions[:, 5:]

            boxes_xyxy = np.ones_like(boxes)
            boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2]/2.
            boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3]/2.
            boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2]/2.
            boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3]/2.
            boxes_xyxy /= self.ratio
            dets = multiclass_nms(boxes_xyxy, scores, nms_thr=self.nms_th, score_thr=self.conf)
            if dets is not None:
                final_boxes, final_scores, self.final_cls_inds = dets[:, :4], dets[:, 4], dets[:, 5]
                final_cls_inds = []
                [final_cls_inds.append(i) for i in self.final_cls_inds if i not in final_cls_inds]
                if len(final_cls_inds) > 1:
                    vertical_split = min(int(final_boxes[0][2]), int(final_boxes[1][2]))
                    split_img_1 = img[:, :vertical_split, :]
                    split_img_2 = img[:, vertical_split:, :]
                    new_image_dict, result_img = self.image_detection(split_img_1, new_image_dict, result_img)
                    new_image_dict, result_img = self.image_detection(split_img_2, new_image_dict, result_img)
                else:
                    filtered_painting = deepcopy(img)
                    
                    max_x = 0
                    for i in range(len(self.final_cls_inds)):
                        if max_x < int(final_boxes[i][2]):
                            max_x = int(final_boxes[i][2])
                    filtered_painting = filtered_painting[:, :max_x, :]

                    for i in range(len(final_boxes)):
                        filtered_painting[int(final_boxes[i][1]):int(final_boxes[i][3]), int(final_boxes[i][0]):int(final_boxes[i][2]), :] = 0

                    result, annotated_img = apriltag.detect_tags(filtered_painting,
                                                        self.detector,
                                                        camera_params=(1696.802685832259, 1696.802685832259, 960.5, 540.5),
                                                        tag_size=0.0762,
                                                        vizualization=0,
                                                        verbose=0,
                                                        annotation=True
                                                        )
                    tag_id = list(result[0])[1]

                    new_image_dict[tag_id] = str(COCO_CLASSES[int(final_cls_inds[0])])

                    for i in range(len(final_boxes)):
                        annotated_img[int(final_boxes[i][1]):int(final_boxes[i][3]), int(final_boxes[i][0]):int(final_boxes[i][2]), :] = \
                                               img[int(final_boxes[i][1]):int(final_boxes[i][3]), int(final_boxes[i][0]):int(final_boxes[i][2]), :]

                    result_img = cv2.hconcat([result_img, vis(annotated_img, final_boxes, final_scores, self.final_cls_inds,
                                    conf=0.3, class_names=COCO_CLASSES)])

        return new_image_dict, result_img


def main():
    yolox = YOLOX()
    image_dict = {}
    image_list = [f for f in sorted(glob("/home/vh5465s/ros_ws/install/my_robot_slam/share/my_robot_slam/pics/*.jpg"))]
    
    result_img = np.zeros([1080,1,3],dtype=np.uint8)
    result_img[:] = 255
    for i, image in enumerate(image_list):
        print('Scanning', i + 1, '/', len(image_list))
        img = cv2.imread(image)
        image_dict, result_img = yolox.image_detection(img, image_dict, result_img)

    scale_percent = 1080 / result_img.shape[1]
    width = int(result_img.shape[1] * scale_percent)
    height = int(result_img.shape[0] * scale_percent)
    dim = (width, height)

    resized = cv2.resize(result_img, dim, interpolation = cv2.INTER_AREA)
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,25)
    fontScale              = 1
    fontColor              = (255,255,255)
    thickness              = 2
    lineType               = 2

    resized = cv2.putText(resized,str(image_dict), 
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)

    cv2.imshow('paintings', resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
