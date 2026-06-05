#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_detector.py
Nó ROS1 que detecta objetos via YOLOv3-tiny usando cv2.dnn (formato darknet).
Compatível com Python 3.6 e OpenCV 4.1.1 — sem torch, sem ultralytics.

Para baixar o modelo no Limo:
  wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg
  wget https://pjreddie.com/media/files/yolov3-tiny.weights
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
]


class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=False)

        self.bridge = CvBridge()

        cfg_path          = rospy.get_param('~cfg_path',             '/home/agilex/yolov3-tiny.cfg')
        weights_path      = rospy.get_param('~weights_path',         '/home/agilex/yolov3-tiny.weights')
        self.target_class = rospy.get_param('~target_class',         'sports ball')
        self.conf_thresh  = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_thresh   = rospy.get_param('~nms_threshold',        0.45)
        self.hfov         = rospy.get_param('~camera_hfov',          60.0)
        self.input_size   = 416  # YOLOv3-tiny usa 416x416

        rospy.loginfo("yolo_detector: carregando modelo darknet...")
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Camadas de saída do YOLOv3
        layer_names = self.net.getLayerNames()
        self.output_layers = [
            layer_names[i[0] - 1]
            for i in self.net.getUnconnectedOutLayers()
        ]

        rospy.loginfo("yolo_detector: modelo carregado. Classe alvo: '%s'", self.target_class)

        self.pub_detection = rospy.Publisher(
            '/yolo/detection', Float32MultiArray, queue_size=1)
        self.pub_class = rospy.Publisher(
            '/yolo/class', String, queue_size=1)
        self.pub_image = rospy.Publisher(
            '/yolo/image_debug', Image, queue_size=1)

        self.sub_image = rospy.Subscriber(
            '/camera/rgb/image_raw', Image, self.image_callback,
            queue_size=1, buff_size=2**24)

        rospy.loginfo("yolo_detector: iniciado.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        height, width = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame, 1.0 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        boxes       = []
        confidences = []

        target_id = COCO_CLASSES.index(self.target_class) \
            if self.target_class in COCO_CLASSES else -1

        for out in outs:
            for detection in out:
                scores     = detection[5:]
                class_id   = int(np.argmax(scores))
                confidence = float(scores[class_id])

                if class_id != target_id or confidence < self.conf_thresh:
                    continue

                cx = int(detection[0] * width)
                cy = int(detection[1] * height)
                w  = int(detection[2] * width)
                h  = int(detection[3] * height)
                x  = cx - w // 2
                y  = cy - h // 2

                boxes.append([x, y, w, h])
                confidences.append(confidence)

        detection_msg  = Float32MultiArray()
        debug_frame    = frame.copy()
        detected_class = ""

        if boxes:
            indices = cv2.dnn.NMSBoxes(
                boxes, confidences, self.conf_thresh, self.nms_thresh)

            if len(indices) > 0:
                idx  = indices[0][0]
                x, y, w, h = boxes[idx]
                conf = confidences[idx]

                cx_px = x + w // 2
                cy_px = y + h // 2
                area  = float(w * h)

                cx_norm   = (cx_px - width  / 2.0) / (width  / 2.0)
                cy_norm   = (cy_px - height / 2.0) / (height / 2.0)
                angle_deg = cx_norm * (self.hfov / 2.0)

                detection_msg.data = [1.0, cx_norm, cy_norm, area, angle_deg]
                detected_class     = self.target_class

                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(debug_frame, (cx_px, cy_px), 6, (0, 0, 255), -1)
                cv2.putText(debug_frame,
                            "{} {:.0f}% | {:.1f}deg".format(
                                self.target_class, conf * 100, angle_deg),
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                rospy.loginfo_throttle(1.0,
                    "YOLO | '%s' conf=%.2f | angle=%.1f deg | area=%.0f",
                    self.target_class, conf, angle_deg, area)
        else:
            detection_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            cv2.putText(debug_frame, "SEM ALVO YOLO", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self.pub_detection.publish(detection_msg)
        self.pub_class.publish(String(data=detected_class))

        debug_small = cv2.resize(debug_frame, (320, 240))
        try:
            self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(debug_small, encoding='bgr8'))
        except Exception as e:
            rospy.logerr("Erro ao publicar imagem debug: %s", e)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YoloDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
