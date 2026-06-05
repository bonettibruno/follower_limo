#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_detector.py
Nó ROS1 que detecta objetos via YOLOv8 exportado para ONNX.
Usa cv2.dnn para inferência — compatível com Python 3.6, sem torch/ultralytics.

Para gerar o modelo ONNX (rodar no PC com ultralytics instalado):
  python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx')"
  scp yolov8n.onnx agilex@IP_DO_LIMO:/home/agilex/
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

# 80 classes do dataset COCO (YOLOv8 padrão)
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

        model_path        = rospy.get_param('~model_path',           '/home/agilex/yolov8n.onnx')
        self.target_class = rospy.get_param('~target_class',         'sports ball')
        self.conf_thresh  = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_thresh   = rospy.get_param('~nms_threshold',        0.45)
        self.hfov         = rospy.get_param('~camera_hfov',          60.0)
        self.input_size   = 640

        rospy.loginfo("yolo_detector: carregando modelo ONNX: %s", model_path)
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        rospy.loginfo("yolo_detector: modelo carregado. Classe alvo: '%s'", self.target_class)

        # [found(0/1), cx_norm(-1..1), cy_norm(-1..1), area_px2, angle_deg]
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

        # Pré-processamento: redimensiona para 640x640, normaliza e converte para blob
        blob = cv2.dnn.blobFromImage(
            frame, 1.0 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False)
        self.net.setInput(blob)
        raw_output = self.net.forward()  # (1, 84, 8400)

        # Pós-processamento
        output = raw_output[0].T  # (8400, 84)
        boxes_raw   = output[:, :4]        # cx, cy, w, h em escala 640x640
        class_scores = output[:, 4:]       # 80 scores de classe

        class_ids    = np.argmax(class_scores, axis=1)
        confidences  = class_scores[np.arange(len(class_ids)), class_ids]

        # Filtra pela classe alvo e confiança mínima
        target_id = COCO_CLASSES.index(self.target_class) if self.target_class in COCO_CLASSES else -1
        mask = (confidences >= self.conf_thresh) & (class_ids == target_id)

        boxes_raw   = boxes_raw[mask]
        confidences = confidences[mask]

        detection_msg = Float32MultiArray()
        debug_frame   = frame.copy()
        detected_class = ""

        if len(boxes_raw) > 0:
            # Converte cx,cy,w,h → x,y,w,h para NMS
            x_scale = width  / self.input_size
            y_scale = height / self.input_size
            nms_boxes = []
            for cx, cy, w, h in boxes_raw:
                x1 = int((cx - w / 2) * x_scale)
                y1 = int((cy - h / 2) * y_scale)
                nms_boxes.append([x1, y1, int(w * x_scale), int(h * y_scale)])

            indices = cv2.dnn.NMSBoxes(
                nms_boxes, confidences.tolist(), self.conf_thresh, self.nms_thresh)

            if len(indices) > 0:
                # Pega a detecção com maior confiança após NMS
                idx = indices[0][0] if isinstance(indices[0], (list, np.ndarray)) else indices[0]
                x, y, w, h = nms_boxes[idx]
                conf = confidences[idx]

                cx_px = x + w // 2
                cy_px = y + h // 2
                area  = float(w * h)

                cx_norm   = (cx_px - width  / 2.0) / (width  / 2.0)
                cy_norm   = (cy_px - height / 2.0) / (height / 2.0)
                angle_deg = cx_norm * (self.hfov / 2.0)

                detection_msg.data = [1.0, cx_norm, cy_norm, area, angle_deg]
                detected_class = self.target_class

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
