#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_detector.py
Nó ROS1 que detecta objetos via YOLOv8 (ultralytics).
Publica no mesmo formato do color_detector para compatibilidade com lidar_reader.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError("ultralytics nao instalado. Execute: pip install ultralytics")


class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=False)

        self.bridge = CvBridge()

        model_path        = rospy.get_param('~model_path',           'yolov8n.pt')
        self.target_class = rospy.get_param('~target_class',         'sports ball')
        self.conf_thresh  = rospy.get_param('~confidence_threshold', 0.5)
        self.hfov         = rospy.get_param('~camera_hfov',          60.0)

        rospy.loginfo("Carregando modelo YOLO: %s", model_path)
        self.model = YOLO(model_path)
        rospy.loginfo("Modelo carregado. Classe alvo: '%s'", self.target_class)

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

        rospy.loginfo("yolo_detector iniciado.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        height, width = frame.shape[:2]
        results = self.model(frame, conf=self.conf_thresh, verbose=False)

        detection_msg = Float32MultiArray()
        debug_frame   = frame.copy()
        best_box      = None
        best_conf     = 0.0
        detected_class = ""

        for result in results:
            for box in result.boxes:
                class_name = self.model.names[int(box.cls)]
                conf       = float(box.conf)
                if class_name == self.target_class and conf > best_conf:
                    best_conf      = conf
                    best_box       = box
                    detected_class = class_name

        if best_box is not None:
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            cx   = (x1 + x2) // 2
            cy   = (y1 + y2) // 2
            area = float((x2 - x1) * (y2 - y1))

            cx_norm   = (cx - width  / 2.0) / (width  / 2.0)
            cy_norm   = (cy - height / 2.0) / (height / 2.0)
            angle_deg = cx_norm * (self.hfov / 2.0)

            detection_msg.data = [1.0, cx_norm, cy_norm, area, angle_deg]

            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug_frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(debug_frame,
                        "{} {:.0f}% | {:.1f}deg".format(
                            detected_class, best_conf * 100, angle_deg),
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            rospy.loginfo_throttle(1.0,
                "YOLO | '%s' conf=%.2f | angle=%.1f deg | area=%.0f",
                detected_class, best_conf, angle_deg, area)
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
