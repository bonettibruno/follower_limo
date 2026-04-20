#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
color_detector.py
Nó ROS1 (Melodic) que detecta um alvo colorido via segmentação HSV.
Publica a posição do alvo na imagem e uma imagem anotada para debug.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server

from std_msgs.msg import Float32MultiArray


# ──────────────────────────────────────────
# Parâmetros HSV — ajuste conforme sua cor
# Comece com esses valores para laranja
# e use o hsv_calibrator.py para afinar
# ──────────────────────────────────────────
HSV_LOWER = np.array([0, 120, 70])    # H_min, S_min, V_min
HSV_UPPER = np.array([15, 255, 255])  # H_max, S_max, V_max

# Laranja também aparece perto de H=170~180 (wrap do hue)
HSV_LOWER2 = np.array([170, 120, 70])
HSV_UPPER2 = np.array([180, 255, 255])

# Área mínima do contorno para considerar detecção válida (px²)
MIN_CONTOUR_AREA = 500

# Campo de visão horizontal da câmera Astra em graus
CAMERA_HFOV = 60.0  # graus — confirme no datasheet da Astra


class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=False)

        self.bridge = CvBridge()

        # Permite sobrescrever parâmetros HSV via rosparam
        self.hsv_lower = np.array(rospy.get_param('~hsv_lower', HSV_LOWER.tolist()))
        self.hsv_upper = np.array(rospy.get_param('~hsv_upper', HSV_UPPER.tolist()))
        self.hsv_lower2 = np.array(rospy.get_param('~hsv_lower2', HSV_LOWER2.tolist()))
        self.hsv_upper2 = np.array(rospy.get_param('~hsv_upper2', HSV_UPPER2.tolist()))
        self.min_area = rospy.get_param('~min_contour_area', MIN_CONTOUR_AREA)
        self.hfov = rospy.get_param('~camera_hfov', CAMERA_HFOV)

        # Publishers
        # [found(0/1), center_x_norm(-1..1), center_y_norm(-1..1), area, angle_deg]
        self.pub_detection = rospy.Publisher(
            '/target/detection', Float32MultiArray, queue_size=1)
        self.pub_image = rospy.Publisher(
            '/target/image_debug', Image, queue_size=1)

        # Subscriber
        self.sub_image = rospy.Subscriber(
            '/camera/rgb/image_raw', Image, self.image_callback, queue_size=1,
            buff_size=2**24)

        rospy.loginfo("color_detector iniciado. Aguardando imagens...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        height, width = frame.shape[:2]

        # 1. Converter para HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Criar máscara (suporta duas faixas para laranja que quebra em H=180)
        mask1 = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 3. Operações morfológicas para limpar ruído
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # 4. Encontrar contornos
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detection_msg = Float32MultiArray()
        debug_frame = frame.copy()

        # Linha central de referência
        cv2.line(debug_frame, (width // 2, 0), (width // 2, height),
                 (255, 255, 255), 1)

        if contours:
            # Pega o maior contorno
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area >= self.min_area:
                # Centro via momentos
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Normaliza centro_x para [-1, 1] (0 = centro da imagem)
                    cx_norm = (cx - width / 2.0) / (width / 2.0)
                    cy_norm = (cy - height / 2.0) / (height / 2.0)

                    # Converte posição horizontal em ângulo (positivo = direita)
                    angle_deg = cx_norm * (self.hfov / 2.0)

                    # Monta mensagem: [found, cx_norm, cy_norm, area, angle_deg]
                    detection_msg.data = [1.0, cx_norm, cy_norm, float(area), angle_deg]

                    # Desenha na imagem de debug
                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.putText(debug_frame,
                                "angle: {:.1f} deg".format(angle_deg),
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)
                    cv2.putText(debug_frame,
                                "area: {:.0f}".format(area),
                                (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)

                    rospy.loginfo_throttle(1.0,
                        "Alvo detectado | angle=%.1f deg | cx_norm=%.2f | area=%.0f",
                        angle_deg, cx_norm, area)
            else:
                detection_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            detection_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            cv2.putText(debug_frame, "SEM ALVO", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self.pub_detection.publish(detection_msg)

        # Publica imagem de debug (comprimida em resolução menor para economizar banda SSH)
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
        node = ColorDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass