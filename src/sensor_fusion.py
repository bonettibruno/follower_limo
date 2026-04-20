#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
sensor_fusion.py

Este nó é o "cérebro geométrico" do sistema. Ele não fala com sensores
diretamente — apenas combina o que o color_detector e o lidar_reader
já processaram, e produz uma estimativa limpa da posição relativa do alvo.

Fluxo:
  /target/detection  ──┐
                        ├──> [sensor_fusion] ──> /target/pose
  /lidar/distance_at_angle ─┘

Saída /target/pose — Float32MultiArray:
  [encontrado, angulo_deg, distancia_m, erro_angular_norm, erro_distancia_m]

  - angulo_deg        : para onde o robô deve virar (positivo = direita)
  - distancia_m       : distância atual até o alvo
  - erro_angular_norm : erro de direção normalizado [-1, 1], alimenta rotação
  - erro_distancia_m  : distancia_atual - distancia_desejada, alimenta avanço
"""

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray


# Distância que o robô deve manter do alvo (metros)
DEFAULT_TARGET_DISTANCE = 1.0

# Se o alvo sumir da câmera por mais que isso, o robô para
TARGET_TIMEOUT_SEC = 1.0

# Filtro de suavização exponencial — evita movimentos bruscos
# 0.0 = ignora novo valor, 1.0 = sem filtro
ALPHA_ANGLE = 0.4
ALPHA_DIST  = 0.6


class SensorFusion:
    def __init__(self):
        rospy.init_node('sensor_fusion', anonymous=False)

        self.target_distance = rospy.get_param('~target_distance', DEFAULT_TARGET_DISTANCE)
        self.timeout = rospy.get_param('~target_timeout', TARGET_TIMEOUT_SEC)
        self.alpha_angle = rospy.get_param('~alpha_angle', ALPHA_ANGLE)
        self.alpha_dist = rospy.get_param('~alpha_dist', ALPHA_DIST)

        # Estado interno — última estimativa válida (para suavização)
        self.last_angle = 0.0
        self.last_dist = self.target_distance
        self.last_detection_time = None

        # Publisher
        self.pub_pose = rospy.Publisher(
            '/target/pose', Float32MultiArray, queue_size=1)

        # Subscribers
        self.sub_detection = rospy.Subscriber(
            '/target/detection', Float32MultiArray,
            self.detection_callback, queue_size=1)

        self.sub_lidar = rospy.Subscriber(
            '/lidar/distance_at_angle', Float32MultiArray,
            self.lidar_callback, queue_size=1)

        # Guarda última leitura de cada sensor separadamente
        self.current_detection = None   # [found, cx_norm, cy_norm, area, angle_deg]
        self.current_lidar = None       # [angle_deg, distance_m, valid]

        # Timer que publica a fusão a 20 Hz independente dos callbacks
        rospy.Timer(rospy.Duration(0.05), self.fusion_callback)

        rospy.loginfo("sensor_fusion iniciado. Distancia alvo: %.1f m", self.target_distance)

    # ── Callbacks de entrada ──────────────────────────────────────

    def detection_callback(self, msg):
        self.current_detection = msg.data

    def lidar_callback(self, msg):
        self.current_lidar = msg.data

    # ── Fusão principal (rodando a 20 Hz) ────────────────────────

    def fusion_callback(self, event):
        out = Float32MultiArray()

        if self.current_detection is None:
            out.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_pose.publish(out)
            return

        cam_found  = self.current_detection[0]
        angle_cam  = self.current_detection[4]

        if cam_found < 0.5:
            if self.last_detection_time is not None:
                elapsed = (rospy.Time.now() - self.last_detection_time).to_sec()
                if elapsed > self.timeout:
                    rospy.logwarn_throttle(2.0, "Alvo perdido ha %.1f s — robô parado.", elapsed)
            out.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_pose.publish(out)
            return

        # Alvo visível — atualiza timestamp
        self.last_detection_time = rospy.Time.now()

        # ── Distância: prefere LiDAR, cai de volta na câmera ─────
        distance = self._estimate_distance(angle_cam)

        # ── Suavização exponencial ────────────────────────────────
        # Evita que ruídos pontuais causem movimentos bruscos
        smooth_angle = self.alpha_angle * angle_cam + (1 - self.alpha_angle) * self.last_angle
        smooth_dist  = self.alpha_dist  * distance  + (1 - self.alpha_dist)  * self.last_dist

        self.last_angle = smooth_angle
        self.last_dist  = smooth_dist

        # ── Erros para o controlador ──────────────────────────────
        # erro_angular_norm em [-1, 1]: o controlador multiplica por Kp_angular
        erro_angular_norm = smooth_angle / 30.0   # normaliza pelo semi-FOV
        erro_angular_norm = max(-1.0, min(1.0, erro_angular_norm))

        # erro_distancia em metros: positivo = muito longe, negativo = muito perto
        erro_distancia = smooth_dist - self.target_distance

        out.data = [
            1.0,                  # encontrado
            smooth_angle,         # ângulo suavizado (graus)
            smooth_dist,          # distância suavizada (m)
            erro_angular_norm,    # erro direcional → rotação
            erro_distancia        # erro de distância → avanço/recuo
        ]
        self.pub_pose.publish(out)

        rospy.loginfo_throttle(1.0,
            "Fusão | ang=%.1f° | dist=%.2f m | err_ang=%.2f | err_dist=%.2f m",
            smooth_angle, smooth_dist, erro_angular_norm, erro_distancia)

    # ── Estimativa de distância ───────────────────────────────────

    def _estimate_distance(self, angle_cam):
        """
        Estratégia em camadas:
        1. Usa LiDAR se disponível e válido — é o mais preciso
        2. Fallback: usa a última distância conhecida (mantém o robô estável
           por um curto período se o LiDAR falhar pontualmente)
        """
        if self.current_lidar is not None:
            lidar_angle = self.current_lidar[0]
            lidar_dist  = self.current_lidar[1]
            lidar_valid = self.current_lidar[2]

            # Aceita leitura LiDAR só se o ângulo ainda é coerente com a câmera
            angle_diff = abs(lidar_angle - angle_cam)
            if lidar_valid > 0.5 and angle_diff < 10.0:
                return lidar_dist

        # Fallback — mantém última distância conhecida
        rospy.logwarn_throttle(3.0, "LiDAR indisponivel — usando ultima distancia conhecida.")
        return self.last_dist

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SensorFusion()
        node.run()
    except rospy.ROSInterruptException:
        pass