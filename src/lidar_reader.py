#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
lidar_reader.py

No percurso do pipeline, este nó é o "sensor de distância" do sistema.
Ele não decide nada sozinho — apenas escuta o LiDAR e fica disponível
para responder: "qual é a distância nesse ângulo que o detector de cor
me pediu?"

Fluxo:
  /scan  -->  [lidar_reader]  -->  /lidar/distance_at_angle

O tópico de saída publica um Float32MultiArray com:
  [angulo_requisitado, distancia_medida, valido(0/1)]
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


# Ângulo de abertura do LiDAR que o robô considera "à frente"
# O LiDAR do Limo cobre 360 graus, mas só a frente nos interessa
# para seguir o alvo. Esse valor limita a faixa de busca.
VALID_ANGLE_RANGE_DEG = 60.0  # graus para cada lado do centro (frente)

# Distâncias fora desse intervalo são descartadas como inválidas
MIN_VALID_DIST = 0.15   # metros — abaixo disso é ruído ou colisão iminente
MAX_VALID_DIST = 5.0    # metros — acima disso o alvo provavelmente sumiu


class LidarReader:
    def __init__(self):
        rospy.init_node('lidar_reader', anonymous=False)

        self.valid_range = rospy.get_param('~valid_angle_range_deg', VALID_ANGLE_RANGE_DEG)
        self.min_dist = rospy.get_param('~min_valid_dist', MIN_VALID_DIST)
        self.max_dist = rospy.get_param('~max_valid_dist', MAX_VALID_DIST)

        # Guarda o último scan recebido para consulta sob demanda
        self.last_scan = None

        # Publisher — responde com a distância no ângulo pedido
        # Formato: [angulo_pedido_deg, distancia_m, valido]
        self.pub = rospy.Publisher(
            '/lidar/distance_at_angle', Float32MultiArray, queue_size=1)

        # Subscriber do detector de cor — escuta o ângulo estimado do alvo
        self.sub_detection = rospy.Subscriber(
            '/target/detection', Float32MultiArray,
            self.detection_callback, queue_size=1)

        # Subscriber do LiDAR — atualiza o scan continuamente
        self.sub_scan = rospy.Subscriber(
            '/scan', LaserScan,
            self.scan_callback, queue_size=1)

        rospy.loginfo("lidar_reader iniciado.")

    # ── Callbacks ────────────────────────────────────────────────

    def scan_callback(self, msg):
        """Apenas armazena o scan mais recente. Leve e rápido."""
        self.last_scan = msg

    def detection_callback(self, msg):
        """
        Toda vez que o detector de cor publica uma detecção,
        este callback consulta o scan atual para obter a distância
        no ângulo onde o alvo foi visto.
        """
        if self.last_scan is None:
            return

        found = msg.data[0]
        angle_deg = msg.data[4]  # ângulo vindo do color_detector

        out = Float32MultiArray()

        if found < 0.5:
            # Sem alvo visual — publica inválido
            out.data = [0.0, 0.0, 0.0]
            self.pub.publish(out)
            return

        # Consulta a distância no ângulo pedido
        distance, valid = self.get_distance_at_angle(angle_deg)

        out.data = [angle_deg, distance, 1.0 if valid else 0.0]
        self.pub.publish(out)

        if valid:
            rospy.loginfo_throttle(1.0,
                "LiDAR | angulo=%.1f deg | dist=%.2f m", angle_deg, distance)
        else:
            rospy.logwarn_throttle(2.0,
                "LiDAR | angulo=%.1f deg | leitura invalida", angle_deg)

    # ── Lógica principal ─────────────────────────────────────────

    def get_distance_at_angle(self, angle_deg):
        """
        Converte o ângulo em graus para o índice correspondente
        no array de ranges do LaserScan e retorna a distância.

        O LaserScan do ROS representa ângulos de angle_min a angle_max
        em radianos. O índice 0 corresponde a angle_min.
        """
        scan = self.last_scan

        # Verifica se o ângulo pedido está dentro da faixa frontal válida
        if abs(angle_deg) > self.valid_range:
            return 0.0, False

        angle_rad = np.deg2rad(angle_deg)

        # Índice correspondente ao ângulo pedido
        idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(idx, len(scan.ranges) - 1))

        # Pega uma pequena janela ao redor do índice e usa a mediana
        # para ser mais robusto a leituras espúrias pontuais
        window = 5
        start = max(0, idx - window)
        end = min(len(scan.ranges), idx + window + 1)
        samples = [scan.ranges[i] for i in range(start, end)
                   if self.min_dist <= scan.ranges[i] <= self.max_dist
                   and not np.isnan(scan.ranges[i])
                   and not np.isinf(scan.ranges[i])]

        if not samples:
            return 0.0, False

        distance = float(np.median(samples))
        return distance, True

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LidarReader()
        node.run()
    except rospy.ROSInterruptException:
        pass