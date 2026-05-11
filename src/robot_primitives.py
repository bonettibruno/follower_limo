#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
robot_primitives.py
Biblioteca de ações primitivas do robô. Importada pelo task_executor — não é um nó ROS.

Cada função é bloqueante: só retorna quando a ação termina ou o timeout estoura.
Os publishers e o objeto RobotState são criados e mantidos pelo task_executor.

Freio de segurança:
  go_forward e approach_object verificam lidar_min_front a cada iteração.
  Se um obstáculo estiver a menos de SAFETY_DIST metros na frente, param imediatamente.

Campos potenciais em approach_object:
  A velocidade linear resulta da soma de uma força atrativa (em direção ao alvo)
  e uma força repulsiva (para longe de obstáculos próximos detectados pelo LiDAR).
  Força atrativa:  fa = kp_linear * (distancia_atual - distancia_alvo)
  Força repulsiva: fr = kr * (1/d - 1/d0)  quando d < d0, senão 0
  Velocidade resultante: v = fa - fr
"""

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Distância mínima frontal para acionar o freio de segurança (metros)
SAFETY_DIST = 0.20

# Distância a partir da qual a repulsão começa a agir no approach_object (metros)
REPULSIVE_INFLUENCE_DIST = 0.50

# Ganho da força repulsiva — aumente se o robô ainda bate, reduza se para cedo demais
KR = 0.15


class RobotState:
    """
    Armazena o estado mais recente dos sensores.
    Atualizado pelo task_executor via callbacks de subscriber.
    """
    def __init__(self):
        self.yolo_found     = False
        self.yolo_cx_norm   = 0.0   # posição horizontal normalizada [-1, 1]
        self.yolo_cy_norm   = 0.0
        self.yolo_area      = 0.0   # área do bounding box em px²
        self.yolo_angle_deg = 0.0
        self.yolo_class     = ""

        self.lidar_distance = 0.0   # distância ao alvo detectado (metros)
        self.lidar_valid    = False

        # Distância mínima no setor frontal (±30°) do LiDAR bruto
        self.lidar_min_front       = float('inf')
        self.lidar_min_front_valid = False


def _safety_brake_triggered(state):
    """Retorna True se deve parar por segurança."""
    return (state is not None
            and state.lidar_min_front_valid
            and state.lidar_min_front < SAFETY_DIST)


# ── Primitivas de movimento ──────────────────────────────────────────────────

def stop(pub_cmd):
    """Para o robô imediatamente."""
    pub_cmd.publish(Twist())
    rospy.sleep(0.1)


def go_forward(pub_cmd, distance_m, speed=0.2, state=None):
    """
    Avança 'distance_m' metros a 'speed' m/s.
    Distância é estimada por tempo (sem odometria).
    Para imediatamente se o freio de segurança disparar.
    Valores negativos fazem o robô recuar.
    """
    if distance_m == 0:
        return

    duration = abs(distance_m) / abs(speed)
    cmd = Twist()
    cmd.linear.x = speed if distance_m > 0 else -speed

    end_time = rospy.Time.now() + rospy.Duration(duration)
    rate = rospy.Rate(20)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        if _safety_brake_triggered(state):
            stop(pub_cmd)
            rospy.logwarn("go_forward: FREIO DE SEGURANCA — obstáculo a %.2fm",
                          state.lidar_min_front)
            return
        pub_cmd.publish(cmd)
        rate.sleep()

    stop(pub_cmd)
    rospy.loginfo("go_forward: %.2f m concluido", distance_m)


def rotate(pub_cmd, degrees, angular_speed=0.5, state=None):
    """
    Gira 'degrees' graus no lugar.
    Positivo = esquerda (anti-horário), negativo = direita (horário).
    Ângulo é estimado por tempo com a velocidade angular fornecida.
    """
    if degrees == 0:
        return

    angle_rad = math.radians(abs(degrees))
    duration  = angle_rad / abs(angular_speed)
    cmd = Twist()
    cmd.angular.z = angular_speed if degrees > 0 else -angular_speed

    end_time = rospy.Time.now() + rospy.Duration(duration)
    rate = rospy.Rate(20)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        pub_cmd.publish(cmd)
        rate.sleep()

    stop(pub_cmd)
    rospy.loginfo("rotate: %.1f graus concluido", degrees)


def scan_area(pub_cmd, angular_speed=0.3, state=None):
    """Rotação de 360° para varrer a área ao redor do robô."""
    rospy.loginfo("scan_area: iniciando varredura 360 graus")
    rotate(pub_cmd, 360, angular_speed, state)
    rospy.loginfo("scan_area: concluido")


# ── Primitivas com percepção ─────────────────────────────────────────────────

def search_object(pub_cmd, class_name, state, angular_speed=0.35, timeout=15.0):
    """
    Gira devagar até encontrar 'class_name' via YOLO.
    Retorna True se encontrou, False se timeout.
    """
    rospy.loginfo("search_object: procurando '%s' (timeout=%.0fs)", class_name, timeout)
    cmd = Twist()
    cmd.angular.z = angular_speed

    end_time = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        if state.yolo_found and state.yolo_class == class_name:
            stop(pub_cmd)
            rospy.loginfo("search_object: '%s' encontrado!", class_name)
            return True
        pub_cmd.publish(cmd)
        rate.sleep()

    stop(pub_cmd)
    rospy.logwarn("search_object: timeout — '%s' nao encontrado", class_name)
    return False


def approach_object(pub_cmd, class_name, state,
                    target_dist=0.5, kp_angular=0.6, kp_linear=0.4,
                    max_linear=0.3, max_angular=0.6,
                    deadzone_angle=0.05, deadzone_dist=0.05,
                    timeout=20.0):
    """
    Aproxima do objeto usando P-controller com campos potenciais.

    Velocidade linear = força_atrativa - força_repulsiva
      Atrativa:  kp_linear * (dist_atual - dist_alvo)
      Repulsiva: KR * (1/d_obst - 1/REPULSIVE_INFLUENCE_DIST)  se d_obst < REPULSIVE_INFLUENCE_DIST

    Para imediatamente se o freio de segurança disparar (SAFETY_DIST).
    Retorna True se chegou na distância alvo, False se timeout ou freio.
    """
    rospy.loginfo("approach_object: aproximando de '%s' (alvo=%.1fm)", class_name, target_dist)

    end_time = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)

    while rospy.Time.now() < end_time and not rospy.is_shutdown():

        # Freio de segurança — tem prioridade sobre tudo
        if _safety_brake_triggered(state):
            stop(pub_cmd)
            rospy.logwarn("approach_object: FREIO DE SEGURANCA — obstáculo a %.2fm",
                          state.lidar_min_front)
            return False

        if not (state.yolo_found and state.yolo_class == class_name):
            rospy.logwarn_throttle(2.0, "approach_object: '%s' fora de visao", class_name)
            pub_cmd.publish(Twist())
            rate.sleep()
            continue

        cmd = Twist()
        error_angle = state.yolo_cx_norm  # [-1, 1], positivo = alvo à direita

        # Controle angular: centraliza o objeto no campo de visão
        if abs(error_angle) > deadzone_angle:
            cmd.angular.z = -kp_angular * error_angle
            cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))

        # Força atrativa: puxa em direção à distância alvo
        if state.lidar_valid:
            error_dist = state.lidar_distance - target_dist
        else:
            error_dist = 0.3 if state.yolo_area < 20000 else 0.0

        f_attractive = kp_linear * error_dist

        # Força repulsiva: empurra para longe de obstáculos próximos
        f_repulsive = 0.0
        if (state.lidar_min_front_valid
                and state.lidar_min_front < REPULSIVE_INFLUENCE_DIST
                and state.lidar_min_front > 0.01):
            f_repulsive = KR * (1.0 / state.lidar_min_front
                                - 1.0 / REPULSIVE_INFLUENCE_DIST)

        # Velocidade resultante: só aplica quando razoavelmente alinhado
        net_linear = f_attractive - f_repulsive
        if abs(error_angle) < 0.3 and abs(error_dist) > deadzone_dist:
            cmd.linear.x = max(-max_linear, min(max_linear, net_linear))

        rospy.loginfo_throttle(1.0,
            "approach | dist=%.2fm err=%.2f fa=%.2f fr=%.2f v=%.2f",
            state.lidar_distance if state.lidar_valid else -1,
            error_dist, f_attractive, f_repulsive, cmd.linear.x)

        # Critério de parada: chegou na distância alvo (só com lidar)
        if state.lidar_valid and abs(state.lidar_distance - target_dist) < deadzone_dist:
            stop(pub_cmd)
            rospy.loginfo("approach_object: distancia alvo atingida (%.2fm)",
                          state.lidar_distance)
            return True

        pub_cmd.publish(cmd)
        rate.sleep()

    stop(pub_cmd)
    rospy.logwarn("approach_object: timeout")
    return False


# ── Alerta ───────────────────────────────────────────────────────────────────

def alert(pub_alert, message):
    """Loga alerta no terminal e publica em /task/alert."""
    rospy.logwarn("*** ALERTA: %s ***", message)
    pub_alert.publish(String(data=message))
