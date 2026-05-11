#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
robot_primitives.py
Biblioteca de ações primitivas do robô. Importada pelo task_executor — não é um nó ROS.

Cada função é bloqueante: só retorna quando a ação termina ou o timeout estoura.
Os publishers e o objeto RobotState são criados e mantidos pelo task_executor.

Odometria:
  go_forward e rotate usam /odom para medir distância e ângulo reais quando disponível.
  Se o Limo não publicar /odom, cai automaticamente para estimativa por tempo.

Freio de segurança:
  go_forward e approach_object verificam lidar_min_front a cada iteração.
  Se um obstáculo estiver a menos de SAFETY_DIST metros na frente, param imediatamente.

Campos potenciais em approach_object:
  Velocidade linear = força atrativa − força repulsiva
  Atrativa:  kp_linear * (distancia_atual - distancia_alvo)
  Repulsiva: KR * (1/d − 1/d0)  quando d < d0, senão 0
"""

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

SAFETY_DIST              = 0.20   # metros — freio de segurança
REPULSIVE_INFLUENCE_DIST = 0.50   # metros — distância onde repulsão começa
KR                       = 0.15   # ganho da força repulsiva


class RobotState:
    """
    Armazena o estado mais recente dos sensores.
    Atualizado pelo task_executor via callbacks de subscriber.
    """
    def __init__(self):
        # Percepção visual (YOLO)
        self.yolo_found     = False
        self.yolo_cx_norm   = 0.0
        self.yolo_cy_norm   = 0.0
        self.yolo_area      = 0.0
        self.yolo_angle_deg = 0.0
        self.yolo_class     = ""

        # Distância ao alvo (lidar_reader)
        self.lidar_distance = 0.0
        self.lidar_valid    = False

        # LiDAR bruto — mínimo no setor frontal ±30°
        self.lidar_min_front       = float('inf')
        self.lidar_min_front_valid = False

        # Odometria
        self.odom_x     = 0.0
        self.odom_y     = 0.0
        self.odom_yaw   = 0.0   # radianos
        self.odom_valid = False


# ── Utilitários ──────────────────────────────────────────────────────────────

def _safety_brake_triggered(state):
    return (state is not None
            and state.lidar_min_front_valid
            and state.lidar_min_front < SAFETY_DIST)


def _angle_diff(a, b):
    """Diferença angular com/sem wraparound: retorna a − b em [−π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


# ── Primitivas de movimento ──────────────────────────────────────────────────

def stop(pub_cmd):
    """Para o robô imediatamente."""
    pub_cmd.publish(Twist())
    rospy.sleep(0.1)


def go_forward(pub_cmd, distance_m, speed=0.2, state=None):
    """
    Avança distance_m metros a speed m/s.
    Usa odometria se disponível; senão estima por tempo.
    Para imediatamente se o freio de segurança disparar.
    """
    if distance_m == 0:
        return

    cmd = Twist()
    cmd.linear.x = speed if distance_m > 0 else -speed
    rate = rospy.Rate(20)

    if state is not None and state.odom_valid:
        start_x, start_y = state.odom_x, state.odom_y
        target = abs(distance_m)
        timeout = rospy.Time.now() + rospy.Duration(target / abs(speed) * 3)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if _safety_brake_triggered(state):
                stop(pub_cmd)
                rospy.logwarn("go_forward: FREIO DE SEGURANCA — obstáculo a %.2fm",
                              state.lidar_min_front)
                return
            traveled = math.sqrt((state.odom_x - start_x)**2
                                 + (state.odom_y - start_y)**2)
            if traveled >= target:
                break
            pub_cmd.publish(cmd)
            rate.sleep()
        rospy.loginfo("go_forward: %.2fm por odometria", distance_m)
    else:
        end_time = rospy.Time.now() + rospy.Duration(abs(distance_m) / abs(speed))
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            if _safety_brake_triggered(state):
                stop(pub_cmd)
                rospy.logwarn("go_forward: FREIO DE SEGURANCA — obstáculo a %.2fm",
                              state.lidar_min_front)
                return
            pub_cmd.publish(cmd)
            rate.sleep()
        rospy.loginfo("go_forward: %.2fm por tempo estimado", distance_m)

    stop(pub_cmd)


def rotate(pub_cmd, degrees, angular_speed=0.5, state=None):
    """
    Gira degrees graus no lugar. Positivo = esquerda, negativo = direita.
    Usa odometria se disponível (acumula ângulo para suportar >360°); senão por tempo.
    """
    if degrees == 0:
        return

    cmd = Twist()
    cmd.angular.z = angular_speed if degrees > 0 else -angular_speed
    rate = rospy.Rate(20)

    if state is not None and state.odom_valid:
        target_rad  = math.radians(abs(degrees))
        prev_yaw    = state.odom_yaw
        accumulated = 0.0
        timeout = rospy.Time.now() + rospy.Duration(target_rad / abs(angular_speed) * 3)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            diff = abs(_angle_diff(state.odom_yaw, prev_yaw))
            accumulated += diff
            prev_yaw = state.odom_yaw
            if accumulated >= target_rad:
                break
            pub_cmd.publish(cmd)
            rate.sleep()
        rospy.loginfo("rotate: %.1f graus por odometria", degrees)
    else:
        end_time = rospy.Time.now() + rospy.Duration(
            math.radians(abs(degrees)) / abs(angular_speed))
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            pub_cmd.publish(cmd)
            rate.sleep()
        rospy.loginfo("rotate: %.1f graus por tempo estimado", degrees)

    stop(pub_cmd)


def scan_area(pub_cmd, angular_speed=0.3, state=None):
    """Rotação de 360° para varrer a área ao redor do robô."""
    rospy.loginfo("scan_area: iniciando varredura 360 graus")
    rotate(pub_cmd, 360, angular_speed, state)
    rospy.loginfo("scan_area: concluido")


# ── Primitivas com percepção ─────────────────────────────────────────────────

def search_object(pub_cmd, class_name, state, angular_speed=0.35, timeout=15.0):
    """
    Gira devagar até encontrar class_name via YOLO.
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
    Aproxima do objeto com P-controller e campos potenciais.
    Velocidade linear = força_atrativa − força_repulsiva.
    Para se o freio de segurança disparar.
    """
    rospy.loginfo("approach_object: aproximando de '%s' (alvo=%.1fm)", class_name, target_dist)
    end_time = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)

    while rospy.Time.now() < end_time and not rospy.is_shutdown():
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
        error_angle = state.yolo_cx_norm

        if abs(error_angle) > deadzone_angle:
            cmd.angular.z = -kp_angular * error_angle
            cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))

        if state.lidar_valid:
            error_dist = state.lidar_distance - target_dist
        else:
            error_dist = 0.3 if state.yolo_area < 20000 else 0.0

        f_attractive = kp_linear * error_dist
        f_repulsive  = 0.0
        if (state.lidar_min_front_valid
                and state.lidar_min_front < REPULSIVE_INFLUENCE_DIST
                and state.lidar_min_front > 0.01):
            f_repulsive = KR * (1.0 / state.lidar_min_front
                                - 1.0 / REPULSIVE_INFLUENCE_DIST)

        net_linear = f_attractive - f_repulsive
        if abs(error_angle) < 0.3 and abs(error_dist) > deadzone_dist:
            cmd.linear.x = max(-max_linear, min(max_linear, net_linear))

        rospy.loginfo_throttle(1.0,
            "approach | dist=%.2f err=%.2f fa=%.2f fr=%.2f v=%.2f",
            state.lidar_distance if state.lidar_valid else -1,
            error_dist, f_attractive, f_repulsive, cmd.linear.x)

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
