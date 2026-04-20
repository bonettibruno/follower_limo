#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
follower_controller.py

Nó final do pipeline — o "músculo" do sistema.
Recebe a posição estimada do alvo e converte em comandos de velocidade
para o robô. É aqui que o robô decide se vira, avança, recua ou para.

Fluxo:
  /target/pose ──> [follower_controller] ──> /cmd_vel

Lógica: controle proporcional simples (P-controller)
  velocidade_angular = Kp_angular * erro_angular_norm
  velocidade_linear  = Kp_linear  * erro_distancia
  (com saturação para não ultrapassar os limites do Limo)

Recovery behavior:
  Se o alvo sumir, o robô gira devagar no último sentido que viu o alvo
  tentando reencontrá-lo. Após recovery_timeout segundos sem achar, para.
"""

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class FollowerController:
    def __init__(self):
        rospy.init_node('follower_controller', anonymous=False)

        # ── Ganhos do controlador proporcional ────────────────────
        self.kp_angular = rospy.get_param('~kp_angular', 0.6)
        self.kp_linear  = rospy.get_param('~kp_linear',  0.4)

        # ── Limites de velocidade ─────────────────────────────────
        self.max_linear  = rospy.get_param('~max_linear_vel',  0.3)  # m/s
        self.max_angular = rospy.get_param('~max_angular_vel', 0.8)  # rad/s

        # ── Zona morta ────────────────────────────────────────────
        self.deadzone_angle = rospy.get_param('~deadzone_angle', 0.05)
        self.deadzone_dist  = rospy.get_param('~deadzone_dist',  0.10)

        # ── Recovery behavior ─────────────────────────────────────
        # Velocidade angular de busca quando o alvo some
        self.recovery_angular_vel = rospy.get_param('~recovery_angular_vel', 0.3)  # rad/s
        # Quanto tempo tenta procurar antes de desistir e parar
        self.recovery_timeout = rospy.get_param('~recovery_timeout', 4.0)  # segundos

        # ── Estado interno do recovery ────────────────────────────
        # Guarda o último sentido de rotação (+1 = esquerda, -1 = direita)
        self.last_angular_dir = 1.0
        # Momento em que perdeu o alvo
        self.lost_target_time = None

        # ── Tópico cmd_vel ────────────────────────────────────────
        cmd_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        self.sub_pose = rospy.Subscriber(
            '/target/pose', Float32MultiArray,
            self.pose_callback, queue_size=1)

        # ── Safety stop ───────────────────────────────────────────
        self.last_msg_time = rospy.Time.now()
        self.safety_timeout = rospy.get_param('~safety_timeout', 0.5)
        rospy.Timer(rospy.Duration(0.1), self.safety_check)

        rospy.loginfo("follower_controller iniciado.")
        rospy.loginfo("Kp_angular=%.2f | Kp_linear=%.2f", self.kp_angular, self.kp_linear)
        rospy.loginfo("Recovery: vel=%.2f rad/s | timeout=%.1f s",
                      self.recovery_angular_vel, self.recovery_timeout)

    # ── Callback principal ────────────────────────────────────────

    def pose_callback(self, msg):
        self.last_msg_time = rospy.Time.now()

        found          = msg.data[0]
        erro_angular   = msg.data[3]   # normalizado [-1, 1]
        erro_distancia = msg.data[4]   # metros

        cmd = Twist()

        if found < 0.5:
            self._recovery(cmd)
            self.pub_cmd.publish(cmd)
            return

        # ── Alvo visível — reseta estado de recovery ──────────────
        self.lost_target_time = None

        # Guarda o último sentido de rotação para usar no recovery
        # (sinal do erro angular indica para qual lado o alvo foi)
        if abs(erro_angular) > self.deadzone_angle:
            self.last_angular_dir = 1.0 if erro_angular > 0 else -1.0

        # ── Velocidade angular ────────────────────────────────────
        if abs(erro_angular) > self.deadzone_angle:
            cmd.angular.z = -self.kp_angular * erro_angular
            cmd.angular.z = self._clamp(cmd.angular.z, self.max_angular)

        # ── Velocidade linear ─────────────────────────────────────
        if abs(erro_angular) < 0.3 and abs(erro_distancia) > self.deadzone_dist:
            cmd.linear.x = self.kp_linear * erro_distancia
            cmd.linear.x = self._clamp(cmd.linear.x, self.max_linear)

        self.pub_cmd.publish(cmd)

        rospy.loginfo_throttle(1.0,
            "CMD | linear=%.2f m/s | angular=%.2f rad/s",
            cmd.linear.x, cmd.angular.z)

    # ── Recovery behavior ─────────────────────────────────────────

    def _recovery(self, cmd):
        """
        Gira devagar no último sentido que viu o alvo tentando reencontrá-lo.
        Após recovery_timeout segundos sem achar, para completamente.
        """
        now = rospy.Time.now()

        # Marca o momento em que perdeu o alvo
        if self.lost_target_time is None:
            self.lost_target_time = now
            rospy.logwarn("Alvo perdido — iniciando recovery...")

        elapsed = (now - self.lost_target_time).to_sec()

        if elapsed < self.recovery_timeout:
            # Gira devagar no último sentido conhecido
            cmd.angular.z = self.last_angular_dir * self.recovery_angular_vel
            rospy.loginfo_throttle(1.0,
                "Recovery | girando %.2f rad/s | ha %.1f s",
                cmd.angular.z, elapsed)
        else:
            # Desistiu — para e aguarda
            rospy.logwarn_throttle(2.0,
                "Recovery timeout — aguardando alvo.")

    # ── Safety stop ───────────────────────────────────────────────

    def safety_check(self, event):
        elapsed = (rospy.Time.now() - self.last_msg_time).to_sec()
        if elapsed > self.safety_timeout:
            rospy.logwarn_throttle(2.0,
                "Safety stop — sem /target/pose ha %.1f s", elapsed)
            self.pub_cmd.publish(Twist())

    # ── Utilitário ────────────────────────────────────────────────

    def _clamp(self, value, max_abs):
        return max(-max_abs, min(max_abs, value))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = FollowerController()
        node.run()
    except rospy.ROSInterruptException:
        pass