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
"""

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class FollowerController:
    def __init__(self):
        rospy.init_node('follower_controller', anonymous=False)

        # ── Ganhos do controlador proporcional ────────────────────
        # Aumente Kp_angular se o robô demora para centralizar o alvo
        # Aumente Kp_linear  se o robô demora para ajustar a distância
        # Diminua se o robô oscilar (ficar indo e voltando)
        self.kp_angular = rospy.get_param('~kp_angular', 0.6)
        self.kp_linear  = rospy.get_param('~kp_linear',  0.4)

        # ── Limites de velocidade (respeitam specs do Limo) ───────
        self.max_linear  = rospy.get_param('~max_linear_vel',  0.3)  # m/s
        self.max_angular = rospy.get_param('~max_angular_vel', 0.8)  # rad/s

        # Zona morta — abaixo desse erro o robô considera que já chegou
        # Evita que o robô fique se mexendo infinitamente por erro residual
        self.deadzone_angle = rospy.get_param('~deadzone_angle', 0.05)  # normalizado
        self.deadzone_dist  = rospy.get_param('~deadzone_dist',  0.10)  # metros

        # Tópico de cmd_vel — confirme com `rostopic list` no Limo
        cmd_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')

        self.pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        self.sub_pose = rospy.Subscriber(
            '/target/pose', Float32MultiArray,
            self.pose_callback, queue_size=1)

        # Safety stop — se não receber pose por X segundos, para o robô
        self.last_msg_time = rospy.Time.now()
        self.safety_timeout = rospy.get_param('~safety_timeout', 0.5)
        rospy.Timer(rospy.Duration(0.1), self.safety_check)

        rospy.loginfo("follower_controller iniciado.")
        rospy.loginfo("Kp_angular=%.2f | Kp_linear=%.2f", self.kp_angular, self.kp_linear)
        rospy.loginfo("max_linear=%.2f m/s | max_angular=%.2f rad/s",
                      self.max_linear, self.max_angular)

    # ── Callback principal ────────────────────────────────────────

    def pose_callback(self, msg):
        self.last_msg_time = rospy.Time.now()

        found          = msg.data[0]
        erro_angular   = msg.data[3]   # normalizado [-1, 1]
        erro_distancia = msg.data[4]   # metros

        cmd = Twist()

        if found < 0.5:
            # Alvo perdido — para completamente
            self.pub_cmd.publish(cmd)
            return

        # ── Velocidade angular (rotação) ──────────────────────────
        if abs(erro_angular) > self.deadzone_angle:
            # Negativo porque: alvo à direita (erro>0) → virar à direita (angular<0)
            cmd.angular.z = -self.kp_angular * erro_angular
            cmd.angular.z = self._clamp(cmd.angular.z, self.max_angular)

        # ── Velocidade linear (avanço/recuo) ──────────────────────
        # Só avança/recua se o robô já estiver minimamente alinhado
        # Isso evita que o robô avance em diagonal
        if abs(erro_angular) < 0.3 and abs(erro_distancia) > self.deadzone_dist:
            cmd.linear.x = self.kp_linear * erro_distancia
            cmd.linear.x = self._clamp(cmd.linear.x, self.max_linear)

        self.pub_cmd.publish(cmd)

        rospy.loginfo_throttle(1.0,
            "CMD | linear=%.2f m/s | angular=%.2f rad/s",
            cmd.linear.x, cmd.angular.z)

    # ── Safety stop ───────────────────────────────────────────────

    def safety_check(self, event):
        """Para o robô se o pipeline travar ou um nó morrer."""
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