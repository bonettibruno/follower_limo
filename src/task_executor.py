#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
task_executor.py
Nó ROS que recebe uma lista de ações em JSON (/task/actions) e executa
cada uma em sequência usando robot_primitives.

Mantém o estado dos sensores atualizado via subscribers e passa ao primitives.

Tópicos:
  Assina:  /yolo/detection, /yolo/class, /lidar/distance_at_angle, /task/actions
  Publica: /cmd_vel, /task/status, /task/alert
"""

import json
import sys
import os
import rospy
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist

# Adiciona o diretório src ao path para importar robot_primitives
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_primitives as primitives
from robot_primitives import RobotState


class TaskExecutor:
    def __init__(self):
        rospy.init_node('task_executor', anonymous=False)

        self.state   = RobotState()
        self.running = False

        cmd_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.pub_cmd    = rospy.Publisher(cmd_topic,      Twist,  queue_size=1)
        self.pub_status = rospy.Publisher('/task/status', String, queue_size=1)
        self.pub_alert  = rospy.Publisher('/task/alert',  String, queue_size=1)

        rospy.Subscriber('/yolo/detection',          Float32MultiArray, self._yolo_detection_cb)
        rospy.Subscriber('/yolo/class',              String,            self._yolo_class_cb)
        rospy.Subscriber('/lidar/distance_at_angle', Float32MultiArray, self._lidar_cb)
        rospy.Subscriber('/task/actions',            String,            self._actions_cb)

        self._publish_status("IDLE")
        rospy.loginfo("task_executor iniciado. Aguardando /task/actions ...")

    # ── Callbacks de sensores ────────────────────────────────────────────────

    def _yolo_detection_cb(self, msg):
        self.state.yolo_found     = msg.data[0] > 0.5
        self.state.yolo_cx_norm   = msg.data[1]
        self.state.yolo_cy_norm   = msg.data[2]
        self.state.yolo_area      = msg.data[3]
        self.state.yolo_angle_deg = msg.data[4]

    def _yolo_class_cb(self, msg):
        self.state.yolo_class = msg.data

    def _lidar_cb(self, msg):
        self.state.lidar_distance = msg.data[1]
        self.state.lidar_valid    = msg.data[2] > 0.5

    # ── Callback de ações ────────────────────────────────────────────────────

    def _actions_cb(self, msg):
        if self.running:
            rospy.logwarn("task_executor: tarefa em andamento — nova lista ignorada.")
            return

        try:
            actions = json.loads(msg.data)
        except ValueError as e:
            rospy.logerr("task_executor: JSON invalido — %s", e)
            self._publish_status("ERROR: JSON invalido")
            return

        if not isinstance(actions, list):
            rospy.logerr("task_executor: esperado JSON array, recebido outro tipo")
            return

        rospy.loginfo("task_executor: recebido %d acoes", len(actions))
        self._execute_actions(actions)

    # ── Executor principal ───────────────────────────────────────────────────

    def _execute_actions(self, actions):
        self.running = True
        self._publish_status("RUNNING")

        for action in actions:
            if rospy.is_shutdown():
                break

            name = action.get("action", "")

            if name == "cancel":
                rospy.logwarn("task_executor: cancelado pelo usuario")
                break

            rospy.loginfo("task_executor: executando '%s' | params=%s", name, action)
            self._publish_status("RUNNING: " + name)

            try:
                self._run_action(name, action)
            except Exception as e:
                rospy.logerr("task_executor: erro em '%s': %s", name, e)
                self._publish_status("ERROR: " + str(e))
                break

        primitives.stop(self.pub_cmd)
        self.running = False
        self._publish_status("IDLE")
        rospy.loginfo("task_executor: sequencia concluida")

    def _run_action(self, name, params):
        if name == "stop":
            primitives.stop(self.pub_cmd)

        elif name == "go_forward":
            primitives.go_forward(
                self.pub_cmd,
                distance_m=float(params.get("distance_m", 0.5)),
                speed=float(params.get("speed", 0.2)))

        elif name == "rotate":
            primitives.rotate(
                self.pub_cmd,
                degrees=float(params.get("degrees", 90)),
                angular_speed=float(params.get("speed", 0.5)))

        elif name == "scan_area":
            primitives.scan_area(
                self.pub_cmd,
                angular_speed=float(params.get("speed", 0.3)))

        elif name == "search_object":
            primitives.search_object(
                self.pub_cmd,
                class_name=params.get("class_name", ""),
                state=self.state,
                timeout=float(params.get("timeout", 15.0)))

        elif name == "approach_object":
            primitives.approach_object(
                self.pub_cmd,
                class_name=params.get("class_name", ""),
                state=self.state,
                target_dist=float(params.get("target_dist", 0.5)),
                timeout=float(params.get("timeout", 20.0)))

        elif name == "alert":
            primitives.alert(self.pub_alert, params.get("message", ""))

        else:
            rospy.logwarn("task_executor: acao desconhecida '%s' — ignorada", name)

    # ── Utilitário ───────────────────────────────────────────────────────────

    def _publish_status(self, status):
        self.pub_status.publish(String(data=status))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = TaskExecutor()
        node.run()
    except rospy.ROSInterruptException:
        pass
