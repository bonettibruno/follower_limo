#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
task_logger.py
Nó ROS que grava dados dos sensores durante a execução de tarefas em CSV.

Inicia a gravação quando /task/status muda de IDLE para RUNNING.
Encerra e salva o arquivo quando o status volta para IDLE ou ERROR.

Cada linha do CSV contém um snapshot dos tópicos a ~10 Hz:
  timestamp_s, elapsed_s, status, yolo_found, yolo_class, yolo_angle_deg,
  lidar_distance, lidar_valid, lidar_min_front, odom_x, odom_y, odom_yaw,
  cmd_linear_x, cmd_angular_z

Os arquivos são salvos em <log_dir>/task_YYYYMMDD_HHMMSS.csv
Por padrão log_dir = <pacote>/logs/
"""

import csv
import math
import os
import rospy
import rospkg
from datetime import datetime
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

FRONT_SECTOR_DEG = 30.0


class TaskLogger:
    def __init__(self):
        rospy.init_node('task_logger', anonymous=False)

        pkg_path = rospkg.RosPack().get_path('follower_limo')
        default_log_dir = os.path.join(pkg_path, 'logs')
        self.log_dir = rospy.get_param('~log_dir', default_log_dir)

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            rospy.loginfo("task_logger: criado diretório %s", self.log_dir)

        # Estado dos sensores
        self.status       = "IDLE"
        self.yolo_found   = False
        self.yolo_class   = ""
        self.yolo_angle   = 0.0
        self.lidar_dist   = 0.0
        self.lidar_valid  = False
        self.lidar_min    = float('inf')
        self.odom_x       = 0.0
        self.odom_y       = 0.0
        self.odom_yaw     = 0.0
        self.cmd_linear   = 0.0
        self.cmd_angular  = 0.0

        # Controle de gravação
        self._csv_file    = None
        self._csv_writer  = None
        self._recording   = False
        self._start_time  = None
        self._current_path = None

        rospy.Subscriber('/task/status',            String,            self._status_cb)
        rospy.Subscriber('/yolo/detection',         Float32MultiArray, self._yolo_detection_cb)
        rospy.Subscriber('/yolo/class',             String,            self._yolo_class_cb)
        rospy.Subscriber('/lidar/distance_at_angle',Float32MultiArray, self._lidar_cb)
        rospy.Subscriber('/scan',                   LaserScan,         self._scan_cb)
        rospy.Subscriber('/odom',                   Odometry,          self._odom_cb)
        rospy.Subscriber('/cmd_vel',                Twist,             self._cmd_cb)

        rospy.Timer(rospy.Duration(0.1), self._log_tick)  # 10 Hz
        rospy.loginfo("task_logger: aguardando tarefas. Logs em: %s", self.log_dir)

    # ── Callbacks de tópicos ─────────────────────────────────────────────────

    def _status_cb(self, msg):
        new_status = msg.data
        if not self._recording and new_status.startswith("RUNNING"):
            self._start_recording()
        if self._recording and (new_status == "IDLE" or new_status.startswith("ERROR")):
            self._stop_recording()
        self.status = new_status

    def _yolo_detection_cb(self, msg):
        self.yolo_found = msg.data[0] > 0.5
        self.yolo_angle = msg.data[4]

    def _yolo_class_cb(self, msg):
        self.yolo_class = msg.data

    def _lidar_cb(self, msg):
        self.lidar_dist  = msg.data[1]
        self.lidar_valid = msg.data[2] > 0.5

    def _scan_cb(self, msg):
        front_rad = math.radians(FRONT_SECTOR_DEG)
        valid = [
            r for i, r in enumerate(msg.ranges)
            if (abs(msg.angle_min + i * msg.angle_increment) <= front_rad
                and msg.range_min < r < msg.range_max
                and not math.isnan(r) and not math.isinf(r))
        ]
        self.lidar_min = min(valid) if valid else float('inf')

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        self.odom_x   = pos.x
        self.odom_y   = pos.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def _cmd_cb(self, msg):
        self.cmd_linear  = msg.linear.x
        self.cmd_angular = msg.angular.z

    # ── Gravação ─────────────────────────────────────────────────────────────

    def _start_recording(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._current_path = os.path.join(self.log_dir, "task_{}.csv".format(stamp))
        self._csv_file   = open(self._current_path, 'w')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp_s", "elapsed_s", "status",
            "yolo_found", "yolo_class", "yolo_angle_deg",
            "lidar_distance_m", "lidar_valid", "lidar_min_front_m",
            "odom_x", "odom_y", "odom_yaw_rad",
            "cmd_linear_x", "cmd_angular_z"
        ])
        self._start_time = rospy.Time.now()
        self._recording  = True
        rospy.loginfo("task_logger: gravando em %s", self._current_path)

    def _stop_recording(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
        self._recording = False
        rospy.loginfo("task_logger: log salvo em %s", self._current_path)

    def _log_tick(self, event):
        if not self._recording or self._csv_writer is None:
            return
        now     = rospy.Time.now()
        elapsed = (now - self._start_time).to_sec()
        lidar_min_str = "{:.3f}".format(self.lidar_min) if self.lidar_min != float('inf') else "inf"
        self._csv_writer.writerow([
            "{:.3f}".format(now.to_sec()),
            "{:.3f}".format(elapsed),
            self.status,
            int(self.yolo_found),
            self.yolo_class,
            "{:.2f}".format(self.yolo_angle),
            "{:.3f}".format(self.lidar_dist),
            int(self.lidar_valid),
            lidar_min_str,
            "{:.3f}".format(self.odom_x),
            "{:.3f}".format(self.odom_y),
            "{:.3f}".format(self.odom_yaw),
            "{:.3f}".format(self.cmd_linear),
            "{:.3f}".format(self.cmd_angular),
        ])

    def run(self):
        rospy.spin()
        if self._recording:
            self._stop_recording()


if __name__ == '__main__':
    try:
        node = TaskLogger()
        node.run()
    except rospy.ROSInterruptException:
        pass
