#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
plot_task.py
Gera gráficos a partir de um arquivo CSV produzido pelo task_logger.

Uso:
  python tools/plot_task.py logs/task_20240101_120000.csv

Saída: PNG com o mesmo nome do CSV no mesmo diretório.

Painéis gerados:
  1. Distância ao alvo ao longo do tempo (com linha da distância alvo)
  2. Distância mínima frontal (LiDAR bruto) com linha do freio de segurança
  3. Forças atrativa e repulsiva calculadas (campos potenciais)
  4. Velocidade linear e angular enviadas ao robô
  5. Ângulo do alvo (YOLO) — mostra convergência para o centro
  6. Trajetória do robô (x, y) via odometria
"""

import sys
import os
import csv
import math
import argparse

try:
    import matplotlib
    matplotlib.use('Agg')  # sem display — funciona via SSH
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
except ImportError:
    print("matplotlib nao instalado. Execute: pip install matplotlib")
    sys.exit(1)

# Parâmetros do robot_primitives — devem bater com os valores do código
SAFETY_DIST              = 0.20
REPULSIVE_INFLUENCE_DIST = 0.50
KR                       = 0.15
KP_LINEAR                = 0.40
TARGET_DIST_DEFAULT      = 0.50


def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def parse_float(val, default=0.0):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def compute_forces(lidar_dist, lidar_valid, lidar_min, target_dist):
    """Recalcula as forças com as mesmas fórmulas do robot_primitives."""
    if lidar_valid:
        error = lidar_dist - target_dist
        f_att = KP_LINEAR * error
    else:
        f_att = 0.0

    f_rep = 0.0
    if lidar_min < REPULSIVE_INFLUENCE_DIST and lidar_min > 0.01:
        f_rep = KR * (1.0 / lidar_min - 1.0 / REPULSIVE_INFLUENCE_DIST)

    return f_att, f_rep


def plot(path):
    rows = load_csv(path)
    if not rows:
        print("Arquivo vazio ou sem dados.")
        return

    t           = [parse_float(r['elapsed_s'])       for r in rows]
    lidar_dist  = [parse_float(r['lidar_distance_m']) for r in rows]
    lidar_valid = [int(r['lidar_valid']) == 1         for r in rows]
    lidar_min   = [parse_float(r['lidar_min_front_m'], REPULSIVE_INFLUENCE_DIST + 1)
                   for r in rows]
    cmd_lin     = [parse_float(r['cmd_linear_x'])     for r in rows]
    cmd_ang     = [parse_float(r['cmd_angular_z'])    for r in rows]
    yolo_angle  = [parse_float(r['yolo_angle_deg'])   for r in rows]
    yolo_found  = [int(r['yolo_found']) == 1          for r in rows]
    odom_x      = [parse_float(r['odom_x'])           for r in rows]
    odom_y      = [parse_float(r['odom_y'])           for r in rows]
    status      = [r['status']                         for r in rows]

    # Detecta distância alvo a partir dos dados (mediana dos últimos 20% quando parado)
    valid_dists = [d for d, v in zip(lidar_dist, lidar_valid) if v]
    if valid_dists:
        target_dist = valid_dists[-1] if valid_dists else TARGET_DIST_DEFAULT
    else:
        target_dist = TARGET_DIST_DEFAULT

    # Calcula forças
    f_att_list, f_rep_list = [], []
    for d, v, m in zip(lidar_dist, lidar_valid, lidar_min):
        fa, fr = compute_forces(d, v, m, target_dist)
        f_att_list.append(fa)
        f_rep_list.append(fr)

    # Identifica mudanças de ação para linhas verticais
    action_changes = []
    prev = ""
    for i, s in enumerate(status):
        if s != prev and s.startswith("RUNNING:"):
            action_changes.append((t[i], s.replace("RUNNING: ", "")))
        prev = s

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(os.path.basename(path).replace('.csv', ''), fontsize=12)

    def add_action_lines(ax):
        for tx, label in action_changes:
            ax.axvline(x=tx, color='gray', linestyle='--', linewidth=0.8, alpha=0.6)
            ax.text(tx + 0.05, ax.get_ylim()[1] * 0.95, label,
                    fontsize=6, color='gray', rotation=90, va='top')

    # 1. Distância ao alvo
    ax = axes[0, 0]
    dist_plot = [d if v else None for d, v in zip(lidar_dist, lidar_valid)]
    ax.plot(t, dist_plot, color='steelblue', label='distância ao alvo')
    ax.axhline(y=target_dist, color='green', linestyle='--', linewidth=1,
               label='distância alvo ({:.2f}m)'.format(target_dist))
    ax.set_ylabel('distância (m)')
    ax.set_title('Distância ao alvo')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    add_action_lines(ax)

    # 2. LiDAR mínimo frontal
    ax = axes[0, 1]
    lidar_min_plot = [m if m < 10 else None for m in lidar_min]
    ax.plot(t, lidar_min_plot, color='darkorange', label='LiDAR min frontal')
    ax.axhline(y=SAFETY_DIST, color='red', linestyle='--', linewidth=1,
               label='freio de segurança ({:.2f}m)'.format(SAFETY_DIST))
    ax.axhline(y=REPULSIVE_INFLUENCE_DIST, color='gold', linestyle='--', linewidth=1,
               label='influência repulsiva ({:.2f}m)'.format(REPULSIVE_INFLUENCE_DIST))
    ax.set_ylabel('distância (m)')
    ax.set_title('Distância mínima frontal (obstáculos)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    add_action_lines(ax)

    # 3. Campos potenciais
    ax = axes[1, 0]
    ax.plot(t, f_att_list, color='steelblue', label='força atrativa')
    ax.plot(t, f_rep_list, color='tomato',    label='força repulsiva')
    net = [a - r for a, r in zip(f_att_list, f_rep_list)]
    ax.plot(t, net, color='purple', linestyle='--', label='resultante (v linear)')
    ax.axhline(y=0, color='black', linewidth=0.5)
    ax.set_ylabel('força (m/s equiv.)')
    ax.set_title('Campos potenciais')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    add_action_lines(ax)

    # 4. Velocidades enviadas
    ax = axes[1, 1]
    ax.plot(t, cmd_lin, color='steelblue', label='linear (m/s)')
    ax.plot(t, cmd_ang, color='tomato',    label='angular (rad/s)')
    ax.axhline(y=0, color='black', linewidth=0.5)
    ax.set_ylabel('velocidade')
    ax.set_title('Comandos /cmd_vel')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    add_action_lines(ax)

    # 5. Ângulo do alvo (YOLO)
    ax = axes[2, 0]
    angle_plot = [a if f else None for a, f in zip(yolo_angle, yolo_found)]
    ax.plot(t, angle_plot, color='mediumseagreen', label='ângulo do alvo')
    ax.axhline(y=0, color='black', linestyle='--', linewidth=0.8, label='centro')
    ax.set_ylabel('ângulo (graus)')
    ax.set_xlabel('tempo (s)')
    ax.set_title('Ângulo do alvo (YOLO)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    add_action_lines(ax)

    # 6. Trajetória (odometria)
    ax = axes[2, 1]
    ax.plot(odom_x, odom_y, color='steelblue', linewidth=1.5)
    if odom_x:
        ax.plot(odom_x[0],  odom_y[0],  'go', markersize=8, label='início')
        ax.plot(odom_x[-1], odom_y[-1], 'rs', markersize=8, label='fim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajetória (odometria)')
    ax.legend(fontsize=8)
    ax.set_aspect('equal', 'box')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = path.replace('.csv', '.png')
    plt.savefig(out_path, dpi=150)
    print("Grafico salvo em: {}".format(out_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Gera graficos a partir de um log CSV do task_logger')
    parser.add_argument('csv_file', help='caminho para o arquivo .csv')
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print("Arquivo nao encontrado: {}".format(args.csv_file))
        sys.exit(1)

    plot(args.csv_file)
