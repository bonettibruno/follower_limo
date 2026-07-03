#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
plot_task.py
Gera gráficos e métricas a partir de um arquivo CSV produzido pelo task_logger.

Uso:
  python tools/plot_task.py logs/task_....csv
  python tools/plot_task.py logs/task_....csv --target 0.5
  python tools/plot_task.py logs/task_....csv --target 0.5 --separate
  python tools/plot_task.py logs/task_....csv --target 0.5 --separate --outdir logs/exp1

Saída:
  - Modo padrão: um único PNG (6 painéis) com o nome do CSV
  - Modo --separate: cada painel salvo como um PNG individual numa pasta
  - Métricas impressas no terminal

Painéis:
  1. Distância ao alvo (com linha da distância alvo)
  2. Distância mínima frontal (LiDAR bruto) com freio e influência repulsiva
  3. Forças atrativa e repulsiva (campos potenciais)
  4. Velocidade linear e angular enviadas ao robô
  5. Ângulo do alvo (YOLO) — convergência para o centro
  6. Trajetória do robô (x, y) via odometria, com a posição estimada do objeto
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
except ImportError:
    print("matplotlib nao instalado. Execute: pip install matplotlib")
    sys.exit(1)

# Parâmetros do robot_primitives — devem bater com os valores do código
SAFETY_DIST              = 0.20
REPULSIVE_INFLUENCE_DIST = 0.50
KR                       = 0.15
KP_LINEAR                = 0.40
TARGET_DIST_DEFAULT      = 0.50
SETTLE_TOL               = 0.05   # tolerância (m) para considerar "acomodado"


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


def median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def compute_forces(lidar_dist, lidar_valid, lidar_min, target_dist):
    if lidar_valid:
        f_att = KP_LINEAR * (lidar_dist - target_dist)
    else:
        f_att = 0.0
    f_rep = 0.0
    if lidar_min < REPULSIVE_INFLUENCE_DIST and lidar_min > 0.01:
        f_rep = KR * (1.0 / lidar_min - 1.0 / REPULSIVE_INFLUENCE_DIST)
    return f_att, f_rep


def estimate_object_position(lidar_dist, lidar_valid, yolo_angle, yolo_found,
                             odom_x, odom_y, odom_yaw):
    """
    Estima a posição do objeto no referencial da odometria a partir de cada
    deteção válida: posição do robô + distância na direção (yaw - ângulo do alvo).
    Retorna a mediana (robusta a ruído), ou None.
    """
    xs, ys = [], []
    for d, dv, ang, yf, ox, oy, yaw in zip(
            lidar_dist, lidar_valid, yolo_angle, yolo_found,
            odom_x, odom_y, odom_yaw):
        if dv and yf and d > 0.01:
            bearing = yaw - math.radians(ang)
            xs.append(ox + d * math.cos(bearing))
            ys.append(oy + d * math.sin(bearing))
    if not xs:
        return None
    return (median(xs), median(ys))


def compute_metrics(t, lidar_dist, lidar_valid, lidar_min,
                    yolo_angle, yolo_found, odom_x, odom_y, target_dist):
    m = {}

    valid_idx = [i for i, v in enumerate(lidar_valid) if v]
    if valid_idx:
        m['dist_inicial'] = lidar_dist[valid_idx[0]]
        m['dist_final']   = lidar_dist[valid_idx[-1]]
        m['erro_final']   = m['dist_final'] - target_dist
    else:
        m['dist_inicial'] = m['dist_final'] = m['erro_final'] = None

    m['t_acomodacao'] = None
    if valid_idx and m['erro_final'] is not None and abs(m['erro_final']) <= 2 * SETTLE_TOL:
        for i in valid_idx:
            if abs(lidar_dist[i] - target_dist) <= SETTLE_TOL:
                tail = [j for j in valid_idx if j >= i]
                if all(abs(lidar_dist[j] - target_dist) <= 2 * SETTLE_TOL for j in tail):
                    m['t_acomodacao'] = t[i]
                    break

    fronts = [d for d in lidar_min if 0.01 < d < 10.0]
    if fronts:
        m['obstaculo_min'] = min(fronts)
        m['freio_disparou'] = m['obstaculo_min'] < SAFETY_DIST
    else:
        m['obstaculo_min'] = None
        m['freio_disparou'] = False

    ang_idx = [i for i, f in enumerate(yolo_found) if f]
    if ang_idx:
        m['ang_inicial'] = yolo_angle[ang_idx[0]]
        m['ang_final']   = yolo_angle[ang_idx[-1]]
    else:
        m['ang_inicial'] = m['ang_final'] = None

    path = 0.0
    for i in range(1, len(odom_x)):
        path += math.hypot(odom_x[i] - odom_x[i - 1], odom_y[i] - odom_y[i - 1])
    m['caminho'] = path
    m['duracao'] = (t[-1] - t[0]) if t else 0.0
    return m


def format_metrics(m, target_dist):
    def fmt(v, casas=2, unidade=''):
        return '---' if v is None else '{:.{c}f}{u}'.format(v, c=casas, u=unidade)
    linhas = []
    linhas.append("Distância alvo (setpoint): {}".format(fmt(target_dist, 2, ' m')))
    linhas.append("Distância inicial -> final: {} -> {}".format(
        fmt(m['dist_inicial'], 2, ' m'), fmt(m['dist_final'], 2, ' m')))
    if m['erro_final'] is not None:
        linhas.append("Erro final de posição: {:+.2f} m ({:.1f} cm)".format(
            m['erro_final'], m['erro_final'] * 100))
    else:
        linhas.append("Erro final de posição: ---")
    linhas.append("Tempo de acomodação: {}".format(
        fmt(m['t_acomodacao'], 1, ' s') if m['t_acomodacao'] is not None
        else 'não acomodou'))
    linhas.append("Obstáculo frontal mín.: {} {}".format(
        fmt(m['obstaculo_min'], 2, ' m'),
        '(FREIO ACIONADO)' if m['freio_disparou'] else ''))
    linhas.append("Ângulo do alvo inicial -> final: {} -> {}".format(
        fmt(m['ang_inicial'], 1, ' graus'), fmt(m['ang_final'], 1, ' graus')))
    linhas.append("Caminho percorrido (odom.): {}".format(fmt(m['caminho'], 2, ' m')))
    linhas.append("Duração: {}".format(fmt(m['duracao'], 1, ' s')))
    return linhas


def plot(path, target_dist, separate=False, outdir=None):
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
    odom_yaw    = [parse_float(r.get('odom_yaw_rad')) for r in rows]
    status      = [r['status']                         for r in rows]

    metrics = compute_metrics(t, lidar_dist, lidar_valid, lidar_min,
                              yolo_angle, yolo_found, odom_x, odom_y, target_dist)
    obj_pos = estimate_object_position(lidar_dist, lidar_valid, yolo_angle,
                                       yolo_found, odom_x, odom_y, odom_yaw)

    linhas = format_metrics(metrics, target_dist)
    print("\n===== MÉTRICAS: {} =====".format(os.path.basename(path)))
    for ln in linhas:
        print("  " + ln)
    print("")

    f_att_list, f_rep_list = [], []
    for d, v, mn in zip(lidar_dist, lidar_valid, lidar_min):
        fa, fr = compute_forces(d, v, mn, target_dist)
        f_att_list.append(fa)
        f_rep_list.append(fr)

    action_changes = []
    prev = ""
    for i, s in enumerate(status):
        if s != prev and s.startswith("RUNNING:"):
            action_changes.append((t[i], s.replace("RUNNING: ", "")))
        prev = s

    def add_action_lines(ax):
        for tx, label in action_changes:
            ax.axvline(x=tx, color='gray', linestyle='--', linewidth=0.8, alpha=0.6)
            ax.text(tx + 0.05, ax.get_ylim()[1] * 0.95, label,
                    fontsize=6, color='gray', rotation=90, va='top')

    # ── Funções de desenho de cada painel (fecham sobre os dados acima) ──

    def p_distancia(ax):
        dist_plot = [d if v else None for d, v in zip(lidar_dist, lidar_valid)]
        ax.plot(t, dist_plot, color='steelblue', label='distância ao alvo')
        ax.axhline(y=target_dist, color='green', linestyle='--', linewidth=1,
                   label='distância alvo ({:.2f}m)'.format(target_dist))
        if metrics['t_acomodacao'] is not None:
            ax.axvline(x=metrics['t_acomodacao'], color='purple', linestyle=':',
                       linewidth=1, label='acomodação')
        if metrics['erro_final'] is not None:
            txt = 'erro final: {:+.2f} m'.format(metrics['erro_final'])
            if metrics['t_acomodacao'] is not None:
                txt += '\nacomodou em {:.1f} s'.format(metrics['t_acomodacao'])
            ax.text(0.97, 0.95, txt, transform=ax.transAxes, fontsize=8,
                    ha='right', va='top',
                    bbox=dict(boxstyle='round', fc='lightyellow', ec='gray', alpha=0.9))
        ax.set_xlabel('tempo (s)')
        ax.set_ylabel('distância (m)')
        ax.set_title('Distância ao alvo')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        add_action_lines(ax)

    def p_lidar(ax):
        lidar_min_plot = [mn if mn < 10 else None for mn in lidar_min]
        ax.plot(t, lidar_min_plot, color='darkorange', label='LiDAR min frontal')
        ax.axhline(y=SAFETY_DIST, color='red', linestyle='--', linewidth=1,
                   label='freio de segurança ({:.2f}m)'.format(SAFETY_DIST))
        ax.axhline(y=REPULSIVE_INFLUENCE_DIST, color='gold', linestyle='--', linewidth=1,
                   label='influência repulsiva ({:.2f}m)'.format(REPULSIVE_INFLUENCE_DIST))
        ax.set_xlabel('tempo (s)')
        ax.set_ylabel('distância (m)')
        ax.set_title('Distância mínima frontal (obstáculos)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        add_action_lines(ax)

    def p_forcas(ax):
        ax.plot(t, f_att_list, color='steelblue', label='força atrativa')
        ax.plot(t, f_rep_list, color='tomato',    label='força repulsiva')
        net = [a - r for a, r in zip(f_att_list, f_rep_list)]
        ax.plot(t, net, color='purple', linestyle='--', label='resultante (v linear)')
        ax.axhline(y=0, color='black', linewidth=0.5)
        ax.set_xlabel('tempo (s)')
        ax.set_ylabel('força (m/s equiv.)')
        ax.set_title('Campos potenciais')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        add_action_lines(ax)

    def p_cmdvel(ax):
        ax.plot(t, cmd_lin, color='steelblue', label='linear (m/s)')
        ax.plot(t, cmd_ang, color='tomato',    label='angular (rad/s)')
        ax.axhline(y=0, color='black', linewidth=0.5)
        ax.set_xlabel('tempo (s)')
        ax.set_ylabel('velocidade')
        ax.set_title('Comandos /cmd_vel')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        add_action_lines(ax)

    def p_angulo(ax):
        angle_plot = [a if f else None for a, f in zip(yolo_angle, yolo_found)]
        ax.plot(t, angle_plot, color='mediumseagreen', label='ângulo do alvo')
        ax.axhline(y=0, color='black', linestyle='--', linewidth=0.8, label='centro')
        ax.set_xlabel('tempo (s)')
        ax.set_ylabel('ângulo (graus)')
        ax.set_title('Ângulo do alvo (YOLO)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        add_action_lines(ax)

    def p_trajetoria(ax):
        ax.plot(odom_x, odom_y, color='steelblue', linewidth=1.5, label='trajetória')
        if odom_x:
            ax.plot(odom_x[0],  odom_y[0],  'go', markersize=8, label='início')
            ax.plot(odom_x[-1], odom_y[-1], 'rs', markersize=8, label='fim')
        if obj_pos is not None:
            ox, oy = obj_pos
            ax.plot(ox, oy, marker='*', color='darkorange', markersize=18,
                    linestyle='None', label='objeto (estim.)')
            circ = plt.Circle((ox, oy), target_dist, fill=False, linestyle='--',
                              color='orange', alpha=0.7)
            ax.add_patch(circ)
            all_x = odom_x + [ox - target_dist, ox + target_dist]
            all_y = odom_y + [oy - target_dist, oy + target_dist]
            mx = 0.15 * max(max(all_x) - min(all_x), max(all_y) - min(all_y), 0.5)
            ax.set_xlim(min(all_x) - mx, max(all_x) + mx)
            ax.set_ylim(min(all_y) - mx, max(all_y) + mx)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Trajetória (odometria)')
        ax.legend(fontsize=8)
        ax.set_aspect('equal', 'box')
        ax.grid(True, alpha=0.3)

    panels = [
        ("1_distancia_ao_alvo", p_distancia, False),
        ("2_lidar_frontal",     p_lidar,     False),
        ("3_campos_potenciais", p_forcas,    False),
        ("4_cmd_vel",           p_cmdvel,    False),
        ("5_angulo_yolo",       p_angulo,    False),
        ("6_trajetoria",        p_trajetoria, True),  # True = painel quadrado
    ]

    if separate:
        base = outdir if outdir else path.replace('.csv', '')
        if not os.path.exists(base):
            os.makedirs(base)
        for fname, drawer, square in panels:
            fig, ax = plt.subplots(figsize=(6.5, 6.5) if square else (8, 5))
            drawer(ax)
            plt.tight_layout()
            out = os.path.join(base, fname + ".png")
            plt.savefig(out, dpi=150)
            plt.close(fig)
            print("  painel salvo: {}".format(out))
        print("\n6 paineis salvos em: {}".format(base))
    else:
        fig, axes = plt.subplots(3, 2, figsize=(14, 12.6))
        fig.suptitle(os.path.basename(path).replace('.csv', ''), fontsize=12)
        grid = [axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1], axes[2, 0], axes[2, 1]]
        for (fname, drawer, square), ax in zip(panels, grid):
            drawer(ax)
        fig.text(0.5, 0.012, "   |   ".join(linhas), ha='center', va='bottom',
                 fontsize=8, family='monospace',
                 bbox=dict(boxstyle='round', fc='whitesmoke', ec='gray', alpha=0.9))
        plt.tight_layout(rect=[0, 0.03, 1, 0.98])
        out_path = path.replace('.csv', '.png')
        plt.savefig(out_path, dpi=150)
        print("Grafico salvo em: {}".format(out_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Gera graficos e metricas a partir de um log CSV do task_logger')
    parser.add_argument('csv_file', help='caminho para o arquivo .csv')
    parser.add_argument('--target', type=float, default=TARGET_DIST_DEFAULT,
                        help='distancia-alvo (setpoint) usada no experimento, em metros '
                             '(padrao: %(default)s)')
    parser.add_argument('--separate', action='store_true',
                        help='salva cada painel como um PNG separado numa pasta')
    parser.add_argument('--outdir', default=None,
                        help='pasta de saida para os paineis separados '
                             '(padrao: nome do CSV)')
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print("Arquivo nao encontrado: {}".format(args.csv_file))
        sys.exit(1)

    plot(args.csv_file, args.target, separate=args.separate, outdir=args.outdir)
