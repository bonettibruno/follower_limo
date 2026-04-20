#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
hsv_calibrator.py
Ferramenta standalone (sem ROS) para calibrar valores HSV.
Rode: python hsv_calibrator.py --image foto.jpg
      python hsv_calibrator.py --camera 0
"""

import cv2
import numpy as np
import argparse


def nothing(x):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', help='Caminho para imagem de referência')
    parser.add_argument('--camera', type=int, default=None,
                        help='Índice da câmera (0, 1, ...)')
    args = parser.parse_args()

    cv2.namedWindow('HSV Calibrator', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)

    # Trackbars
    cv2.createTrackbar('H_min', 'HSV Calibrator', 0, 179, nothing)
    cv2.createTrackbar('H_max', 'HSV Calibrator', 179, 179, nothing)
    cv2.createTrackbar('S_min', 'HSV Calibrator', 100, 255, nothing)
    cv2.createTrackbar('S_max', 'HSV Calibrator', 255, 255, nothing)
    cv2.createTrackbar('V_min', 'HSV Calibrator', 50, 255, nothing)
    cv2.createTrackbar('V_max', 'HSV Calibrator', 255, 255, nothing)

    cap = None
    static_frame = None

    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
    elif args.image:
        static_frame = cv2.imread(args.image)
        if static_frame is None:
            print("Erro: imagem nao encontrada.")
            return
    else:
        print("Informe --image ou --camera")
        return

    print("Ajuste os sliders. Pressione 'p' para printar valores. 'q' para sair.")

    while True:
        if cap:
            ret, frame = cap.read()
            if not ret:
                break
        else:
            frame = static_frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos('H_min', 'HSV Calibrator')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Calibrator')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Calibrator')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Calibrator')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Calibrator')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Calibrator')

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('HSV Calibrator', result)
        cv2.imshow('Mask', mask)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            print("\n========= VALORES HSV =========")
            print("HSV_LOWER = [{}, {}, {}]".format(h_min, s_min, v_min))
            print("HSV_UPPER = [{}, {}, {}]".format(h_max, s_max, v_max))
            print("===============================\n")

    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()