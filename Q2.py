#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Streaming shape detection on video using OpenCV.

- Reads video as a stream (cv2.VideoCapture), processes frame-by-frame
- Detects colored shapes (red, yellow, blue, magenta, bright green)
- Draws outlines and centers on each frame
- Writes annotated video (same size/FPS as input by default)
- Optionally displays a preview window
- Saves per-frame detections to CSV

Usage:
    python Q2.py --input PennAir_2024_App_Dynamic.mp4 \
                                 --output annotated.mp4 \
                                 --csv detections.csv \
                                 --preview 1
"""
import cv2
import numpy as np
import argparse
import csv
from pathlib import Path
from collections import defaultdict
import time

def classify_shape(contour):
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return "unknown"
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    v = len(approx)
    area = cv2.contourArea(contour)
    if area < 200:
        return None
    circ = 4 * np.pi * area / (peri * peri)
    if v == 3:
        return "triangle"
    elif v == 4:
        return "quadrilateral"
    elif v == 5:
        return "pentagon"
    else:
        if circ > 0.75:
            return "circle"
        return "polygon"

def color_masks(hsv):
    # HSV ranges (H 0-179 in OpenCV)
    H_low  = cv2.getTrackbarPos("H_low",  "annotated")
    H_high = cv2.getTrackbarPos("H_high", "annotated")
    S_low  = cv2.getTrackbarPos("S_low",  "annotated")
    S_high = cv2.getTrackbarPos("S_high", "annotated")
    V_low  = cv2.getTrackbarPos("V_low",  "annotated")
    V_high = cv2.getTrackbarPos("V_high", "annotated")

    lower = np.array([H_low, S_low, V_low], dtype=np.uint8)
    upper = np.array([H_high, S_high, V_high], dtype=np.uint8)

    # 用新的 lower/upper 替换绿色的掩膜阈值
    # green_mask = cv2.inRange(hsv, lower, upper)

    ranges = {
        "red1": ((0, 120, 70), (10, 255, 255)),
        "red2": ((170, 120, 70), (179, 255, 255)),
        "yellow": ((20, 120, 120), (35, 255, 255)),
        "blue": ((100, 120, 70), (130, 255, 255)),
        "magenta": ((140, 120, 70), (170, 255, 255)),
        # "bright_green": ((lower, upper)),
        "bright_green": ((45, 80, 161), (85, 255, 255)),  # high V to avoid grass
    }
    masks = {}
    for name, (lo, hi) in ranges.items():
        lo = np.array(lo, dtype=np.uint8)
        hi = np.array(hi, dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
        masks[name] = m

    # Merge red wrap-around
    red = cv2.bitwise_or(masks["red1"], masks["red2"])
    masks_out = {"red": red, "yellow": masks["yellow"], "blue": masks["blue"],
                 "magenta": masks["magenta"], "bright_green": masks["bright_green"]}
    return masks_out

def detect_on_frame(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    masks = color_masks(hsv)

    annot = frame_bgr.copy()
    detections = []

    for label, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < 600:
                continue
            shape = classify_shape(c)
            if shape is None:
                continue
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
            else:
                (x, y), _ = cv2.minEnclosingCircle(c)
                cx, cy = int(x), int(y)

            cv2.drawContours(annot, [c], -1, (255,255,255), 3)
            cv2.circle(annot, (cx, cy), 6, (0,0,255), -1)
            text = f"{shape}"
            cv2.putText(annot, text, (cx+8, cy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(annot, text, (cx+8, cy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            detections.append((shape, label, cx, cy, int(area)))
    return annot, detections

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="path to input video (or camera index like 0)")
    ap.add_argument("--output", default="annotated.mp4", help="path to save annotated video")
    ap.add_argument("--csv", default="detections.csv", help="path to save per-frame detections CSV")
    ap.add_argument("--preview", type=int, default=0, help="1 to show live preview window")
    ap.add_argument("--fourcc", default="mp4v", help="fourcc for VideoWriter, e.g., mp4v, avc1, XVID")
    args = ap.parse_args()
    cv2.namedWindow("annotated", cv2.WINDOW_NORMAL)  # 让窗口可调大小

    # 创建 HSV 阈值调节滑块
    cv2.createTrackbar("H_low",  "annotated", 35, 179, lambda x: None)
    cv2.createTrackbar("H_high", "annotated", 90, 179, lambda x: None)
    cv2.createTrackbar("S_low",  "annotated", 60, 255, lambda x: None)
    cv2.createTrackbar("S_high", "annotated", 255, 255, lambda x: None)
    cv2.createTrackbar("V_low",  "annotated", 80, 255, lambda x: None)
    cv2.createTrackbar("V_high", "annotated", 255, 255, lambda x: None)


    # Open as stream (no preloading)
    src = args.input
    try:
        cam_index = int(src)
        cap = cv2.VideoCapture(cam_index)
    except ValueError:
        cap = cv2.VideoCapture(src)

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open input: {args.input}")

    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0

    fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
    writer = cv2.VideoWriter(args.output, fourcc, fps, (width, height))

    # CSV logging
    csv_file = open(args.csv, "w", newline="")
    writer_csv = csv.writer(csv_file)
    writer_csv.writerow(["frame_index", "shape", "color_mask", "cx", "cy", "area"])

    frame_idx = 0
    t0 = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        annotated, dets = detect_on_frame(frame)
        writer.write(annotated)

        for shape, mask, cx, cy, area in dets:
            writer_csv.writerow([frame_idx, shape, mask, cx, cy, area])

        if args.preview:
            preview = cv2.resize(annotated, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow("annotated", preview)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
                break

        frame_idx += 1

    elapsed = time.time() - t0
    cap.release()
    writer.release()
    csv_file.close()
    if args.preview:
        cv2.destroyAllWindows()
    print(f"Processed {frame_idx} frames in {elapsed:.2f}s ({frame_idx/elapsed:.1f} FPS).")
    print(f"Saved video to: {args.output}")
    print(f"Saved CSV to: {args.csv}")

if __name__ == "__main__":
    main()