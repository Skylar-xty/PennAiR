#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Static shape detection on a grassy background:
- Detect colored shapes (red, yellow, blue, magenta, bright green)
- Outline contours
- Mark centers
- Label each shape

Usage:
    python shape_detect_static.py --input PennAir_2024_App_Static.png --output annotated.png
"""
import cv2
import numpy as np
import argparse
from pathlib import Path

def classify_shape(contour):
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return "unknown"
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    v = len(approx)
    area = cv2.contourArea(contour)
    if area < 200:  # ignore tiny blobs
        return None
    circ = 4 * np.pi * area / (peri * peri)   # calculate circularity
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

def detect_and_annotate(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Color ranges (OpenCV HSV: H in [0,179])
    color_ranges = {
        "red1": ((0, 120, 70), (10, 255, 255)),
        "red2": ((170, 120, 70), (179, 255, 255)),
        "yellow": ((20, 120, 120), (35, 255, 255)),
        "blue": ((100, 120, 70), (130, 255, 255)),
        "magenta": ((140, 120, 70), (170, 255, 255)),
        # bright green; increase V to avoid grass
        "bright_green": ((45, 120, 170), (85, 255, 255)),
    }

    masks = []
    for name, (low, high) in color_ranges.items():
        low = np.array(low, dtype=np.uint8)
        high = np.array(high, dtype=np.uint8)
        mask = cv2.inRange(hsv, low, high)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        masks.append((name, mask))

    # Merge red mask
    red_mask = None
    for name, m in masks:
        if name.startswith("red"):
            red_mask = m if red_mask is None else cv2.bitwise_or(red_mask, m)

    final_masks = []
    if red_mask is not None:
        final_masks.append(("red", red_mask))
    for name, m in masks:
        if not name.startswith("red"):
            final_masks.append((name, m))

    annot = img_bgr.copy()
    detections = []

    for label, mask in final_masks:
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

            cv2.drawContours(annot, [c], -1, (255,255,255), 3)     # outline
            cv2.circle(annot, (cx, cy), 6, (0,0,255), -1)          # center dot
            text = f"{shape}"
            cv2.putText(annot, text, (cx+8, cy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(annot, text, (cx+8, cy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            detections.append({"shape": shape, "mask": label, "center": (cx, cy), "area": int(area)})
    return annot, detections

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="path to input image")
    ap.add_argument("--output", default="annotated.png", help="path to save annotated image")
    args = ap.parse_args()

    img_path = Path(args.input)
    img = cv2.imread(str(img_path))
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {img_path}")

    annot, det = detect_and_annotate(img)
    cv2.imwrite(args.output, annot)

    det_sorted = sorted(det, key=lambda d: d["center"][0])
    for d in det_sorted:
        print(f"{d['shape']:>11s} @ {d['center']} (area={d['area']}) via {d['mask']}")

if __name__ == "__main__":
    main()