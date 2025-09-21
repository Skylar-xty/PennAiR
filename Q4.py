#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Static shape detection on a grassy background (minimal) + Part 4 (3D centers):
- Detect colored shapes (red, yellow, blue, magenta, bright green)
- Outline contours, mark centers, label shape
- NEW: Estimate plane depth Z from a known-size circle (radius=10 in), then back-project
       each center (u,v) -> (X,Y,Z) in camera frame with given intrinsics K.
- If no circle is found, Z is reported as unknown (still outputs 2D annotations).

Usage:
python Q4.py --input PennAir_2024_App_Static.png --output annotatedQ4.png
"""
import cv2
import numpy as np
import argparse
from pathlib import Path
import math

# ---------- Camera intrinsics (given) ----------
FX = 2564.3186869
FY = 2569.70273111
CX = 0.0
CY = 0.0
F_MEAN = 0.5 * (FX + FY)

# Real circle size (inches)
CIRCLE_RADIUS_IN = 10.0
CIRCLE_DIAM_IN = 2.0 * CIRCLE_RADIUS_IN

def classify_shape(contour):
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return "unknown"
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    v = len(approx)
    area = cv2.contourArea(contour)
    if area < 200:  # ignore tiny blobs
        return None
    circ = 4 * np.pi * area / (peri * peri)   # circularity
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

# --- small helpers for Part 4 ---
def depth_from_circle_pixels(r_px: float):
    """Z (in) = f_mean * D_real / d_px, where d_px = 2*r_px (circle diameter in pixels)."""
    if r_px is None or r_px <= 0:
        return None
    d_px = 2.0 * r_px
    return (F_MEAN * CIRCLE_DIAM_IN) / d_px

def pixel_to_camera_xyz(u: float, v: float, Z_in: float):
    """(u,v,1)*Z with intrinsics: X=(u-cx)/fx*Z, Y=(v-cy)/fy*Z, Z=Z. Units: inches."""
    X = (u - CX) / FX * Z_in
    Y = (v - CY) / FY * Z_in
    return (X, Y, Z_in)

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

    # -------- First pass: collect detections (no drawing yet) --------
    detections = []          
    circle_candidates = []    # store (contour, r_px) for circles (for Z estimation)

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

            det = {
                "shape": shape,
                "mask": label,
                "center": (cx, cy),
                "area": int(area),
                "contour": c
            }
            detections.append(det)

            # 圆的像素半径（用于 Z）：取最小外接圆
            if shape == "circle":
                (_, _), r = cv2.minEnclosingCircle(c)
                circle_candidates.append((c, r))

    # -------- Estimate plane depth Z from the largest circle --------
    Z_plane = None
    if circle_candidates:
        # pick the circle with largest radius (more stable)
        _, r_best = max(circle_candidates, key=lambda t: t[1])
        Z_plane = depth_from_circle_pixels(r_best)

    # -------- Second pass: draw + 3D text --------
    annot = img_bgr.copy()

    # Put Z note on the image
    z_note = "Z: unknown (no circle)" if Z_plane is None else f"Z = {Z_plane:.2f} in (from circle)"
    cv2.putText(annot, z_note, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(annot, z_note, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)

    for d in detections:
        c = d["contour"]
        cx, cy = d["center"]
        shape = d["shape"]

        cv2.drawContours(annot, [c], -1, (255,255,255), 3)     # outline
        cv2.circle(annot, (cx, cy), 6, (0,0,255), -1)          # center dot

        if Z_plane is not None:
            X, Y, Z = pixel_to_camera_xyz(cx, cy, Z_plane)
            text = f"{shape}  ({X:.1f},{Y:.1f},{Z:.1f}) in"
        else:
            text = f"{shape}"

        cv2.putText(annot, text, (cx-130, cy-18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(annot, text, (cx-130, cy-18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

    det_sorted = sorted(detections, key=lambda d: d["center"][0])
    for d in det_sorted:
        print(f"{d['shape']:>11s} @ {d['center']} (area={d['area']}) via {d['mask']}")

    return annot, detections, Z_plane

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="path to input image")
    ap.add_argument("--output", default="annotated.png", help="path to save annotated image")
    args = ap.parse_args()

    img_path = Path(args.input)
    img = cv2.imread(str(img_path))
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {img_path}")

    annot, det, Z_plane = detect_and_annotate(img)
    cv2.imwrite(args.output, annot)

    print("Estimated plane depth Z:", "unknown" if Z_plane is None else f"{Z_plane:.3f} in")

if __name__ == "__main__":
    main()
