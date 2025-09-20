#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quantile-adaptive, background-agnostic shape detection (no HSV tables, no CSV).

Per frame:
1) HSV.S、Lab-chroma、V/L 取 75~90分位自适应阈值 → 彩色/明亮前景
2) 自适应 Canny 边缘
3) (前景 ∧ 边缘) → 形态学清理 → 连通域
4) 连通域过滤：面积占比、实心“厚度”(distance transform)、solidity/extent/circularity
5) 轮廓多边形拟合分类并标中心

Usage:
  python shape_detect_video_quantile.py --input video.mp4 --output annotated.mp4 --preview 1
"""
import cv2, numpy as np, argparse, time, math

# -------- 自适应但可微调的少量参数（和分辨率无关） --------
Q_S = 0.75           # S/色度分位阈值，0.70~0.85
Q_V = 0.75           # V/L 分位阈值，0.70~0.85
EDGE_SIGMA = 0.40    # Canny 灵敏度 0.33~0.50（大=更敏感）

MIN_AREA_FRAC = 0.0012   # 连通域最小面积占整帧比例(≈0.12%)；小目标→降到 0.0006~0.001
MIN_EXTENT    = 0.52     # 面积/外接矩形面积
MIN_SOLIDITY  = 0.78
MIN_CIRC      = 0.66
MIN_THICKNESS = 2.5      # 组件最小“厚度”(由距离变换估计)像素；太碎的纹理会被刷掉

def classify_shape(c):
    peri = cv2.arcLength(c, True)
    if peri <= 0: return None
    approx = cv2.approxPolyDP(c, 0.02*peri, True)
    v = len(approx)
    area = cv2.contourArea(c)
    circ = 4.0*math.pi*area/(peri*peri)
    if v==3: return "triangle"
    if v==4: return "quadrilateral"
    if v==5: return "pentagon"
    return "circle" if circ>0.78 else "polygon"

def quantile_mask(frame):
    """用分位数阈值产生彩色/明亮前景（对不同背景自适应）。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    S, V = hsv[...,1].astype(np.float32), hsv[...,2].astype(np.float32)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    A = lab[...,1].astype(np.float32) - 128.0
    B = lab[...,2].astype(np.float32) - 128.0
    chroma = np.sqrt(A*A + B*B)

    # 分位阈值（逐帧自适应）
    s_thr = np.quantile(S, Q_S)
    v_thr = np.quantile(V, Q_V)
    c_thr = np.quantile(chroma, Q_S)

    s_bin = (S >= s_thr).astype(np.uint8)*255
    v_bin = (V >= v_thr).astype(np.uint8)*255
    c_bin = (chroma >= c_thr).astype(np.uint8)*255

    fg = cv2.bitwise_or(s_bin, c_bin)
    fg = cv2.bitwise_or(fg, v_bin)  # 保留白亮梯形
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,  np.ones((5,5),np.uint8), 1)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8), 1)
    return fg

def edge_mask(gray):
    sm = cv2.bilateralFilter(gray, 9, 75, 75)
    v  = np.median(sm)
    lo = int(max(0, (1.0-EDGE_SIGMA)*v))
    hi = int(min(255,(1.0+EDGE_SIGMA)*v))
    e  = cv2.Canny(sm, lo, hi, L2gradient=True)
    e  = cv2.dilate(e, np.ones((3,3),np.uint8), 1)
    e  = cv2.morphologyEx(e, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), 1)
    return e

def detect_on_frame(frame):
    H, W = frame.shape[:2]
    min_area = int(MIN_AREA_FRAC * H * W)

    # 1) 自适应前景 & 2) 边缘
    m_fg  = quantile_mask(frame)
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    m_edg = edge_mask(gray)

    # 3) 融合 + 清理
    mask = cv2.bitwise_and(m_fg, m_edg)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9,9),np.uint8), 1)

    # 拿到连通域（比 findContours 更方便做统计）
    num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    annot = frame.copy()

    for i in range(1, num):  # 0 是背景
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue

        # 组件子图和二值掩膜
        roi = mask[y:y+h, x:x+w]
        if roi.size == 0: 
            continue

        # 4a) “厚度”过滤：距离变换的最大值（像素）
        dist = cv2.distanceTransform((roi>0).astype(np.uint8), cv2.DIST_L2, 3)
        thickness = float(dist.max())
        if thickness < MIN_THICKNESS:
            continue

        # 4b) 从组件再取精确轮廓
        cnts,_ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: 
            continue
        c = max(cnts, key=cv2.contourArea)
        c[:,0,0] += x; c[:,0,1] += y  # 回到整帧坐标

        # 4c) 几何过滤
        area_c = cv2.contourArea(c)
        if area_c < min_area:
            continue
        hull = cv2.convexHull(c); hull_area = max(cv2.contourArea(hull),1.0)
        solidity = area_c / hull_area
        bx,by,bw,bh = cv2.boundingRect(c)
        extent = area_c / float(bw*bh)
        peri = max(cv2.arcLength(c, True), 1.0)
        circularity = 4.0*math.pi*area_c/(peri*peri)
        if solidity < MIN_SOLIDITY and circularity < MIN_CIRC:
            continue
        if extent < MIN_EXTENT:
            continue

        # 5) 分类 + 绘制
        label = classify_shape(c)
        if not label:
            continue
        M = cv2.moments(c)
        if M["m00"]!=0:
            cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
        else:
            (xx,yy),_ = cv2.minEnclosingCircle(c); cx,cy = int(xx),int(yy)

        cv2.drawContours(annot, [c], -1, (255,255,255), 3)
        cv2.circle(annot, (cx,cy), 6, (0,0,255), -1)
        cv2.putText(annot, label, (cx+8, cy-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(annot, label, (cx+8, cy-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

    return annot

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True)
    p.add_argument("--output", default="annotated.mp4")
    p.add_argument("--preview", type=int, default=0)
    p.add_argument("--fourcc", default="mp4v")
    a = p.parse_args()

    try: cap = cv2.VideoCapture(int(a.input))
    except: cap = cv2.VideoCapture(a.input)
    if not cap.isOpened(): raise RuntimeError(f"Failed to open input: {a.input}")

    w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps= cap.get(cv2.CAP_PROP_FPS) or 30.0
    writer = cv2.VideoWriter(a.output, cv2.VideoWriter_fourcc(*a.fourcc), fps, (w,h))
    if a.preview: cv2.namedWindow("annotated", cv2.WINDOW_NORMAL)

    n, t0 = 0, time.time()
    while True:
        ok, frame = cap.read()
        if not ok: break
        out = detect_on_frame(frame)
        writer.write(out)
        if a.preview:
            cv2.imshow("annotated", cv2.resize(out,(0,0),fx=0.5,fy=0.5))
            if cv2.waitKey(1) & 0xFF == 27: break
        n += 1

    cap.release(); writer.release()
    if a.preview: cv2.destroyAllWindows()
    dt = max(1e-6, time.time()-t0)
    print(f"Processed {n} frames in {dt:.2f}s ({n/dt:.1f} FPS). Saved to {a.output}")

if __name__ == "__main__":
    main()
