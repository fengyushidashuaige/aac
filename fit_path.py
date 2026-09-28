#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
水槽中心是【直线】-> 用鲁棒直线拟合把误检点剔掉, 输出干净路径 + 出图对比.

 ① 对重建点做鲁棒直线拟合(迭代剔除离群点)
 ② 输出: 只保留内点的直线段作为路径(比平滑干净得多)
 ③ 出 PNG: 飞行轨迹(蓝) + 原始重建点(红小点) + 拟合直线(绿) + 被剔除的误检(红圈)

用法: python2 fit_path.py <bag> <polyline.csv> <out.png> [out_path.csv] [阈值mm=60]
"""
import sys
import numpy as np
import rosbag
import cv2

BAG = sys.argv[1]
PL = sys.argv[2]
OUT = sys.argv[3] if len(sys.argv) > 3 else '/tmp/fit_path.png'
OUTCSV = sys.argv[4] if len(sys.argv) > 4 else ''
THR = float(sys.argv[5]) if len(sys.argv) > 5 else 60.0     # 离群阈值 mm

# --- 飞行轨迹 ---
bag = rosbag.Bag(BAG)
TP = []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position
    TP.append((p.x, p.y))
bag.close()
TP = np.array(TP)

# --- 重建的槽点 ---
W = np.loadtxt(PL, delimiter=',', skiprows=1)
if W.ndim == 1:
    W = W.reshape(1, -1)
P = W[:, :2]
print('轨迹 %d 点   重建槽点 %d 个' % (len(TP), len(P)))


def fit_line(Q):
    c = Q.mean(axis=0)
    u, s, vt = np.linalg.svd(Q - c, full_matrices=False)
    return c, vt[0]


def dist(Q, c, d):
    return np.abs((Q - c).dot(np.array([-d[1], d[0]])))


# --- 鲁棒拟合: 迭代剔除 ---
keep = np.ones(len(P), dtype=bool)
c, d = fit_line(P)
for it in range(8):
    dd = dist(P, c, d)
    thr = max(THR / 1000.0, 3.0 * np.median(dd[keep]) if keep.sum() > 10 else THR / 1000.0)
    nk = dd <= thr
    if nk.sum() < 10 or (nk == keep).all():
        break
    keep = nk
    c, d = fit_line(P[keep])
dd = dist(P, c, d)
print()
print('鲁棒拟合: 迭代 %d 次   阈值 %.1f mm' % (it + 1, 1000 * thr))
print('  内点 %d / %d  (剔除 %d 个误检点, %.0f%%)' % (
    keep.sum(), len(P), (~keep).sum(), 100.0 * (~keep).mean()))
print('  直线方向 %.2f deg' % np.degrees(np.arctan2(d[1], d[0])))
print('  内点残差: 中位 %.1f mm  P90 %.1f  max %.1f' % (
    1000 * np.median(dd[keep]), 1000 * np.percentile(dd[keep], 90), 1000 * dd[keep].max()))
if (~keep).any():
    print('  剔除点残差: 中位 %.1f mm  max %.1f' % (
        1000 * np.median(dd[~keep]), 1000 * dd[~keep].max()))

# --- 输出干净路径: 内点在直线上的投影, 按沿轴排序 ---
t = (P[keep] - c).dot(d)
order = np.argsort(t)
t = t[order]
q = c[None, :] + np.outer(t, d)
# 裁掉两端 1% (端点常是起飞/收尾)
k = max(1, int(len(q) * 0.01))
q = q[k:-k] if len(q) > 20 else q
# 重采样成等间距
L = np.linalg.norm(np.diff(q, axis=0), axis=1).sum()
n = max(2, int(L / 0.05))
ti = np.linspace(0, L, n)
tq = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
q2 = np.column_stack([np.interp(ti, tq, q[:, 0]), np.interp(ti, tq, q[:, 1])])
print()
print('干净路径: %d 点, 长 %.2f m  (原 %.2f m)' % (
    len(q2), L, np.linalg.norm(np.diff(P, axis=0), axis=1).sum()))
if OUTCSV:
    Z = np.zeros((len(q2), 1))
    np.savetxt(OUTCSV, np.hstack([q2, Z]), delimiter=',', header='x,y,z', comments='', fmt='%.4f')
    print('  已存 %s' % OUTCSV)

# --- 出图 ---
allx = np.concatenate([TP[:, 0], P[:, 0], q2[:, 0]])
ally = np.concatenate([TP[:, 1], P[:, 1], q2[:, 1]])
x0, x1 = allx.min() - 0.4, allx.max() + 0.4
y0, y1 = ally.min() - 0.4, ally.max() + 0.4
Wp, Hp = 1200, 900
sc = min(Wp / (x1 - x0), Hp / (y1 - y0))
ox = int((Wp - (x1 - x0) * sc) / 2); oy = int((Hp - (y1 - y0) * sc) / 2)


def tp(x, y):
    return (int(ox + (x - x0) * sc), int(Hp - oy - (y - y0) * sc))


img = np.full((Hp, Wp, 3), 28, np.uint8)
for g in np.arange(np.ceil(x0), x1, 0.5):
    cv2.line(img, tp(g, y0), tp(g, y1), (52, 52, 52), 1)
for g in np.arange(np.ceil(y0), y1, 0.5):
    cv2.line(img, tp(x0, g), tp(x1, g), (52, 52, 52), 1)
# 轨迹
for i in range(1, len(TP)):
    cv2.line(img, tp(TP[i - 1][0], TP[i - 1][1]), tp(TP[i][0], TP[i][1]), (200, 140, 60), 2)
# 原始重建点: 内点=灰, 离群=红圈
for i in range(len(P)):
    pt = tp(P[i, 0], P[i, 1])
    if keep[i]:
        cv2.circle(img, pt, 1, (150, 150, 150), -1)
    else:
        cv2.circle(img, pt, 6, (60, 60, 240), 2)
# 拟合直线(绿)
tt = np.linspace(t.min() - 0.3, t.max() + 0.3, 50)
for i in range(1, len(tt)):
    a = c + d * tt[i - 1]; b = c + d * tt[i]
    cv2.line(img, tp(a[0], a[1]), tp(b[0], b[1]), (80, 220, 80), 2)
cv2.putText(img, 'trajectory (LiDAR)', (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 140, 60), 2)
cv2.putText(img, 'raw gutter points: %d' % len(P), (14, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)
cv2.putText(img, 'rejected outliers: %d' % (~keep).sum(), (14, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 240), 2)
cv2.putText(img, 'robust straight line fit', (14, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 80), 2)
cv2.putText(img, 'grid = 0.5 m', (14, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 2)
cv2.imwrite(OUT, img)
print('  图已存 %s' % OUT)
