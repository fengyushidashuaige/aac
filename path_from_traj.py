#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
直接用【飞行轨迹】拟合直线出路径 —— 再用【相机平均偏差】把它平移回槽中心.

依据:
  - 你沿槽飞时, 轨迹的平均位置 就是 槽中心 的估计
  - 但手飞不可能压着槽中心飞: 实测各趟的相机 off 中位数在 -41 ~ +76 mm 之间
  - 关键: 这 40~80mm 是【系统偏差】, 相机每帧都测到了, 取中位数就能把它提出来
    (逐帧用相机修才会引入 100~350mm 噪声; 用整趟中位数修一个常量, 噪声被平均掉)

用法: python2 path_from_traj.py <bag> <out.csv> [out.png] [阈值mm=120]

相机修正:
  默认开启, 用环境变量关闭:  PFT_CAM=0 python2 path_from_traj.py ...
"""
import sys
import os
import glob
import numpy as np
import rosbag
import cv2

BAG = sys.argv[1]
OUTCSV = sys.argv[2]
OUTPNG = sys.argv[3] if len(sys.argv) > 3 else ''
THR = float(sys.argv[4]) if len(sys.argv) > 4 else 120.0
USE_CAM = os.environ.get('PFT_CAM', '1') not in ('0', 'false', 'False')

bag = rosbag.Bag(BAG)
T, P = [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position
    T.append(t.to_sec()); P.append((p.x, p.y, p.z))
bag.close()
T = np.array(T); P = np.array(P)
print('轨迹 %d 点  %.0fs  x %.2f~%.2f  y %.2f~%.2f  z %.2f~%.2f' % (
    len(P), T[-1] - T[0], P[:, 0].min(), P[:, 0].max(),
    P[:, 1].min(), P[:, 1].max(), P[:, 2].min(), P[:, 2].max()))


def fit_line(Q):
    c = Q.mean(axis=0)
    u, s, vt = np.linalg.svd(Q - c, full_matrices=False)
    return c, vt[0]


def dperp(Q, c, d):
    return np.abs((Q - c).dot(np.array([-d[1], d[0]])))


Q = P[:, :2]
keep = np.ones(len(Q), dtype=bool)
c, d = fit_line(Q)
for it in range(10):
    dd = dperp(Q, c, d)
    thr = max(THR / 1000.0, 3.0 * np.median(dd[keep]))
    nk = dd <= thr
    if nk.sum() < 20 or (nk == keep).all():
        break
    keep = nk
    c, d = fit_line(Q[keep])
dd = dperp(Q, c, d)
print()
print('鲁棒直线拟合: 迭代 %d  阈值 %.0fmm' % (it + 1, 1000 * thr))
print('  内点 %d/%d (剔除 %d 个 = %.0f%%: 起飞/降落/掉头那些)' % (
    keep.sum(), len(Q), (~keep).sum(), 100.0 * (~keep).mean()))
print('  直线方向 %.2f deg' % np.degrees(np.arctan2(d[1], d[0])))
print('  内点残差: 中位 %.1f mm  P90 %.1f  max %.1f' % (
    1000 * np.median(dd[keep]), 1000 * np.percentile(dd[keep], 90), 1000 * dd[keep].max()))

t = (Q[keep] - c).dot(d)
lo, hi = np.percentile(t, 1), np.percentile(t, 99)
L = hi - lo
n = max(2, int(L / 0.05))
ti = np.linspace(lo, hi, n)
q = c[None, :] + np.outer(ti, d)

# ─────────────────────────────────────────────────────────────
# 2026-09-24: 用相机的平均偏差把路径平移回槽中心
# ─────────────────────────────────────────────────────────────
bias = None
if USE_CAM:
    z = P[:, 2]
    air = z > 0.15
    if air.sum() >= 30:
        t0, t1 = float(T[air].min()), float(T[air].max())
        vals = []
        for sd in sorted(glob.glob(os.path.expanduser('~/logs/guttermap_*/'))):
            dp = os.path.join(sd, 'detect.csv')
            if not os.path.isfile(dp):
                continue
            try:
                with open(dp) as fh:
                    fh.readline()
                    for ln in fh:
                        qq = ln.split(',')
                        if len(qq) < 8:
                            continue
                        try:
                            ts = float(qq[0])
                            if ts < t0 or ts > t1:
                                continue
                            if int(float(qq[4])) != 1:
                                continue
                            vals.append(float(qq[1]))
                        except ValueError:
                            pass
            except IOError:
                pass
        if len(vals) >= 20:
            bias = float(np.median(vals))
print()
if bias is None:
    if not USE_CAM:
        print('相机修正: 已关闭(PFT_CAM=0) —— 路径未平移')
    else:
        print('相机修正: 不可用(该时间段 detect.csv 没有足够数据) —— 路径未平移')
else:
    # 相机 off 为正 = 槽在画面中心右侧 = 飞机偏左 = 路径应向右平移
    # 右方向 = 把前进方向 d 顺时针转 90 度
    right = np.array([d[1], -d[0]])
    shift = right * (bias / 1000.0)
    q = q + shift[None, :]
    print('相机修正: off 中位 %+.1f mm  ->  路径平移 %+.1f mm (沿垂直方向)' % (
        bias, bias))

print()
print('路径: %d 点, 长 %.2f m' % (len(q), L))
print('  起点 (%.2f, %.2f)   终点 (%.2f, %.2f)' % (
    q[0, 0], q[0, 1], q[-1, 0], q[-1, 1]))
Z = np.full((len(q), 1), float(np.median(P[:, 2])))
np.savetxt(OUTCSV, np.hstack([q, Z]), delimiter=',', header='x,y,z', comments='', fmt='%.4f')
print('  已存 %s' % OUTCSV)

if OUTPNG:
    x0, x1 = Q[:, 0].min() - 0.4, Q[:, 0].max() + 0.4
    y0, y1 = Q[:, 1].min() - 0.4, Q[:, 1].max() + 0.4
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
    for i in range(len(Q)):
        pt = tp(Q[i, 0], Q[i, 1])
        if keep[i]:
            cv2.circle(img, pt, 1, (150, 150, 150), -1)
        else:
            cv2.circle(img, pt, 5, (60, 60, 240), 2)
    if bias is not None:
        q0 = c[None, :] + np.outer(ti, d)
        for i in range(1, len(q0)):
            cv2.line(img, tp(q0[i - 1, 0], q0[i - 1, 1]),
                     tp(q0[i, 0], q0[i, 1]), (170, 170, 170), 2)   # 修正前(灰)
    for i in range(1, len(q)):
        cv2.line(img, tp(q[i - 1, 0], q[i - 1, 1]), tp(q[i, 0], q[i, 1]), (80, 220, 80), 3)
    cv2.putText(img, 'flight trajectory: gray=%d (rejected %d)' % (keep.sum(), (~keep).sum()),
                (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)
    cv2.putText(img, ('path = fit + camera shift %+.0fmm (green)' % bias)
                if bias is not None else 'path = robust fit (green)',
                (14, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 80), 2)
    if bias is not None:
        cv2.putText(img, 'grey line = fit before camera shift',
                    (14, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (170, 170, 170), 2)
    cv2.putText(img, 'grid = 0.5 m', (14, 98 if bias is not None else 74),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 2)
    cv2.imwrite(OUTPNG, img)
    print('  图已存 %s' % OUTPNG)
