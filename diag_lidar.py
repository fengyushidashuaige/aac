#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
最终检验: 在"wpx 正常"的好帧上, 用雷达局部位移标定 offset.
  c1 ≈ +1.0  -> 雷达轨迹与检测一致 (雷达可靠)
  c1 明显≠1  -> 二者尺度/方向不一致, 要去查是谁错
同时给出: 若把 c1 当作标定系数, 建图残差是多少.
"""
import sys
import numpy as np
import rosbag

BAG = sys.argv[1]
CSVF = sys.argv[2]
WPX_LO, WPX_HI = 0.85, 1.15     # wpx 相对中位的允许带宽
CONF = 0.4


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


bag = rosbag.Bag(BAG)
T, P, Q = [], [], []
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
bag.close()
T = np.array(T); P = np.array(P); Rall = np.array([quat_to_R(q) for q in Q])

raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
G = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        r = (float(p[0]), float(p[1]) / 1000.0, float(p[2]), float(p[3]), int(p[4]), float(p[6]))
    except Exception:
        continue
    if r[4] == 1 and r[3] >= CONF and T[0] <= r[0] <= T[-1]:
        G.append(r)

med_wpx = np.median([r[5] for r in G])
print('good 帧 %d, wpx 中位 %.1f  -> 带宽 [%.1f, %.1f]' % (
    len(G), med_wpx, med_wpx * WPX_LO, med_wpx * WPX_HI))

for tag, sel in (('全部 good 帧', lambda r: True),
                 ('wpx 正常', lambda r: med_wpx * WPX_LO <= r[5] <= med_wpx * WPX_HI),
                 ('wpx 正常 & conf>=0.9', lambda r: med_wpx * WPX_LO <= r[5] <= med_wpx * WPX_HI
                  and r[3] >= 0.9)):
    g = [r for r in G if sel(r)]
    X, Y = [], []
    for k in range(len(g) - 1):
        t1, o1 = g[k][0], g[k][1]
        for j in range(k + 1, min(k + 6, len(g))):
            t2, o2 = g[j][0], g[j][1]
            dt = t2 - t1
            if dt <= 1e-3 or dt > 0.4:
                continue
            i1 = int(np.clip(np.searchsorted(T, t1), 1, len(T) - 1))
            i2 = int(np.clip(np.searchsorted(T, t2), 1, len(T) - 1))
            dpb = Rall[i1].T.dot(P[i2] - P[i1])
            dpsi = np.arctan2(Rall[i2][1, 0], Rall[i2][0, 0]) - np.arctan2(Rall[i1][1, 0], Rall[i1][0, 0])
            dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
            X.append([dpb[1], dpb[0], dpsi, 1.0])
            Y.append(o2 - o1)
            break
    if len(Y) < 20:
        print('%-24s 样本不足' % tag)
        continue
    X = np.array(X); Y = np.array(Y)
    sol, _, _, _ = np.linalg.lstsq(X, Y)
    pred = X.dot(sol)
    r2 = 1 - np.sum((Y - pred) ** 2) / max(np.sum((Y - Y.mean()) ** 2), 1e-12)
    se = np.sqrt(np.sum((Y - pred) ** 2) / max(len(Y) - 4, 1)
                 / max(np.sum((X[:, 0] - X[:, 0].mean()) ** 2), 1e-9))
    print('%-24s n=%4d  c1=%+.3f±%.3f  c2(前)=%+.3f  c3(偏航)=%+.3f  R2=%.3f' % (
        tag, len(Y), sol[0], se, sol[1], sol[2], r2))

# 用 c1 反标定后建图
print()
print('== 用 c1 系数重标定 offset 后的建图残差 ==')
Ri, Pi, of = [], [], []
for r in G:
    i = int(np.clip(np.searchsorted(T, r[0]), 1, len(T) - 1))
    al = (r[0] - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + al * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Ri.append(u.dot(vt)); Pi.append(P[i - 1] + al * (P[i] - P[i - 1])); of.append(r[1])
Ri = np.array(Ri); Pi = np.array(Pi); of = np.array(of)
wpx = np.array([r[5] for r in G])
good_m = (wpx >= med_wpx * WPX_LO) & (wpx <= med_wpx * WPX_HI)

for scale in (1.0, 0.74, 1.35):
    for gm, gname in ((np.ones(len(of), bool), '全部'), (good_m, 'wpx正常')):
        m = gm
        if m.sum() < 30:
            continue
        offm = of[m] * scale
        W = Pi[m] + np.einsum('kij,j->ki', Ri[m], np.array([0.0, 0.0, -0.55])) \
            + np.einsum('kij,kj->ki', Ri[m], np.outer(-offm, np.array([0.0, 1.0, 0.0])))
        c = W.mean(axis=0)
        u, s, vt = np.linalg.svd(W - c, full_matrices=False)
        d = vt[0]
        perp = (W - c) - np.outer((W - c).dot(d), d)
        pn = np.linalg.norm(perp, axis=1)
        print('   scale %.2f  %-8s %4d点  中位 %6.1f  P90 %6.1f mm' % (
            scale, gname, m.sum(), 1000 * np.median(pn), 1000 * np.percentile(pn, 90)))
