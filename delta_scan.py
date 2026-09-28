#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""扫描检测时间戳相对雷达位姿的时间偏移 delta, 看 c1/R2 怎么变.
若某个 delta 让 R2 显著变大 -> 说明在线链路里 offset 和位姿没对齐(时延/时钟),
这本身就是建图误差的一个根因 (偏移方向被转错).
"""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
CONF = 0.9
BAND = 0.15


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


bag = rosbag.Bag(BAG)
T, P, Q = [], [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
bag.close()
T = np.array(T); P = np.array(P); Rall = np.array([quat_to_R(q) for q in Q])
print('雷达 %.1f Hz (%.4f s 间隔)' % (len(T) / (T[-1] - T[0]), np.median(np.diff(T))))

# 也检查消息头时间 vs 录包时间
bag = rosbag.Bag(BAG)
n = 0
dt_hdr = []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    if hasattr(m, 'header'):
        dt_hdr.append(t.to_sec() - m.header.stamp.to_sec())
    n += 1
    if n > 400:
        break
bag.close()
if dt_hdr:
    dt_hdr = np.array(dt_hdr)
    print('录包时间 - 消息头时间: 中位 %.4f s  std %.4f  (大=录包缓冲延迟)' % (
        np.median(dt_hdr), dt_hdr.std()))

rows = []
raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
# rows 元素: (t, off_m, conf, det, wpx)
BASE = [r for r in rows if r[3] == 1 and r[2] >= CONF and T[0] <= r[0] <= T[-1]]
med = np.median([r[4] for r in BASE])
G = [r for r in BASE if abs(r[4] - med) <= BAND * med]
print('帧集合: conf>=%.1f 且 wpx 在 %.1f±%.0f%% -> %d 帧' % (CONF, med, BAND * 100, len(G)))
t = np.array([r[0] for r in G]); off = np.array([r[1] for r in G])


def pose_at(ts):
    ts = np.clip(ts, T[0], T[-1])
    i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
    a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    return u.dot(vt), P[i - 1] + a * (P[i] - P[i - 1])


def run(delta, dtmax=0.2):
    R1 = []; P1 = []
    for x in t:
        R, p = pose_at(x + delta)
        R1.append(R); P1.append(p)
    R1 = np.array(R1); P1 = np.array(P1)
    X, Y = [], []
    for k in range(len(t) - 1):
        dt = t[k + 1] - t[k]
        if dt <= 1e-3 or dt > dtmax:
            continue
        dpb = R1[k].T.dot(P1[k + 1] - P1[k])
        dpsi = np.arctan2(R1[k + 1][1, 0], R1[k + 1][0, 0]) - np.arctan2(R1[k][1, 0], R1[k][0, 0])
        dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
        X.append([dpb[1], dpb[0], dpsi, 1.0]); Y.append(off[k + 1] - off[k])
    if len(Y) < 50:
        return None
    X = np.array(X); Y = np.array(Y)
    sol, _, _, _ = np.linalg.lstsq(X, Y)
    pred = X.dot(sol)
    r2 = 1 - np.sum((Y - pred) ** 2) / max(np.sum((Y - Y.mean()) ** 2), 1e-12)
    se = np.sqrt(np.sum((Y - pred) ** 2) / max(len(Y) - 4, 1)
                 / max(np.sum((X[:, 0] - X[:, 0].mean()) ** 2), 1e-9))
    return sol[0], se, r2, len(Y), np.median(np.abs(Y - pred)) * 1000


print()
print('  %7s %8s %8s %7s %6s %10s' % ('delta(s)', 'c1', '±', 'R2', 'n', '残差中位mm'))
best = None
for d in np.arange(-0.30, 0.305, 0.02):
    r = run(d)
    if r is None:
        continue
    print('  %+7.2f %+8.3f %8.3f %7.3f %6d %10.1f' % (d, r[0], r[1], r[2], r[3], r[4]))
    if best is None or r[2] > best[2]:
        best = (d, r[0], r[2], r[1], r[4])
print()
print('  >>> R2 最高: delta = %+.2f s,  c1 = %+.3f±%.3f, R2 = %.3f, 残差中位 %.1f mm' % (
    best[0], best[1], best[3], best[2], best[4]))
print('  (delta>0 表示 offset 相对位姿滞后/位姿要往后取)')
