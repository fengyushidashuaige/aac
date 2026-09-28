#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""在完全相同的帧集合上, 用不同实现算 c1, 找出差异来源."""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]


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

rows = []
raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[2]), float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass

# 固定的帧集合: detected, conf>=0.9, wpx 在 120.2±15%
BASE = [r for r in rows if r[4] == 1 and r[3] >= 0.9 and T[0] <= r[0] <= T[-1]]
med = np.median([r[5] for r in BASE])
G = [r for r in BASE if abs(r[5] - med) <= 0.15 * med]
print('BASE(conf>=0.9) %d 帧, wpx 中位 %.1f, 再取 wpx 带内 -> %d 帧' % (len(BASE), med, len(G)))
t = np.array([r[0] for r in G]); off = np.array([r[1] for r in G])

# 两种位姿取法
idx_odom = [int(np.clip(np.searchsorted(T, x), 1, len(T) - 1)) for x in t]
Ri, Pi = [], []
for ti, i in zip(t, idx_odom):
    a = (ti - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Ri.append(u.dot(vt)); Pi.append(P[i - 1] + a * (P[i] - P[i - 1]))
Ri = np.array(Ri); Pi = np.array(Pi)


def fit(X, Y, tag):
    X = np.array(X); Y = np.array(Y)
    sol, _, _, _ = np.linalg.lstsq(X, Y)
    pred = X.dot(sol)
    r2 = 1 - np.sum((Y - pred) ** 2) / max(np.sum((Y - Y.mean()) ** 2), 1e-12)
    se = np.sqrt(np.sum((Y - pred) ** 2) / max(len(Y) - 4, 1)
                 / max(np.sum((X[:, 0] - X[:, 0].mean()) ** 2), 1e-9))
    print('  %-52s n=%4d  c1=%+.3f±%.3f  c3(偏航)=%+.3f  R2=%.3f  dY_std=%.1fmm' % (
        tag, len(Y), sol[0], se, sol[2], r2, 1000 * X[:, 0].std()))


def pairs(mode_pose, mode_pair, dtmax=0.4):
    X, Y = [], []
    if mode_pair == 'adj':
        ks = [(k, k + 1) for k in range(len(G) - 1)]
    else:
        ks = []
        for k in range(len(G) - 1):
            for j in range(k + 1, min(k + 6, len(G))):
                dt = t[j] - t[k]
                if dt <= 1e-3 or dt > dtmax:
                    continue
                ks.append((k, j))
                break
    for k, j in ks:
        if mode_pair == 'adj' and (t[j] - t[k] <= 1e-3 or t[j] - t[k] > dtmax):
            continue
        if mode_pair == 'adj' and (j - k) > 6:
            continue
        if mode_pose == 'interp':
            R1, P1, R2, P2 = Ri[k], Pi[k], Ri[j], Pi[j]
        else:
            a1, a2 = idx_odom[k], idx_odom[j]
            R1, P1, R2, P2 = Rall[a1], P[a1], Rall[a2], P[a2]
        dpb = R1.T.dot(P2 - P1)
        dpsi = np.arctan2(R2[1, 0], R2[0, 0]) - np.arctan2(R1[1, 0], R1[0, 0])
        dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
        X.append([dpb[1], dpb[0], dpsi, 1.0])
        Y.append(off[j] - off[k])
    return X, Y


print()
print('== 同一帧集合, 四种实现 ==')
fit(*pairs('interp', 'adj'), tag='A 插值位姿 + 相邻对 (fix_eval 用的)')
fit(*pairs('raw', 'adj'), tag='B 最近邻原始位姿 + 相邻对')
fit(*pairs('interp', 'first'), tag='C 插值位姿 + 首个 dt<=0.4 的 j (diag_lidar 用的)')
fit(*pairs('raw', 'first'), tag='D 最近邻原始位姿 + 首个 dt<=0.4 的 j')

print()
print('== 只留 dt 更短的 (排除长间隔带来的积分/漂移偏差) ==')
for dtm in (0.05, 0.1, 0.2, 0.4):
    fit(*pairs('interp', 'adj', dtm), tag='A 相邻对, dt<=%.2f' % dtm)
for dtm in (0.05, 0.1, 0.2, 0.4):
    fit(*pairs('raw', 'first', dtm), tag='D 首个j, dt<=%.2f' % dtm)

print()
print('== 加权: 只用位移足够大的对 (小位移时 offset 噪声占比高) ==')
X, Y = pairs('interp', 'adj')
X = np.array(X); Y = np.array(Y)
for thr in (0.0, 0.005, 0.01, 0.02, 0.05):
    m = np.abs(X[:, 0]) >= thr
    if m.sum() < 30:
        print('  |dy|>=%.3fm 样本不足 (%d)' % (thr, m.sum()))
        continue
    Xm, Ym = X[m], Y[m]
    sol, _, _, _ = np.linalg.lstsq(Xm, Ym)
    pred = Xm.dot(sol)
    r2 = 1 - np.sum((Ym - pred) ** 2) / max(np.sum((Ym - Ym.mean()) ** 2), 1e-12)
    print('  |dy|>=%.3fm: n=%4d  c1=%+.3f  R2=%.3f' % (thr, m.sum(), sol[0], r2))
