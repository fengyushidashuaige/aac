#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
关键判别: 0.74 到底是"检测器尺度错" 还是 "投影里缺了前向注视点 a_x"?

相机是前下方俯视 -> 观测点不在飞机正下方, 而在前下方.
a = (a_x, 0, -h), a_x = h*cot(俯角). 若补上 a_x 后 k -> 1.0,
则说明检测器的 mm 尺度是对的(110mm 没错), 错的是投影模型.

同时扫 a_x 和 h (两者都与俯角/高度耦合), 看 (残差, k) 曲面.
"""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
CONF, BAND = 0.9, 0.10
DT = float(sys.argv[3]) if len(sys.argv) > 3 else -0.20


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
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
base = [r for r in rows if r[3] == 1 and r[2] >= CONF and T[0] <= r[0] <= T[-1]]
med = np.median([r[4] for r in base])
G = [r for r in base if abs(r[4] - med) <= BAND * med]
if len(G) < 40:
    sys.exit('帧太少 %d' % len(G))

Rl, Pl, off = [], [], []
for r in G:
    ts = np.clip(r[0] + DT, T[0], T[-1])
    i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
    a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Rl.append(u.dot(vt)); Pl.append(P[i - 1] + a * (P[i] - P[i - 1])); off.append(r[1])
Rl = np.array(Rl); Pl = np.array(Pl); off = np.array(off)
Rb = np.einsum('kij,j->ki', Rl, np.array([0.0, -1.0, 0.0]))
TH = np.radians(np.arange(0, 180, 0.5))


def fitk(a_vec):
    Ra = np.einsum('kij,j->ki', Rl, np.array(a_vec))
    best = None
    for th in TH:
        n2 = np.array([-np.sin(th), np.cos(th)])
        A = off * (Rb[:, :2].dot(n2))
        D = Pl[:, :2].dot(n2) + Ra[:, :2].dot(n2)
        M = np.column_stack([A, np.ones(len(A))])
        sol, _, _, _ = np.linalg.lstsq(M, -D)
        res = (M.dot(sol) + D) * 1000.0
        e = np.median(np.abs(res))
        if best is None or e < best[0]:
            best = (e, sol[0], np.percentile(np.abs(res), 90), np.degrees(th))
    return best


print('h=0.44 固定, 扫 a_x  (n=%d)' % len(G))
print('   %6s %8s %8s %8s %7s' % ('a_x', 'k', '中位残差', 'P90', 'theta'))
for ax in np.arange(0.0, 1.61, 0.1):
    e, k, p90, th = fitk((ax, 0.0, -0.44))
    c1 = '   <== k~1' if abs(k - 1.0) < 0.03 else ''
    print('   %6.2f %8.3f %8.1f %8.1f %7.1f%s' % (ax, k, e, p90, th, c1))

print()
print('二维扫 (a_x, h): 每格 "k / 中位残差"')
hs = np.arange(0.30, 0.71, 0.05)
print('   %6s' % 'a_x\\h' + ''.join('%13.2f' % h for h in hs))
for ax in np.arange(0.0, 1.21, 0.2):
    line = '   %6.2f' % ax
    for h in hs:
        e, k, p90, th = fitk((ax, 0.0, -h))
        line += '  %5.2f/%5.1f' % (k, e)
    print(line)

print()
print('找 k 最接近 1.0 且残差可接受的组合:')
best = None
for ax in np.arange(0.0, 1.61, 0.05):
    for h in np.arange(0.25, 0.81, 0.05):
        e, k, p90, th = fitk((ax, 0.0, -h))
        score = abs(k - 1.0) + e / 1000.0
        if best is None or score < best[0]:
            best = (score, ax, h, k, e, p90)
print('   a_x=%.2f h=%.2f -> k=%.3f  中位残差 %.1f  P90 %.1f' % (
    best[1], best[2], best[3], best[4], best[5]))
print('   (物理约束: a_x = h*cot(俯角); h≈飞行高度0.44 时 a_x 就是 h*cot(alpha))')
