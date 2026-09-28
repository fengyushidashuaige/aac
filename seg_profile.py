#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""把残差按时间分段打出来, 定位是飞行中的哪一段坏."""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
CONF, BAND, DT = 0.9, 0.15, float(sys.argv[3]) if len(sys.argv) > 3 else -0.08
CAM_DOWN = 0.55


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

raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
rows = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[2]), float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
base = [r for r in rows if r[4] == 1 and r[3] >= CONF and T[0] <= r[0] <= T[-1]]
med = np.median([r[5] for r in base])
G = [r for r in base if abs(r[5] - med) <= BAND * med and abs(r[1]) <= 0.315]
print('base %d -> 门控后 %d' % (len(base), len(G)))

Rl, Pl, off, tt, zz, ww = [], [], [], [], [], []
for r in G:
    ts = np.clip(r[0] + DT, T[0], T[-1])
    i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
    a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Rl.append(u.dot(vt)); Pl.append(P[i - 1] + a * (P[i] - P[i - 1]))
    off.append(r[1]); tt.append(r[0] - T[0]); zz.append(Pl[-1][2]); ww.append(r[5])
Rl = np.array(Rl); Pl = np.array(Pl); off = np.array(off)
tt = np.array(tt); zz = np.array(zz); ww = np.array(ww)

# 用全局拟合拿最优直线的方向 theta, 然后算每点的垂直残差
Rb = np.einsum('kij,j->ki', Rl, np.array([0.0, -1.0, 0.0]))
Ra = np.einsum('kij,j->ki', Rl, np.array([0.0, 0.0, -CAM_DOWN]))
best = None
for th in np.radians(np.arange(0, 180, 0.5)):
    n2 = np.array([-np.sin(th), np.cos(th)])
    A = off * (Rb[:, :2].dot(n2))
    D = Pl[:, :2].dot(n2) + Ra[:, :2].dot(n2)
    M = np.column_stack([A, np.ones(len(A))])
    sol, _, _, _ = np.linalg.lstsq(M, -D)
    res = (M.dot(sol) + D) * 1000.0
    e = np.median(np.abs(res))
    if best is None or e < best[0]:
        best = (e, res, np.degrees(th), sol[0])
print('最优: theta %.1f  k %.3f  中位残差 %.1f mm' % (best[2], best[3], best[0]))
res = best[1]

print()
print('按时间分 12 段:')
print('  %7s %7s %7s %7s %7s %7s %7s' % ('t起', 'n', '残差中位', 'P90', '高度z', 'off中位', 'wpx中位'))
nseg = 12
for i in range(nseg):
    lo = tt.min() + (tt.max() - tt.min()) * i / nseg
    hi = tt.min() + (tt.max() - tt.min()) * (i + 1) / nseg
    m = (tt >= lo) & (tt < hi)
    if m.sum() < 5:
        continue
    print('  %7.1f %7d %7.1f %7.1f %7.3f %7.0f %7.1f' % (
        lo, m.sum(), np.median(np.abs(res[m])), np.percentile(np.abs(res[m]), 90),
        np.median(zz[m]), 1000 * np.median(off[m]), np.median(ww[m])))

print()
print('残差最大的 20 个时刻:')
o = np.argsort(-np.abs(res))[:20]
print('  %7s %8s %8s %8s %8s' % ('t', '残差mm', 'off_mm', 'wpx', '高度'))
for i in o:
    print('  %7.1f %8.1f %8.0f %8.1f %8.3f' % (tt[i], res[i], 1000 * off[i], ww[i], zz[i]))
