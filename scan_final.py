#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""给定 (a_x, dt), 报 k/残差. 用法: scan_final.py <bag> <csv> <ax> <dt> [conf] [band]"""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
AX = float(sys.argv[3])
DT = float(sys.argv[4])
CONF = float(sys.argv[5]) if len(sys.argv) > 5 else 0.9
BAND = float(sys.argv[6]) if len(sys.argv) > 6 else 0.15


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
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
base = [r for r in rows if r[3] == 1 and r[2] >= CONF and T[0] <= r[0] <= T[-1]]
if len(base) < 40:
    print('帧太少'); sys.exit(0)
med = np.median([r[4] for r in base])
G = [r for r in base if abs(r[4] - med) <= BAND * med and abs(r[1]) <= 0.315]

Rl, Pl, off = [], [], []
for r in G:
    ts = np.clip(r[0] + DT, T[0], T[-1])
    i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
    a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Rl.append(u.dot(vt)); Pl.append(P[i - 1] + a * (P[i] - P[i - 1])); off.append(r[1])
Rl = np.array(Rl); Pl = np.array(Pl); off = np.array(off)
h = np.median(Pl[:, 2])
Rb = np.einsum('kij,j->ki', Rl, np.array([0.0, -1.0, 0.0]))
Ra = np.einsum('kij,j->ki', Rl, np.array([AX, 0.0, -h]))
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
        best = (e, sol[0], np.percentile(np.abs(res), 90), np.abs(res).max(), np.degrees(th))
print('ax=%.2f dt=%+.2f conf=%.2f band=%.2f | n=%4d  k=%.3f  theta=%.1f  中位 %5.1f  P90 %5.1f  max %6.1f  (h=%.2f)' % (
    AX, DT, CONF, BAND, len(G), best[1], best[4], best[0], best[2], best[3], h))
