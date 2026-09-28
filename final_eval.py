#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""用全局拟合残差当目标, 扫时移 delta + 门控参数, 给出最终推荐配置."""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
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

THETAS = np.radians(np.arange(0, 180, 0.5))


def run(conf_min, band, delta, verbose=False):
    base = [r for r in rows if r[4] == 1 and r[3] >= conf_min and T[0] <= r[0] <= T[-1]]
    if not base:
        return None
    med = np.median([r[5] for r in base])
    base = [r for r in base if abs(r[5] - med) <= band * med and abs(r[1]) <= 0.7 * 0.45]
    if len(base) < 40:
        return None
    Rl, Pl, off = [], [], []
    for r in base:
        ts = r[0] + delta
        i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
        a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
        R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
        u, s, vt = np.linalg.svd(R)
        Rl.append(u.dot(vt)); Pl.append(P[i - 1] + a * (P[i] - P[i - 1])); off.append(r[1])
    Rl = np.array(Rl); Pl = np.array(Pl); off = np.array(off)
    Ra = np.einsum('kij,j->ki', Rl, np.array([0.0, 0.0, -CAM_DOWN]))
    Rb = np.einsum('kij,j->ki', Rl, np.array([0.0, -1.0, 0.0]))
    best = None
    for th in THETAS:
        n2 = np.array([-np.sin(th), np.cos(th)])
        A = off * (Rb[:, :2].dot(n2))
        D = Pl[:, :2].dot(n2) + Ra[:, :2].dot(n2)
        M = np.column_stack([A, np.ones(len(A))])
        sol, _, _, _ = np.linalg.lstsq(M, -D)
        res = (M.dot(sol) + D) * 1000.0
        e = np.median(np.abs(res))
        if best is None or e < best[0]:
            best = (e, np.degrees(th), sol[0], np.percentile(np.abs(res), 90),
                    np.abs(res).max(), len(base), res.std())
    return best


print('== 时移 delta 扫描 (conf>=0.9, wpx±15%) ==')
print('   %7s %8s %8s %8s %8s %6s' % ('delta', 'k', '中位残差', 'P90', 'max', 'n'))
bestd = None
for d in np.arange(-0.60, 0.115, 0.04):
    r = run(0.9, 0.15, d)
    if r is None:
        continue
    print('   %+7.2f %8.3f %8.1f %8.1f %8.1f %6d' % (d, r[2], r[0], r[3], r[4], r[5]))
    if bestd is None or r[0] < bestd[0]:
        bestd = (r[0], d)
print('   >>> 残差最小 delta = %+.2f s (中位 %.1f mm)' % (bestd[1], bestd[0]))

print()
print('== 细扫 delta (0.01 步长) ==')
for d in np.arange(bestd[1] - 0.10, bestd[1] + 0.105, 0.01):
    r = run(0.9, 0.15, d)
    if r is None:
        continue
    mark = '  <<<' if abs(d - bestd[1]) < 1e-9 else ''
    print('   %+7.2f %8.3f %8.1f %8.1f %8.1f %6d%s' % (d, r[2], r[0], r[3], r[4], r[5], mark))

D = bestd[1]
print()
print('== 在 delta=%+.2f 下扫门控 ==' % D)
print('   %6s %6s %8s %8s %8s %8s %6s' % ('conf', 'band', 'k', '中位', 'P90', 'max', 'n'))
for cm in (0.3, 0.5, 0.7, 0.9, 0.95):
    for band in (0.06, 0.10, 0.15, 0.25, 1.0):
        r = run(cm, band, D)
        if r is None:
            print('   %6.2f %6.2f   点数不足' % (cm, band))
            continue
        print('   %6.2f %6.2f %8.3f %8.1f %8.1f %8.1f %6d' % (cm, band, r[2], r[0], r[3], r[4], r[5]))

print()
print('== 对比: 完全不做时移 (delta=0) 的门控表 ==')
for cm in (0.5, 0.9):
    for band in (0.10, 0.15, 0.25):
        r = run(cm, band, 0.0)
        if r is None:
            continue
        print('   conf>=%.2f band %.2f: k %.3f 中位 %6.1f P90 %6.1f mm n=%d' % (
            cm, band, r[2], r[0], r[3], r[5]))
