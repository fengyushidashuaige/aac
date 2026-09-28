#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Does the fitted scale k depend on how much of the trough is visible (wpx)?

model: W_i = p_i + R_i*a_i + k*off_i*(R_i*b) must lie on a horizontal line.
   a_i = (AX, 0, -h_i)   body frame, h_i = per-frame altitude
   b   = (0, -1, 0)      body left; +off means trough is to the right
If k == 1.0 the reported off_mm is metrically right.
Fit k separately per wpx bucket.
"""
import sys
import numpy as np
import rosbag

BAG, CSVF = sys.argv[1], sys.argv[2]
AX = float(sys.argv[3]) if len(sys.argv) > 3 else 0.45


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
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[2]),
                     float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
# (t, off_m, ang, conf, det, wpx)

base = [r for r in rows if r[4] == 1 and T[0] <= r[0] <= T[-1]]
print('detected rows in bag window: %d' % len(base))
if len(base) < 40:
    sys.exit('too few')

w_all = np.array([r[5] for r in base])
print('wpx: min %.1f  P10 %.1f  med %.1f  P90 %.1f  max %.1f' % (
    w_all.min(), np.percentile(w_all, 10), np.median(w_all),
    np.percentile(w_all, 90), w_all.max()))


def build(sel):
    """rows -> list of (R, p, off_m, wpx)"""
    out = []
    for r in base:
        if not sel(r):
            continue
        ts = r[0]
        i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
        a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
        R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
        u, s, vt = np.linalg.svd(R)
        out.append((u.dot(vt), P[i - 1] + a * (P[i] - P[i - 1]), r[1], r[5]))
    return out


def fit(data, ax=AX):
    Rl = np.array([d[0] for d in data])
    Pl = np.array([d[1] for d in data])
    off = np.array([d[2] for d in data])
    h = np.clip(Pl[:, 2], 0.15, 1.5)
    a_vec = np.column_stack([np.full(len(data), ax), np.zeros(len(data)), -h])
    Ra = np.einsum('kij,kj->ki', Rl, a_vec)
    b = np.array([0.0, -1.0, 0.0])
    Rb = np.einsum('kij,j->ki', Rl, b)
    best = None
    for th in np.radians(np.arange(0, 180, 0.25)):
        n2 = np.array([-np.sin(th), np.cos(th)])
        A = off * (Rb[:, :2].dot(n2))
        D = Pl[:, :2].dot(n2) + Ra[:, :2].dot(n2)
        M = np.column_stack([A, np.ones(len(A))])
        sol, _, _, _ = np.linalg.lstsq(M, -D)
        res = (M.dot(sol) + D) * 1000.0
        e = np.median(np.abs(res))
        if best is None or e < best[0]:
            best = (e, np.degrees(th), sol[0], np.percentile(np.abs(res), 90),
                    np.abs(res).max(), 1.96 * res.std() / np.sqrt(len(res)))
    return best


def report(tag, data, sel=None):
    if len(data) < 40:
        print('%-26s n=%d  (too few)' % (tag, len(data)))
        return
    e, th, k, p90, mx, se = fit(data)
    w = np.array([d[3] for d in data])
    o = np.abs(np.array([d[2] for d in data])) * 1000.0
    e0, th0, k0, p900, _, _ = fit(data, ax=0.0)
    print('%-26s n=%4d  wpx med %5.1f  |off| med %5.1f  theta %5.1f  k %6.3f +-%.3f  '
          'resid med %5.1f P90 %6.1f   [k@ax=0 %6.3f]' % (
              tag, len(data), np.median(w), np.median(o), th, k, se, e, p90, k0))


print()
print('== overall ==')
report('all detected', build(lambda r: True))

print()
print('== by wpx quartile (small wpx = little trough visible) ==')
qs = [(0, 25), (25, 50), (50, 75), (75, 100)]
lo, hi = np.percentile(w_all, 10), np.percentile(w_all, 90)
for a, b_ in qs:
    v0 = np.percentile(w_all, a)
    v1 = np.percentile(w_all, b_)
    tag = 'wpx P%d-P%d  [%.0f,%.0f]' % (a, b_, v0, v1)
    report(tag, build(lambda r, v0=v0, v1=v1: v0 <= r[5] <= v1))

print()
print('== extremes ==')
report('wpx bottom 20%', build(lambda r: r[5] <= np.percentile(w_all, 20)))
report('wpx top 20%', build(lambda r: r[5] >= np.percentile(w_all, 80)))
report('wpx bottom 10%', build(lambda r: r[5] <= np.percentile(w_all, 10)))
report('wpx top 10%', build(lambda r: r[5] >= np.percentile(w_all, 90)))
report('wpx within +-10% of median', build(lambda r: abs(r[5] - np.median(w_all)) <= 0.10 * np.median(w_all)))

print()
print('== conf filter ==')
report('conf>=0.90', build(lambda r: r[3] >= 0.90))
report('conf>=0.95', build(lambda r: r[3] >= 0.95))
