#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
用雷达的局部位移标定检测到的 offset 的尺度/符号 (不依赖雷达全局精度).

物理: 水槽在世界上不动. 飞机机体系横移 dy_body, 相机里的槽就该反向移 -dy_body.
      飞机偏航 dpsi, 前方 d 米处的槽点横向摆动 ≈ -d*dpsi (取决于符号).
回归:  d_off = c1 * (dP_body . y) + c2 * dpsi + c3
      c1 ≈ -1 说明 offset 尺度正确;  |c1| << 1 说明 offset 被放大了.
"""
import sys
import numpy as np
import rosbag

BAG = sys.argv[1]
CSVF = sys.argv[2]
MIN_CONF = 0.4


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
T = np.array(T); P = np.array(P); Q = np.array(Q)
Rall = np.array([quat_to_R(q) for q in Q])

raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
rows = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        rows.append((float(p[0]), float(p[1]) / 1000.0, float(p[2]), float(p[3]),
                     int(p[4]), float(p[5]), float(p[6])))
    except Exception:
        pass

# ---------- A) detected / conf 全景 ----------
print('== 全部 %d 帧 ==' % len(rows))
det = np.array([r[4] for r in rows])
cf = np.array([r[3] for r in rows])
wp = np.array([r[6] for r in rows])
print('   detected=1 共 %d (%.0f%%),  其中 conf>=0.4 的 %d' % (
    det.sum(), 100.0 * det.mean(), int(((det == 1) & (cf >= 0.4)).sum())))
if det.sum():
    dcf = cf[det == 1]
    for q in (10, 25, 50, 75, 90):
        print('   detected 帧的 conf P%-2d = %.3f' % (q, np.percentile(dcf, q)))
    print('   detected 帧的 wpx: 中位 %.1f  P10 %.1f  P90 %.1f' % (
        np.median(wp[det == 1]), np.percentile(wp[det == 1], 10), np.percentile(wp[det == 1], 90)))

# ---------- B) z(t) 与 offset 时间序列 ----------
good = [r for r in rows if r[4] == 1 and r[3] >= MIN_CONF and T[0] <= r[0] <= T[-1]]
print()
print('== 时间序列采样 (每 %.0f 个采样点取 1) ==' % max(1, len(good) // 22))
print('   %8s %7s %8s %7s %7s %7s' % ('t-t0', 'z(m)', 'off_mm', 'ang', 'conf', 'wpx'))
for k in range(0, len(good), max(1, len(good) // 22)):
    r = good[k]
    i = int(np.clip(np.searchsorted(T, r[0]), 0, len(T) - 1))
    print('   %8.1f %7.3f %8.1f %7.1f %7.2f %7.1f' % (
        r[0] - T[0], P[i, 2], r[1] * 1000, r[2], r[3], r[6]))

# ---------- C) 局部一致性回归 ----------
def build(pairs_dt):
    X, Y = [], []
    for k in range(len(good) - 1):
        t1, o1 = good[k][0], good[k][1]
        for j in range(k + 1, min(k + 6, len(good))):
            t2, o2 = good[j][0], good[j][1]
            dt = t2 - t1
            if dt <= 1e-3 or dt > pairs_dt:
                continue
            i1 = int(np.clip(np.searchsorted(T, t1), 1, len(T) - 1))
            i2 = int(np.clip(np.searchsorted(T, t2), 1, len(T) - 1))
            dp = P[i2] - P[i1]
            dpb = Rall[i1].T.dot(dp)                     # 位移转到机体系 (x前 y左 z上)
            dpsi = np.arctan2(Rall[i2][1, 0], Rall[i2][0, 0]) - np.arctan2(Rall[i1][1, 0], Rall[i1][0, 0])
            dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
            X.append([dpb[1], dpb[0], dpsi, 1.0])
            Y.append(o2 - o1)
            break
    return np.array(X), np.array(Y)

for dtmax in (0.15, 0.4):
    X, Y = build(dtmax)
    if len(Y) < 20:
        print('\n!! dt<=%.2f 样本太少 (%d)' % (dtmax, len(Y)))
        continue
    sol, _, _, _ = np.linalg.lstsq(X, Y)
    pred = X.dot(sol)
    ss = 1.0 - np.sum((Y - pred) ** 2) / max(np.sum((Y - Y.mean()) ** 2), 1e-12)
    print()
    print('== 回归 d_off ~ c1*dY_body + c2*dX_body + c3*dpsi + c4   (dt<=%.2fs, %d 样本)' % (dtmax, len(Y)))
    print('   c1(横移 y左+)  = %+.3f   <-- 理想 -1.0 (飞机左移, 槽在画面里右移, offset 变小)' % sol[0])
    print('   c2(前移 x前+)  = %+.3f' % sol[1])
    print('   c3(偏航 dpsi)  = %+.3f   (rad 变化引起的 mm 变化)' % sol[2])
    print('   c4(常数)       = %+.1f mm' % (1000 * sol[3]))
    print('   R^2 = %.3f' % ss)
    print('   d_off 标准差 %.1f mm,  dY_body 标准差 %.1f mm' % (1000 * Y.std(), 1000 * X[:, 0].std()))

# ---------- D) 只按 |off| 物理范围过滤后的残差 ----------
print()
print('== 按 |off| 上限过滤后的建图残差 (现行公式) ==')
CAM_DOWN = 0.55


def quat_R_slerp(q1, q2, a):
    return q1


Ri, Pi, of, ts = [], [], [], []
for r in good:
    i = int(np.clip(np.searchsorted(T, r[0]), 1, len(T) - 1))
    al = (r[0] - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + al * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Ri.append(u.dot(vt))
    Pi.append(P[i - 1] + al * (P[i] - P[i - 1]))
    of.append(r[1]); ts.append(r[0] - T[0])
Ri = np.array(Ri); Pi = np.array(Pi); of = np.array(of); ts = np.array(ts)

for lim in (2000, 1000, 600, 400, 300, 250, 200, 150):
    m = np.abs(of) * 1000 <= lim
    if m.sum() < 30:
        print('   |off|<=%4d mm : 仅 %d 点' % (lim, m.sum()))
        continue
    W = Pi[m] + np.einsum('kij,j->ki', Ri[m], np.array([0.0, 0.0, -CAM_DOWN])) \
        + np.einsum('kij,kj->ki', Ri[m], np.outer(-of[m], np.array([0.0, 1.0, 0.0])))
    c = W.mean(axis=0)
    u, s, vt = np.linalg.svd(W - c, full_matrices=False)
    d = vt[0]
    perp = (W - c) - np.outer((W - c).dot(d), d)
    pn = np.linalg.norm(perp, axis=1)
    print('   |off|<=%4d mm : %4d 点  中位 %6.1f  P90 %6.1f  max %6.1f mm' % (
        lim, m.sum(), 1000 * np.median(pn), 1000 * np.percentile(pn, 90), 1000 * pn.max()))
