#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
用飞行数据(有雷达真值)标定角度公式的系数:
    tan(tilt) = A * tan(psi) + B * (d/h)
真值 psi_true = 轨迹方向 - 机头yaw. 最小二乘解 A,B, 再反解 psi 看残差.
用法: python2 cal_ang.py <bag> <detect.csv>
"""
import sys
import numpy as np
import rosbag


def q2R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


bag = rosbag.Bag(sys.argv[1])
T, P, Y = [], [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    R = q2R((q.x, q.y, q.z, q.w))
    T.append(t.to_sec()); P.append((p.x, p.y, p.z))
    Y.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
bag.close()
T = np.array(T); P = np.array(P); Y = np.unwrap(np.radians(np.array(Y)))
c = P[:, :2].mean(axis=0)
u, s, vt = np.linalg.svd(P[:, :2] - c, full_matrices=False)
traf = np.arctan2(vt[0][1], vt[0][0])

raw = open(sys.argv[2], 'rb').read().replace(b'\x00', b'')
A = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        t = float(p[0]); off = float(p[1]); ang = float(p[2]); conf = float(p[3]); det = int(p[4])
    except Exception:
        continue
    if det == 1 and conf >= 0.5 and T[0] <= t <= T[-1] and abs(off) <= 250:
        A.append((t, off, ang))
A = np.array(A)
h = np.interp(A[:, 0], T, P[:, 2])
yaw = np.interp(A[:, 0], T, Y)
psi_true = (traf - yaw + np.pi) % (2 * np.pi) - np.pi
tilt = np.radians(A[:, 2])
dh = (A[:, 1] / 1000.0) / h
print('样本 %d,  h 中位 %.3f,  psi_true std %.1f deg' % (len(A), np.median(h), np.degrees(psi_true.std())))
print('d/h 范围 %.3f ~ %.3f (std %.3f);  tan(tilt) 范围 %.3f ~ %.3f' % (
    dh.min(), dh.max(), dh.std(), np.tan(tilt).min(), np.tan(tilt).max()))

Yv = np.tan(tilt)
X = np.column_stack([np.tan(psi_true), dh, np.ones(len(A))])
sol, _, _, _ = np.linalg.lstsq(X, Yv)
print()
print('标定结果:  tan(tilt) = %.3f*tan(psi) + %.3f*(d/h) + %.3f' % (sol[0], sol[1], sol[2]))
print('  理论值应为: sin(alpha)=%.3f   -cos(alpha)/1=%.3f  (h 已含在 d/h 里)' % (
    np.sin(np.radians(43.6)), -np.cos(np.radians(43.6))))
pred = X.dot(sol)
r2 = 1 - np.sum((Yv - pred) ** 2) / np.sum((Yv - Yv.mean()) ** 2)
print('  拟合 R2 = %.3f' % r2)

# 反解 psi
psi_rec = np.arctan((Yv - sol[1] * dh - sol[2]) / sol[0])
res = np.degrees(psi_rec - psi_true)
print()
print('=== 反解出来的航向 vs 真值 ===')
print('  残差 std %.1f deg   |残差| 中位 %.1f  P90 %.1f  max %.1f' % (
    res.std(), np.median(np.abs(res)), np.percentile(np.abs(res), 90), np.abs(res).max()))
print('  corr %.3f' % np.corrcoef(psi_rec, psi_true)[0, 1])

# 对照: 只用夹角 / 只用偏差
for tag, M in (('只用夹角', np.column_stack([np.zeros(len(A)), np.zeros(len(A)), np.ones(len(A))])),
               ('只用 d/h', np.column_stack([np.zeros(len(A)), dh, np.ones(len(A))]))):
    s2, _, _, _ = np.linalg.lstsq(M, Yv)
    p2 = M.dot(s2)
    r22 = 1 - np.sum((Yv - p2) ** 2) / np.sum((Yv - Yv.mean()) ** 2)
    print('  [对照] %s 的 R2 = %.3f' % (tag, r22))

print()
print('  抽查 (psi_true vs psi_rec):')
for k in range(0, len(A), max(1, len(A) // 12)):
    print('    真值 %+6.1f   反解 %+6.1f   (tilt %+5.1f  d/h %+.3f)' % (
        np.degrees(psi_true[k]), np.degrees(psi_rec[k]), A[k, 2], dh[k]))
