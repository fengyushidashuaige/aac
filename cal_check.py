#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""检查那套角度标定靠不靠得住: 自变量共线性 + 系数误差 + 稳健性.
用法: python2 cal_check.py <bag> <detect.csv>
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
    if det == 1 and conf >= 0.5 and T[0] <= t <= T[-1] and abs(off) <= 250 and abs(ang) <= 45:
        A.append((t, off, ang))
A = np.array(A)
h = np.interp(A[:, 0], T, P[:, 2])
yaw = np.interp(A[:, 0], T, Y)
psi = (traf - yaw + np.pi) % (2 * np.pi) - np.pi
dh = (A[:, 1] / 1000.0) / np.maximum(h, 0.05)
TILT = A[:, 2]

print('样本 %d' % len(A))
print('  psi_true: std %.1f deg   范围 %.1f ~ %.1f' % (
    np.degrees(psi.std()), np.degrees(psi.min()), np.degrees(psi.max())))
print('  d/h     : std %.3f      范围 %.3f ~ %.3f' % (dh.std(), dh.min(), dh.max()))
print('  倾角tilt: std %.1f deg   范围 %.1f ~ %.1f' % (TILT.std(), TILT.min(), TILT.max()))
print()
print('★ 共线性检查 (自变量相关性):')
print('   corr( tan(psi), d/h )        = %+.3f   <-- |corr| 高 => 系数分不开, 标定不可靠' % (
    np.corrcoef(np.tan(psi), dh)[0, 1]))
print('   corr( psi, d/h )             = %+.3f' % np.corrcoef(psi, dh)[0, 1])
print('   corr( tilt, psi )            = %+.3f' % np.corrcoef(TILT, psi)[0, 1])
print('   corr( tilt, d/h )            = %+.3f' % np.corrcoef(TILT, dh)[0, 1])
print()
X = np.column_stack([np.tan(psi), dh, np.ones(len(A))])
Yv = np.tan(np.radians(TILT))
sol, _, _, _ = np.linalg.lstsq(X, Yv)
res = Yv - X.dot(sol)
dof = len(A) - 3
s2 = (res ** 2).sum() / dof
cov = s2 * np.linalg.inv(X.T.dot(X))
se = np.sqrt(np.diag(cov))
print('系数 (A, B, C) = (%.3f, %.3f, %.3f)' % tuple(sol))
print('标准误        = (%.3f, %.3f, %.3f)' % tuple(se))
print('  => A 的 t 值 %.1f,  B 的 t 值 %.1f   (|t|<2 基本等于没测出来)' % (
    sol[0] / se[0], sol[1] / se[1]))
print()
print('VIF(方差膨胀, >5 说明共线性严重):')
XtX = X.T.dot(X)
for i in range(3):
    others = np.delete(X, i, axis=1)
    b, _, _, _ = np.linalg.lstsq(others, X[:, i])
    r2 = 1 - ((X[:, i] - others.dot(b)) ** 2).sum() / ((X[:, i] - X[:, i].mean()) ** 2).sum()
    print('  x%d: VIF %.2f  (R2 vs 其它自变量 %.3f)' % (i, 1.0 / max(1 - r2, 1e-6), r2))
