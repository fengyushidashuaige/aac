#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
验证: 扣掉"横向偏差项"之后, 相机夹角能否还原真实航向?

几何(相机俯角 alpha, 观测点横向偏差 d, 高度 h):
    tan(tilt) = tan(psi)*sin(alpha) - (d/h)*cos(alpha)
=>  psi = atan( (tan(tilt) + (d/h)*cos(alpha)) / sin(alpha) )

真值由雷达给: psi_true = 路径方向 - 机头yaw   (路径很直, 方向可靠)

用法: python2 psi_from_ang.py <bag> <detect.csv> <alpha_deg>
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


ALPHA = np.radians(float(sys.argv[3]) if len(sys.argv) > 3 else 43.6)
bag = rosbag.Bag(sys.argv[1])
T, P, Y, Z = [], [], [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    R = q2R((q.x, q.y, q.z, q.w))
    T.append(t.to_sec()); P.append((p.x, p.y, p.z))
    Y.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
bag.close()
T = np.array(T); P = np.array(P); Y = np.unwrap(np.radians(np.array(Y)))

# 路径方向: 对整条轨迹做直线拟合(飞机沿槽飞)
c = P[:, :2].mean(axis=0)
u, s, vt = np.linalg.svd(P[:, :2] - c, full_matrices=False)
d = vt[0]
traf_dir = np.arctan2(d[1], d[0])
print('轨迹主方向 %.1f deg (奇异值 %s)' % (np.degrees(traf_dir), np.round(s, 2)))

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
print('样本 %d 帧' % len(A))
h = np.interp(A[:, 0], T, P[:, 2])
yaw = np.interp(A[:, 0], T, Y)
psi_true = np.degrees((traf_dir - yaw + np.pi) % (2 * np.pi) - np.pi)
tilt = np.radians(A[:, 2])
dd = A[:, 1] / 1000.0

print()
print('=== 直接用夹角当航向 (现状) ===')
for sgn in (1, -1):
    a = sgn * A[:, 2]
    r = np.corrcoef(a, psi_true)[0, 1]
    print('  ang%s  vs psi_true : corr %+.3f' % ('' if sgn > 0 else '*(-1)', r))

print()
print('=== 扣掉偏差项之后 ===')
for sgn in (1, -1):
    psi = np.degrees(np.arctan((np.tan(tilt) + sgn * (dd / h) * np.cos(ALPHA)) / np.sin(ALPHA)))
    r = np.corrcoef(psi, psi_true)[0, 1]
    fit = np.polyfit(psi_true, psi, 1)
    res = psi - np.polyval(fit, psi_true)
    print('  psi(偏差项sgn=%+d) vs psi_true: corr %+.3f  斜率 %.2f  残差std %.1f deg  P90 %.1f' % (
        sgn, r, fit[0], res.std(), np.percentile(np.abs(res), 90)))

print()
print('=== 对照: 只用夹角(不扣偏差) ===')
psi0 = A[:, 2]
res0 = psi0 - np.polyval(np.polyfit(psi_true, psi0, 1), psi_true)
print('  残差 std %.1f deg  P90 %.1f' % (res0.std(), np.percentile(np.abs(res0), 90)))
print()
print('  psi_true 范围: %.1f ~ %.1f (std %.1f)' % (
    psi_true.min(), psi_true.max(), psi_true.std()))
