#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
决定性检验: 相机测的 ang 到底有没有携带"航向误差"信息?

真值(雷达给):  herr_true(t) = 机头 yaw(t) − 实际飞行方向(t)
   (飞机沿槽飞 -> 速度方向≈槽方向 -> yaw−速度方向 就是机头相对槽的夹角)
然后把 detect.csv 里的 ang 与 herr_true 按时间对齐, 看相关性和斜率.

用法: python2 ang_truth.py <bag> <detect.csv>
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
T, P, Y, VX, VY, SP = [], [], [], [], [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    R = q2R((q.x, q.y, q.z, q.w))
    v = m.twist.twist.linear
    T.append(t.to_sec()); P.append((p.x, p.y, p.z))
    Y.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    VX.append(v.x); VY.append(v.y); SP.append(np.hypot(v.x, v.y))
bag.close()
VX = np.array(VX); VY = np.array(VY); SP = np.array(SP)
print('twist 速度: 中位 %.3f m/s  P90 %.3f' % (np.median(SP), np.percentile(SP, 90)))
T = np.array(T); P = np.array(P); Y = np.unwrap(np.radians(np.array(Y)))

# 飞行方向: ±1.5s 窗内对位置做 PCA(主方向), 低速也稳
VD = np.full(len(T), np.nan)
W = 15
for i in range(len(T)):
    a = max(0, i - W); b = min(len(T), i + W + 1)
    pts = P[a:b, :2] - P[a:b, :2].mean(axis=0)
    if len(pts) < 6:
        continue
    u, sv, vt = np.linalg.svd(pts, full_matrices=False)
    if sv[0] < 0.02 or sv[0] < 3 * max(sv[1], 1e-9):
        continue                      # 这段几乎没走 / 轨迹太"圆"
    d = vt[0]
    VD[i] = np.arctan2(d[1], d[0])
ok = ~np.isnan(VD)
print('有效方向采样 %d/%d' % (ok.sum(), len(T)))
VD = np.interp(T, T[ok], VD[ok])
HTRUE = np.degrees((Y - VD + np.pi) % (2 * np.pi) - np.pi)
print('雷达真值 herr_true: 中位 %+.1f  P10 %+.1f  P90 %+.1f  std %.1f deg' % (
    np.median(HTRUE), np.percentile(HTRUE, 10), np.percentile(HTRUE, 90), HTRUE.std()))

raw = open(sys.argv[2], 'rb').read().replace(b'\x00', b'')
A = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        t = float(p[0]); ang = float(p[2]); conf = float(p[3]); det = int(p[4])
    except Exception:
        continue
    if det == 1 and conf >= 0.5 and T[0] <= t <= T[-1]:
        A.append((t, ang, conf))
A = np.array(A)
if len(A) < 30:
    print('样本太少 %d' % len(A)); sys.exit(0)
ht = np.interp(A[:, 0], T, HTRUE)
ang = A[:, 1]
print('对齐后 n=%d' % len(A))
print('  ang 中位 %+.1f (P10 %+.1f P90 %+.1f)' % (
    np.median(ang), np.percentile(ang, 10), np.percentile(ang, 90)))


def corr(a, b):
    a = a - a.mean(); b = b - b.mean()
    return float(a.dot(b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-12))


print()
print('  corr(ang, herr_true)     = %+.3f' % corr(ang, ht))
print('  corr(ang, -herr_true)    = %+.3f' % corr(ang, -ht))
s = np.polyfit(ht, ang, 1)
pred = np.polyval(s, ht)
r2 = 1 - np.sum((ang - pred) ** 2) / max(np.sum((ang - ang.mean()) ** 2), 1e-12)
print('  线性拟合 ang = %.2f * herr_true %+.1f   R2 = %.3f' % (s[0], s[1], r2))
print()
if r2 > 0.3:
    print('  => 角度【携带航向信息】, 可用(偏置+斜率可标定)')
else:
    print('  => 角度【基本不携带航向信息】(R2 低) —— 这个测法在这个几何下不可靠')
print()
print('  抽查对照 (t, ang, herr_true):')
for k in range(0, len(A), max(1, len(A) // 14)):
    print('    ang %+7.1f   herr_true %+7.1f' % (ang[k], ht[k]))
