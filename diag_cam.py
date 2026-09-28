#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
反推相机安装几何 (diagnostic, 只读不写).

模型:  世界点 = p + R * (a + off_m * b)
   p, R  = 雷达位姿 (bag /Odometry)
   off_m = 检测到的横向偏差 (m)
   a     = off=0 时水槽中心在**机体系**下的位置 (光轴与地面的交点)
   b     = off 每增加 1m 时, 该点在机体系下的位移方向 (单位向量=1 则尺度正确)

现行 build_polyline.py 用的就是 a=(0,0,-0.55), b=(0,-1,0).
本脚本把 a,b 当自由参数做最小二乘 -> 看残差能降到多少, 反推真实 b 的方向 (=相机偏航).
"""
import sys, os
import numpy as np
import rosbag

BAG = sys.argv[1]
CSVF = sys.argv[2]
CAM_DOWN = 0.55
MIN_CONF = 0.4


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---------- 1) 位姿 ----------
try:
    bag = rosbag.Bag(BAG)
except Exception:
    bag = rosbag.Bag(BAG, allow_unindexed=True)
T, P, Q = [], [], []
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
bag.close()
T = np.array(T); P = np.array(P); Q = np.array(Q)
print('== 位姿 %d 点  %.1f~%.1f s  时长 %.0f s' % (len(T), T[0], T[-1], T[-1] - T[0]))
print('   x [%.2f,%.2f] y [%.2f,%.2f] z [%.2f,%.2f]' % (
    P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(), P[:, 2].min(), P[:, 2].max()))

# 机体 yaw 序列
Rall = np.array([quat_to_R(q) for q in Q])
yaw = np.degrees(np.arctan2(Rall[:, 1, 0], Rall[:, 0, 0]))
print('   机体 yaw: 中位 %.1f  范围 [%.1f, %.1f]  std %.1f' % (
    np.median(yaw), yaw.min(), yaw.max(), yaw.std()))

# ---------- 2) 检测 ----------
raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
rows = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]),
                     int(p[4]), float(p[5]), float(p[6])))
    except Exception:
        pass
good = [r for r in rows if r[4] == 1 and r[3] >= MIN_CONF and r[0] >= T[0] and r[0] <= T[-1]]
print('== 检测 %d 帧, 有效(conf>=%.2f 且在飞行时段内) %d 帧' % (len(rows), MIN_CONF, len(good)))

td = np.array([r[0] for r in good])
off = np.array([r[1] for r in good]) / 1000.0
ang = np.array([r[2] for r in good])
cf = np.array([r[3] for r in good])
wpx = np.array([r[6] for r in good])

for nm, v in (('offset_mm', off * 1000), ('angle_deg', ang), ('conf', cf), ('wpx', wpx)):
    print('   %-10s 中位 %8.2f  P10 %8.2f  P90 %8.2f  min %8.2f  max %8.2f' % (
        nm, np.median(v), np.percentile(v, 10), np.percentile(v, 90), v.min(), v.max()))

# 帧间跳变
print('   帧间 |d_offset| 中位 %.2f mm   P90 %.2f mm' % (
    1000 * np.median(np.abs(np.diff(off))), 1000 * np.percentile(np.abs(np.diff(off)), 90)))

# ---------- 3) 逐帧位姿插值 ----------
def pose_at(ts):
    i = np.searchsorted(T, ts)
    i = min(max(i, 1), len(T) - 1)
    a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    return i, a


Ri = np.zeros((len(td), 3, 3))
Pi = np.zeros((len(td), 3))
for k, ts in enumerate(td):
    i, al = pose_at(ts)
    Ri[k] = Rall[i - 1] + al * (Rall[i] - Rall[i - 1])
    # 重新正交化
    u, s, vt = np.linalg.svd(Ri[k])
    Ri[k] = u.dot(vt)
    Pi[k] = P[i - 1] + al * (P[i] - P[i - 1])


def line_fit(W):
    """返回 (垂直残差数组, 方向, 中心)"""
    c = W.mean(axis=0)
    u, s, vt = np.linalg.svd(W - c, full_matrices=False)
    d = vt[0]
    proj = (W - c).dot(d)
    perp = (W - c) - np.outer(proj, d)
    return np.linalg.norm(perp, axis=1), d, c


def report(tag, W):
    pn, d, c = line_fit(W)
    print('%-34s 中位 %6.1f  P90 %6.1f  max %6.1f mm | 方向[%+.2f %+.2f %+.2f]' % (
        tag, 1000 * np.median(pn), 1000 * np.percentile(pn, 90), 1000 * pn.max(), d[0], d[1], d[2]))
    return np.median(pn)


# ---------- 4) 基准: 现行公式 ----------
a0 = np.array([0.0, 0.0, -CAM_DOWN])
b0 = np.array([0.0, -1.0, 0.0])
W0 = Pi + np.einsum('kij,j->ki', Ri, a0) + np.einsum('kij,kj->ki', Ri, np.outer(off, b0))
print()
print('== 残差对比 (垂直残差) ==')
report('现行 a=(0,0,-0.55) b=(0,-1,0)', W0)

# ---------- 5) 自由拟合 a, b ----------
a, b = a0.copy(), b0.copy()
for it in range(60):
    W = Pi + np.einsum('kij,j->ki', Ri, a) + np.einsum('kij,kj->ki', Ri, np.outer(off, b))
    pn, d, c = line_fit(W)
    # 投影矩阵
    Pp = np.eye(3) - np.outer(d, d)
    # 残差 = Pp(R a + off R b - (c - p))  -> 对 (a,b) 线性
    Rb = np.einsum('kij,k->ij', Ri, off)          # sum_k off_k R_k  (3x3)
    lhs = np.zeros((3 * len(td), 6))
    rhs = np.zeros(3 * len(td))
    for k in range(len(td)):
        lhs[3 * k:3 * k + 3, 0:3] = Pp.dot(Ri[k])
        lhs[3 * k:3 * k + 3, 3:6] = Pp.dot(Ri[k]) * off[k]
        rhs[3 * k:3 * k + 3] = Pp.dot(c - Pi[k])
    sol, _, _, _ = np.linalg.lstsq(lhs, rhs)
    na, nb = sol[0:3], sol[3:6]
    if np.linalg.norm(na - a) < 1e-5 and np.linalg.norm(nb - b) < 1e-5:
        a, b = na, nb
        break
    a, b = na, nb

W = Pi + np.einsum('kij,j->ki', Ri, a) + np.einsum('kij,kj->ki', Ri, np.outer(off, b))
report('自由拟合 a,b (%d 次迭代)' % (it + 1), W)
print()
print('   拟合 a = [%+.4f, %+.4f, %+.4f] m   (off=0 时水槽中心, 机体系 x前 y左 z上)' % (a[0], a[1], a[2]))
print('   拟合 b = [%+.4f, %+.4f, %+.4f]     |b| = %.4f' % (b[0], b[1], b[2], np.linalg.norm(b)))
bh = np.array([b[0], b[1], 0.0])
if np.linalg.norm(bh) > 1e-6:
    print('   b 的机体系水平方向 = %.1f deg (atan2(y,x))' % np.degrees(np.arctan2(bh[1], bh[0])))
    print('   -> 若 b 代表"右方", 相机相对机体的偏航 = %.1f deg' % (
        np.degrees(np.arctan2(bh[1], bh[0])) - 90.0))
print('   a 的水平距离 %.3f m, 俯角(向下看) = %.1f deg' % (
    np.hypot(a[0], a[1]), np.degrees(np.arctan2(-a[2], max(np.hypot(a[0], a[1]), 1e-6)))))
print('   (几何关系: 高度 h 时应有 |a水平| = h*cot(俯角), b 的垂直分量=0 才自洽)')
print('   a 水平 / -a_z = %.3f   -> 若相机俯角为 al, 应等于 cot(al)' % (
    np.hypot(a[0], a[1]) / max(-a[2], 1e-6)))

# ---------- 6) 只用"物理约束"模型: b=(sinB,-cosB,0), a=(k*cosB, k*sinB, -h) ----------
print()
print('== 物理约束模型扫描: a=(k cosB, k sinB, -h), b=(sinB,-cosB,0) ==')
best = None
for Bd in np.arange(-120, 121, 2.5):
    B = np.radians(Bd)
    bb = np.array([np.sin(B), -np.cos(B), 0.0])
    for k in np.arange(0.0, 2.51, 0.1):
        for h in (0.3, 0.4, 0.5, 0.55, 0.6, 0.7, 0.8, 1.0):
            aa = np.array([k * np.cos(B), k * np.sin(B), -h])
            WW = Pi + np.einsum('kij,j->ki', Ri, aa) + np.einsum('kij,kj->ki', Ri, np.outer(off, bb))
            pn, d, c = line_fit(WW)
            m = np.median(pn)
            if best is None or m < best[0]:
                best = (m, Bd, k, h, np.percentile(pn, 90), pn.max())
print('   最优: Beta=%.1f deg  k=%.1f m  h=%.2f m  -> 中位 %.1f mm  P90 %.1f  max %.1f' % (
    best[1], best[2], best[3], 1000 * best[0], 1000 * best[4], 1000 * best[5]))

# 固定 Beta 扫描, 看单一变量的影响
print()
print('   k,h 已最优时, 各 Beta 的中位残差:')
for Bd in np.arange(-100, 101, 10):
    B = np.radians(Bd)
    bb = np.array([np.sin(B), -np.cos(B), 0.0])
    bm = None
    for k in np.arange(0.0, 2.51, 0.1):
        for h in (0.3, 0.4, 0.5, 0.55, 0.6, 0.7, 0.8, 1.0):
            aa = np.array([k * np.cos(B), k * np.sin(B), -h])
            WW = Pi + np.einsum('kij,j->ki', Ri, aa) + np.einsum('kij,kj->ki', Ri, np.outer(off, bb))
            pn, d, c = line_fit(WW)
            m = np.median(pn)
            if bm is None or m < bm[0]:
                bm = (m, k, h)
    print('     Beta %+5.0f : %6.1f mm  (k=%.1f h=%.2f)' % (Bd, 1000 * bm[0], bm[1], bm[2]))
