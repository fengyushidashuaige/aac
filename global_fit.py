#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
全局闭式估计 (比逐对回归稳得多).

模型: 槽是世界里一条水平直线 {c + s*d}, 法向 n.
      投影出的点 W_i = p_i + R_i*a + off_i*(R_i*b) 必须落在直线上.
      b = 机体系"横向"单位向量 (真值应是 (0,-1,0), 即 +off = 槽在右侧).
条件: (W_i - c)·n = 0
  =>  p_i·n + (R_i a)·n + k*off_i*(R_i b·n) = c·n = 常数
给定 n (角度 theta), 对 (k, 常数) 是线性的 -> 最小二乘直接解.
  k = 1.0  说明检测的 mm 就是真 mm (尺度对)
  k > 1    说明 off 被放大了
残差 = 垂直距离(mm), 用 n 分量 (世界系真实横向距离)
同时可扫相机安装偏航 beta: b = (sin b, -cos b, 0)
"""
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
# (t, off_m, ang, conf, det, wpx)


def build(sel, wpx_band=None, dt_shift=0.0):
    """取帧 -> 位姿(可加时移) -> 返回 p2d(N,2), Ra2d(N,2), off(N), rb2d 用的 R(N,3,3)"""
    base = [r for r in rows if r[4] == 1 and r[0] >= T[0] and r[0] <= T[-1] and sel(r)]
    if wpx_band is not None:
        med = np.median([r[5] for r in base])
        base = [r for r in base if abs(r[5] - med) <= wpx_band * med]
    out = []
    for r in base:
        ts = r[0] + dt_shift
        i = int(np.clip(np.searchsorted(T, ts), 1, len(T) - 1))
        a = (ts - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
        R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
        u, s, vt = np.linalg.svd(R)
        out.append((u.dot(vt), P[i - 1] + a * (P[i] - P[i - 1]), r[1]))
    return out


def fit(data, a_vec, beta_deg, thetas):
    Rl = np.array([d[0] for d in data])
    Pl = np.array([d[1] for d in data])
    off = np.array([d[2] for d in data])
    Ra = np.einsum('kij,j->ki', Rl, np.array(a_vec))
    b = np.array([np.sin(np.radians(beta_deg)), -np.cos(np.radians(beta_deg)), 0.0])
    Rb = np.einsum('kij,j->ki', Rl, b)
    best = None
    for th in thetas:
        n2 = np.array([-np.sin(th), np.cos(th)])
        A = off * (Rb[:, :2].dot(n2))          # 对 k 的系数
        D = Pl[:, :2].dot(n2) + Ra[:, :2].dot(n2)
        M = np.column_stack([A, np.ones(len(A))])
        sol, _, _, _ = np.linalg.lstsq(M, -D)
        res = (M.dot(sol) + D) * 1000.0        # mm
        e = np.median(np.abs(res))
        if best is None or e < best[0]:
            best = (e, np.degrees(th), sol[0], np.percentile(np.abs(res), 90), np.abs(res).max(),
                    1.96 * res.std() / np.sqrt(len(res)))
    return best


thetas = np.radians(np.arange(0, 180, 0.25))
print('帧集合 -> (尺度 k, 槽走向 theta, 残差)')
print()
print('%-42s %5s %8s %8s %8s %8s' % ('数据', 'theta', 'k', '中位残差', 'P90', 'max'))

cases = [
    ('全部 detected', lambda r: True, None, 0.0),
    ('conf>=0.4', lambda r: r[3] >= 0.4, None, 0.0),
    ('conf>=0.9', lambda r: r[3] >= 0.9, None, 0.0),
    ('conf>=0.9 & wpx 带内15%', lambda r: r[3] >= 0.9, 0.15, 0.0),
    ('conf>=0.9 & wpx 带内10%', lambda r: r[3] >= 0.9, 0.10, 0.0),
    ('conf>=0.9 & wpx 带内15% & 时移-0.28s', lambda r: r[3] >= 0.9, 0.15, -0.28),
    ('conf>=0.6 & wpx 带内15%', lambda r: r[3] >= 0.6, 0.15, 0.0),
]
for tag, sel, band, dts in cases:
    data = build(sel, band, dts)
    if len(data) < 40:
        print('%-42s 点数不足 %d' % (tag, len(data)))
        continue
    e, th, k, p90, mx, se = fit(data, (0.0, 0.0, -CAM_DOWN), 0.0, thetas)
    print('%-42s %5.1f %8.3f %8.1f %8.1f %8.1f   (n=%d, k±%.3f)' % (
        tag, th, k, e, p90, mx, len(data), se))

print()
print('== 扫相机安装偏航 beta (conf>=0.9 & wpx 带内15%) ==')
data = build(lambda r: r[3] >= 0.9, 0.15, 0.0)
for bd in (-60, -40, -20, -10, 0, 10, 20, 40, 60):
    e, th, k, p90, mx, se = fit(data, (0.0, 0.0, -CAM_DOWN), bd, thetas)
    print('  beta %+4d: theta %5.1f  k %6.3f  中位残差 %6.1f  P90 %7.1f mm' % (bd, th, k, e, p90))

print()
print('== 扫 a 的前向距离 k_f (a=(k_f,0,-CAM_DOWN)), beta=0 ==')
for kf in (0.0, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8):
    e, th, k, p90, mx, se = fit(data, (kf, 0.0, -CAM_DOWN), 0.0, thetas)
    print('  a_x %+0.2f: theta %5.1f  k %6.3f  中位残差 %6.1f  P90 %7.1f mm' % (kf, th, k, e, p90))

print()
print('== 扫 CAM_DOWN (a=(0,0,-h)) ==')
for h in (0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.7):
    e, th, k, p90, mx, se = fit(data, (0.0, 0.0, -h), 0.0, thetas)
    print('  h %.2f: theta %5.1f  k %6.3f  中位残差 %6.1f  P90 %7.1f mm' % (h, th, k, e, p90))
