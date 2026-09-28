#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
门控 + 尺度 方案评估台 (离线, 只读).

思路:
  offset_px = off_mm * wpx / 110        # 还原检测器内部像素量, 与 wpx 无关
  坏帧 = mask 被截断 -> wpx 偏小 -> mm_per_px=110/wpx 爆表 -> off_mm 爆表
门控/尺度候选:
  V0 现状: 只用 conf>=0.4
  V1 只门控: wpx 在滑动中位带内 + |off| 物理上限 + conf
  V2 门控 + 尺度换成滑动中位 (wpx 只当质量门)
  V3 门控 + 尺度由雷达高度推 (mm_per_px = K/h, K 由好帧标定)
验收 (两条一起):
  ① 建图垂直残差 中位/P90/max   (不可单独用: 对常数项盲 + 偏爱缩小值)
  ② c1 回归: 雷达局部位移 <-> offset 变化, 必须仍 ≈ +1.0 (尺度没被改歪)
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
        rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
# 基础过滤: 检出 + 在飞行时段 + conf 底线(很低, 后面逐档调)
G = [r for r in rows if r[4] == 1 and T[0] <= r[0] <= T[-1] and r[3] >= 0.30]
ts = np.array([r[0] for r in G])
conf = np.array([r[3] for r in G])
wpx = np.array([r[5] for r in G])
off_mm = np.array([r[1] for r in G])
off_px = off_mm * wpx / 110.0            # 还原像素量
ang = np.array([r[2] for r in G])
print('候选帧 %d (detected=1, conf>=0.30, 在飞行时段)' % len(G))

# 位姿插值
Ri, Pi = [], []
for t in ts:
    i = int(np.clip(np.searchsorted(T, t), 1, len(T) - 1))
    a = (t - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + a * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Ri.append(u.dot(vt)); Pi.append(P[i - 1] + a * (P[i] - P[i - 1]))
Ri = np.array(Ri); Pi = np.array(Pi)
hgt = Pi[:, 2]                            # 相对 FAST-LIO 原点的高度


def roll_med(v, win):
    """滑动中位 (只在前向窗口上取, 因果, 现场可用)"""
    out = np.empty(len(v))
    for i in range(len(v)):
        lo = max(0, i - win)
        out[i] = np.median(v[lo:i + 1])
    return out


def phys_limits():
    """由雷达高度给物理上限: 横向可见半宽 = h * tan(HFOV/2)"""
    return hgt


def evaluate(mask, off_used_mm, tag, verbose=True):
    if mask.sum() < 40:
        if verbose:
            print('  %-46s 点数不足 (%d)' % (tag, mask.sum()))
        return None
    offm = off_used_mm[mask] / 1000.0
    W = Pi[mask] + np.einsum('kij,j->ki', Ri[mask], np.array([0.0, 0.0, -CAM_DOWN])) \
        + np.einsum('kij,kj->ki', Ri[mask], np.outer(-offm, np.array([0.0, 1.0, 0.0])))
    c = W.mean(axis=0)
    u, s, vt = np.linalg.svd(W - c, full_matrices=False)
    d = vt[0]
    perp = (W - c) - np.outer((W - c).dot(d), d)
    pn = np.linalg.norm(perp, axis=1) * 1000

    # c1 回归 (好帧上的局部一致性) —— 只保留 dt<=0.4s 的相邻对
    idx = np.where(mask)[0]
    X, Y = [], []
    pos = {v: k for k, v in enumerate(idx)}
    for k in range(len(idx) - 1):
        i1, i2 = idx[k], idx[k + 1]
        dt = ts[i2] - ts[i1]
        if dt <= 1e-3 or dt > 0.4:
            continue
        if i2 - i1 > 6:
            continue
        dpb = Ri[i1].T.dot(Pi[i2] - Pi[i1])
        dpsi = np.arctan2(Ri[i2][1, 0], Ri[i2][0, 0]) - np.arctan2(Ri[i1][1, 0], Ri[i1][0, 0])
        dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
        X.append([dpb[1], dpb[0], dpsi, 1.0])
        Y.append((off_used_mm[i2] - off_used_mm[i1]) / 1000.0)
    c1 = c1e = r2 = float('nan')
    if len(Y) > 30:
        X = np.array(X); Y = np.array(Y)
        sol, _, _, _ = np.linalg.lstsq(X, Y)
        pred = X.dot(sol)
        c1 = sol[0]
        c1e = np.sqrt(np.sum((Y - pred) ** 2) / max(len(Y) - 4, 1)
                      / max(np.sum((X[:, 0] - X[:, 0].mean()) ** 2), 1e-9))
        r2 = 1 - np.sum((Y - pred) ** 2) / max(np.sum((Y - Y.mean()) ** 2), 1e-12)
    if verbose:
        print('  %-46s %4d点(%3.0f%%)  残差 中位%6.1f P90%6.1f max%7.1f | c1 %+.2f±%.2f R2 %.2f | z中位 %+.3f' % (
            tag, mask.sum(), 100.0 * mask.mean(), np.median(pn), np.percentile(pn, 90), pn.max(),
            c1, c1e, r2, np.median(W[:, 2])))
    return np.median(pn), c1, mask.sum()


print()
print('== V0 现状 (conf 门限扫描, 尺度=逐帧 110/wpx) ==')
for cmin in (0.3, 0.4, 0.6, 0.9):
    evaluate(conf >= cmin, off_mm, 'V0 conf>=%.1f' % cmin)

# 滑动中位与带宽
wmed = roll_med(wpx, 120)
print()
print('== wpx 参考(滑动中位) 中位 %.1f, 范围 [%.1f, %.1f] ==' % (
    np.median(wmed), wmed.min(), wmed.max()))

print()
print('== V1 门控: wpx 在滑动中位 ±BAND, conf>=CMIN, |off| <= 物理上限 ==')
print('   (物理上限: 横向可见半宽 = 高度*tan(35deg) ≈ 0.7h)')
for band in (0.10, 0.15, 0.25):
    for cmin in (0.3, 0.6, 0.9):
        m = (np.abs(wpx - wmed) <= band * wmed) & (conf >= cmin) & \
            (np.abs(off_mm) <= 700.0 * hgt)
        evaluate(m, off_mm, 'V1 band=%.2f conf>=%.1f' % (band, cmin))

print()
print('== V2 门控 + 尺度换成滑动中位 (off = off_px * 110/wmed) ==')
off_v2 = off_px * 110.0 / wmed
for band in (0.10, 0.15, 0.25):
    for cmin in (0.3, 0.6, 0.9):
        m = (np.abs(wpx - wmed) <= band * wmed) & (conf >= cmin) & \
            (np.abs(off_v2) <= 700.0 * hgt)
        evaluate(m, off_v2, 'V2 band=%.2f conf>=%.1f' % (band, cmin))

print()
print('== V3 门控 + 尺度由高度推 (mm_per_px = K/h, K 由 conf>=0.9 好帧标定) ==')
good0 = (conf >= 0.9) & (np.abs(wpx - wmed) <= 0.15 * wmed)
if good0.sum() > 30:
    K = np.median((110.0 / wpx[good0]) * hgt[good0])
    print('   K = %.4f  (mm_per_px * h)' % K)
    off_v3 = off_px * K / hgt
    for band in (0.15, 0.25):
        for cmin in (0.3, 0.6, 0.9):
            m = (np.abs(wpx - wmed) <= band * wmed) & (conf >= cmin) & \
                (np.abs(off_v3) <= 700.0 * hgt)
            evaluate(m, off_v3, 'V3 band=%.2f conf>=%.1f' % (band, cmin))

print()
print('== 时间离群剔除 (在 V1 band=0.15 conf>=0.6 之上再加) ==')
base = (np.abs(wpx - wmed) <= 0.15 * wmed) & (conf >= 0.6) & (np.abs(off_mm) <= 700.0 * hgt)
omed = roll_med(off_mm, 25)
for tol in (80, 150, 9999):
    m = base & (np.abs(off_mm - omed) <= tol)
    evaluate(m, off_mm, 'V1 + |off-滑动中位| <= %dmm' % tol)
