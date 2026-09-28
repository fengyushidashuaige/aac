#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
核心假设: offset_mm = offset_px * (trough_width_mm/wpx) 这个换算,
          只在"mask 就是完整槽宽、且和 offset 采样行同深度"时成立.
          一旦 mask 只框到槽的一部分(被 ROI 截断/遮挡/模型只出一块),
          mm_per_px 就被放大 -> offset_mm 爆掉 -> 建图全乱.

验证: 按 wpx 稳定性 + |off| 物理上限过滤, 看残差能不能掉到 <20mm.
      再网格搜"真实槽宽", 看最佳值.
"""
import sys
import numpy as np
import rosbag

BAG = sys.argv[1]
CSVF = sys.argv[2]
CAM_DOWN = 0.55
CONF = 0.4


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
T = np.array(T); P = np.array(P); Rall = np.array([quat_to_R(q) for q in Q])

raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
G = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        r = (float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]), float(p[6]))
    except Exception:
        continue
    if r[4] == 1 and r[3] >= CONF and T[0] <= r[0] <= T[-1]:
        G.append(r)
G = np.array(G)
ts, off_mm, ang, cf, det, wpx = G[:, 0], G[:, 1], G[:, 2], G[:, 3], G[:, 4], G[:, 5]

# 恢复 offset_px (detector 内部量)
off_px = off_mm / 110.0 * wpx
print('== 由 off_mm,wpx 反推的 offset_px ==')
for nm, v in (('offset_px', off_px),):
    print('   %-10s 中位 %7.1f  P10 %7.1f  P90 %7.1f  min %7.1f  max %7.1f' % (
        nm, np.median(v), np.percentile(v, 10), np.percentile(v, 90), v.min(), v.max()))
print('   画面宽 640 -> 物理上 |offset_px| 不可能超过 ~320')
print('   |offset_px|>320 的帧: %d / %d (%.0f%%)' % (
    (np.abs(off_px) > 320).sum(), len(off_px), 100.0 * (np.abs(off_px) > 320).mean()))
print('   |offset_px|>480 的帧: %d (%.0f%%)' % (
    (np.abs(off_px) > 480).sum(), 100.0 * (np.abs(off_px) > 480).mean()))

# 位姿
Ri, Pi = [], []
for t in ts:
    i = int(np.clip(np.searchsorted(T, t), 1, len(T) - 1))
    al = (t - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    R = Rall[i - 1] + al * (Rall[i] - Rall[i - 1])
    u, s, vt = np.linalg.svd(R)
    Ri.append(u.dot(vt)); Pi.append(P[i - 1] + al * (P[i] - P[i - 1]))
Ri = np.array(Ri); Pi = np.array(Pi)


def resid(mask, scale=1.0, cam_down=CAM_DOWN):
    if mask.sum() < 30:
        return None
    offm = off_mm[mask] / 1000.0 * scale
    W = Pi[mask] + np.einsum('kij,j->ki', Ri[mask], np.array([0.0, 0.0, -cam_down])) \
        + np.einsum('kij,kj->ki', Ri[mask], np.outer(-offm, np.array([0.0, 1.0, 0.0])))
    c = W.mean(axis=0)
    u, s, vt = np.linalg.svd(W - c, full_matrices=False)
    d = vt[0]
    perp = (W - c) - np.outer((W - c).dot(d), d)
    pn = np.linalg.norm(perp, axis=1)
    return np.median(pn) * 1000, np.percentile(pn, 90) * 1000, pn.max() * 1000, len(pn)


med_w = np.median(wpx)
print()
print('== 残差 vs 过滤条件 (wpx 中位 %.1f) ==' % med_w)
tests = [
    ('全部 good 帧', np.ones(len(ts), bool)),
    ('|off_px| <= 320', np.abs(off_px) <= 320),
    ('|off_px| <= 200', np.abs(off_px) <= 200),
    ('wpx 在 [0.8,1.25]x中位', (wpx >= med_w * 0.8) & (wpx <= med_w * 1.25)),
    ('wpx 在 [0.9,1.11]x中位', (wpx >= med_w * 0.9) & (wpx <= med_w * 1.11)),
    ('两者都要 (px<=320 & wpx窄)', (np.abs(off_px) <= 320) & (wpx >= med_w * 0.8) & (wpx <= med_w * 1.25)),
    ('conf>=0.9', cf >= 0.9),
    ('conf>=0.9 & |off_px|<=320', (cf >= 0.9) & (np.abs(off_px) <= 320)),
    ('conf>=0.9 & wpx窄 & |off_px|<=320',
     (cf >= 0.9) & (np.abs(off_px) <= 320) & (wpx >= med_w * 0.8) & (wpx <= med_w * 1.25)),
]
for nm, m in tests:
    r = resid(m)
    if r is None:
        print('   %-34s 点数不足 (%d)' % (nm, m.sum()))
    else:
        print('   %-34s %4d 点  中位 %6.1f  P90 %6.1f  max %6.1f mm' % (nm, r[3], r[0], r[1], r[2]))

print()
print('== 网格搜真实槽宽 (全 good 帧, 只缩放 off) ==')
best = None
for wr in np.arange(90, 200, 5):
    sc = wr / 110.0
    r = resid(np.ones(len(ts), bool), scale=sc)
    if best is None or r[0] < best[0]:
        best = (r[0], wr, r[1])
    print('   槽宽 %3d mm (scale %.2f): 中位 %6.1f  P90 %6.1f mm' % (wr, sc, r[0], r[1]))
print('   最佳槽宽 %d mm' % best[1])

print()
print('== 网格搜高度 CAM_DOWN (全 good 帧) ==')
for h in (0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.7, 0.8):
    r = resid(np.ones(len(ts), bool), cam_down=h)
    print('   CAM_DOWN %.2f m: 中位 %6.1f  P90 %6.1f mm' % (h, r[0], r[1]))
