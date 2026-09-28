#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""槽线建图: detect.csv(相机的槽检测) + flight.bag(雷达位姿) -> 世界系槽中心线

用法: python2 build_polyline.py <bag> <detect.csv> [选项]
  --conf F     置信度下限 (默认 0.90)   ← 之前是 0.4
  --band F     wpx 相对滑动中位的允许带宽 (默认 0.10)  ← 新增
  --width MM   有效槽宽 mm (默认 88.0)  ← 之前硬编码 110
  --ax M       前向注视距离 m (默认 0.45)  ← 新增
  --dtpose S   位姿相对检测时间戳的偏移 s (默认 -0.24)  ← 新增
  --camdown M  观测点在机体下方距离 m, 数字或 auto

2026-09-22 修复 (auto_map_01 残差 75.6mm; 两趟飞行交叉验证后定型):
 ① 门控: 原先只按 conf, 而 conf 是模型分主导, 筛不掉"mask 被截断"的坏帧。
    坏帧 wpx 偏小 -> mm_per_px=width/wpx 爆表 -> offset 爆表(实测最大 1252mm,
    而 0.45m 高度下相机横向物理上限只有 ~315mm)。现在按 wpx 滑动中位的带宽卡。
 ② 尺度: 有效槽宽 ≈88mm 当量(两趟独立全局拟合 k=0.786 / 0.821)。外形量出来是
    ~110mm, 差 ~20% 说明模型 mask 覆盖的宽度不等于外形宽(大概只到内底/内壁)。
    原硬编码 110 使 offset 系统性偏大, 建图线被推离飞行轨迹形成蛇形。
 ③ 前向注视: 相机在机头前下方俯视, 观测点不在飞机正下方而在**前下方**,
    a = (a_x, 0, -h), a_x = h*cot(俯角) ≈ 0.45m (h≈0.43 -> 俯角≈44deg)。
    缺这项会让重建线随偏航"鼓成弓形"。第二趟单程飞行实测: 中位 33.6 -> 7.9mm。
    (第一趟是折返飞行, 与偏航耦合的误差来回抵消, 所以看不出这项 —— 别被它骗。)
 ④ 时延: 检测时间戳比真实成像时刻晚约 0.24s (录包时间比消息头还晚 36ms;
    配置里 offset_pred_latency_s=0.25 也印证)。位姿按 t+dtpose 取。
"""
import sys
import os
import numpy as np
import rosbag

ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
OPTS = {}
_k = None
for a in sys.argv[1:]:
    if a.startswith('--'):
        _k = a[2:]
        OPTS[_k] = True
    elif _k:
        OPTS[_k] = a
        _k = None

BAG = ARGS[0]
CSVF = ARGS[1]

CONF_MIN = float(OPTS.get('conf', 0.90))
AX = float(OPTS.get('ax', 0.45))                 # 前向注视距离(相机前下方俯视)
WPX_BAND = float(OPTS.get('band', 0.10))
TROUGH_MM = float(OPTS.get('width', 88.0))
DT_POSE = float(OPTS.get('dtpose', -0.24))
CAM_DOWN = OPTS.get('camdown', 'auto')           # 数字 或 'auto'(用每帧高度, 落到 z=0 地面)
AUTO_H = str(CAM_DOWN).lower() == 'auto'
CAM_DOWN = 0.55 if AUTO_H else float(CAM_DOWN)
OFF_MAX_MM = float(OPTS.get('offmax', 315.0))    # 物理上限: 0.7*高度
OFF_PX_MAX = float(OPTS.get('offpxmax', 260.0))  # 槽中心离画面中心的像素上限
WIN = int(OPTS.get('win', 80))                   # 滑动中位半窗(帧)


# --- 1) 位姿 ---
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
print('位姿 %d 点   %.1f ~ %.1f s' % (len(T), T[0], T[-1]))


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# --- 2) 检测 ---
raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
hdr = raw.decode('utf-8', 'ignore').splitlines()[0].split(',')
has_wpx = 'wpx' in hdr
rows = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 5:
        continue
    try:
        rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]),
                     float(p[6]) if has_wpx and len(p) > 6 else 0.0))
    except Exception:
        pass
print('检测 %d 帧%s' % (len(rows), '' if has_wpx else '  (!! 无 wpx 列, 门控会放宽)'))

# 候选: 有检出 + 在飞行时段
cand = [r for r in rows if r[4] == 1 and T[0] <= r[0] <= T[-1] and r[3] >= CONF_MIN]
print('  检出且 conf>=%.2f 且在飞行时段: %d 帧' % (CONF_MIN, len(cand)))
if len(cand) < 10:
    print('!! 太少, 退出'); sys.exit(0)

ts = np.array([r[0] for r in cand])
off_mm = np.array([r[1] for r in cand])
conf = np.array([r[3] for r in cand])
wpx = np.array([r[5] for r in cand])
ang = np.array([r[2] for r in cand])


def roll_med(v, win):
    """居中滑动中位 (NaN 安全)"""
    n = len(v)
    out = np.empty(n)
    for i in range(n):
        seg = v[max(0, i - win):min(n, i + win + 1)]
        seg = seg[~np.isnan(seg)]
        out[i] = np.median(seg) if len(seg) else np.nan
    return out


# --- 3) 门控: wpx 滑动中位带宽 ---
keep = np.ones(len(cand), dtype=bool)
if has_wpx and WPX_BAND < 1.0:
    wref = roll_med(wpx, WIN)
    keep = np.abs(wpx - wref) <= WPX_BAND * wref
    # 二次: 用留下来的帧重算参考
    if keep.sum() >= 20:
        wref2 = roll_med(np.where(keep, wpx, np.nan), WIN)
        bad = np.isnan(wref2)
        if bad.any():
            wref2[bad] = np.median(wpx[keep])
        keep = np.abs(wpx - wref2) <= WPX_BAND * wref2

n0 = len(cand)
keep &= np.abs(off_mm) <= OFF_MAX_MM
# 保险门: 槽中心不能离画面中心太远. 飞偏时水槽会切到画面外 -> mask 被截断 ->
# wpx 和 offset 同时失真(用户反馈的正是这种情况). 还原检测器的像素偏移来卡.
if has_wpx:
    off_px_est = off_mm * wpx / 110.0
    keep &= np.abs(off_px_est) <= OFF_PX_MAX
n1 = int(keep.sum())
print('  门控后剩 %d 帧 (%.0f%%)' % (n1, 100.0 * n1 / max(n0, 1)))
if (~keep).any():
    print('  丢了 %d 帧, 它们的 |off| 中位 %.0f mm (物理上限 %.0f mm)' % (
        int((~keep).sum()), np.median(np.abs(off_mm[~keep])), OFF_MAX_MM))

if keep.sum() < 10:
    print('!! 门控后太少, 退出'); sys.exit(0)

# --- 4) 尺度 ---
# 检测器给的 off_mm = offset_px * (110/wpx)。要还原成 offset_px * (TROUGH_MM/wpx_ref):
#   offset_px = off_mm * wpx/110
#   off_fix   = offset_px * TROUGH_MM/wpx_ref
# wpx_ref 用滑动中位 -> 即使个别帧 wpx 离谱, 尺度也不会爆.
if has_wpx and WPX_BAND < 1.0:
    wref = roll_med(wpx, WIN)                      # 因果/居中都可, 这里居中
    wref = np.maximum(wref, 1e-6)
else:
    wref = np.full(len(cand), 110.0)
off_fix = off_mm * (wpx / 110.0) * (TROUGH_MM / wref)
print('  尺度: 槽宽 %.1fmm, wpx 参考中位 %.1f -> mm/px 中位 %.4f (原 %.4f)' % (
    TROUGH_MM, np.median(wref[keep]), np.median(TROUGH_MM / wref[keep]),
    np.median(110.0 / np.maximum(wpx[keep], 1e-6))))

ts_k = ts[keep]
off_k = (off_fix[keep] / 1000.0)
print('  最终用 %d 帧, |off| 中位 %.0fmm, P90 %.0fmm' % (
    len(ts_k), 1000 * np.median(np.abs(off_k)), 1000 * np.percentile(np.abs(off_k), 90)))

# --- 5) 投影 ---
W = []
for t_i, o in zip(ts_k, off_k):
    tq = np.clip(t_i + DT_POSE, T[0], T[-1])
    i = int(np.clip(np.searchsorted(T, tq), 1, len(T) - 1))
    a = (tq - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
    q = Q[i - 1] + a * (Q[i] - Q[i - 1])
    R = quat_to_R(q)
    p = P[i - 1] + a * (P[i] - P[i - 1])
    # auto: 观测点在地面(z=0)上, 用每帧高度; 否则用固定值
    hz = p[2] if AUTO_H else CAM_DOWN
    p_b = np.array([AX, -o, -hz])
    W.append(R.dot(p_b) + p)
W = np.array(W)
print('投影出 %d 个世界系点' % len(W))
if len(W) < 10:
    print('!! 点太少'); sys.exit(0)

print()
print('=== 世界系槽点 ===')
for i, ax in enumerate('xyz'):
    print('  %s 范围 [%+.2f, %+.2f]  跨度 %.2f m' % (
        ax, W[:, i].min(), W[:, i].max(), W[:, i].max() - W[:, i].min()))
print('  路径长度 %.2f m' % np.linalg.norm(np.diff(W, axis=0), axis=1).sum())

c = W.mean(axis=0)
u, s, vt = np.linalg.svd(W - c, full_matrices=False)
d = vt[0]
proj = (W - c).dot(d)
perp = (W - c) - np.outer(proj, d)
pn = np.linalg.norm(perp, axis=1)
print()
print('=== 直线度 (最优拟合直线) ===')
print('  方向 [%+.3f, %+.3f, %+.3f]   奇异值 %s' % (d[0], d[1], d[2], np.round(s, 2)))
print('  3D 垂直残差: 中位 %.1f mm   P90 %.1f mm   max %.1f mm' % (
    1000 * np.median(pn), 1000 * np.percentile(pn, 90), 1000 * pn.max()))

# ★ 俯视图真正关心的: xy 平面内的横向残差 (高度起伏不应计入)
Wh = W[:, :2]
ch = Wh.mean(axis=0)
uh, sh, vth = np.linalg.svd(Wh - ch, full_matrices=False)
dh = vth[0]
perph = np.abs((Wh - ch).dot(np.array([-dh[1], dh[0]])))
print('  xy 平面残差: 中位 %.1f mm   P90 %.1f mm   max %.1f mm   <-- 俯视图精度' % (
    1000 * np.median(perph), 1000 * np.percentile(perph, 90), 1000 * perph.max()))
print('  (中位 <20mm 为达标; 高度起伏只影响 3D 残差, 不影响俯视图)')

# --- 6) 俯视图 ---
print()
print('=== 俯视图 (x-y 平面, 上=+y) ===')
Wp, Ht = 58, 20
x0, x1 = W[:, 0].min(), W[:, 0].max(); y0, y1 = W[:, 1].min(), W[:, 1].max()
sx = max(x1 - x0, 1e-6); sy = max(y1 - y0, 1e-6)
g = [[' '] * Wp for _ in range(Ht)]
for p in W:
    gx = int((p[0] - x0) / sx * (Wp - 1)); gy = Ht - 1 - int((p[1] - y0) / sy * (Ht - 1))
    if 0 <= gy < Ht and 0 <= gx < Wp:
        g[gy][gx] = 'o'
print('  x %.2f -> %.2f m (格 %.3f m)' % (x0, x1, sx / (Wp - 1)))
for r in g:
    print('  |' + ''.join(r) + '|')
print('  y %.2f -> %.2f m (格 %.3f m)' % (y0, y1, sy / (Ht - 1)))

out = os.path.join(os.path.dirname(os.path.abspath(CSVF)), 'polyline.csv')
np.savetxt(out, W, delimiter=',', header='x,y,z', comments='', fmt='%.4f')
print()
print('结果已存 %s' % out)
print('画图: python2 make_map.py %s %s' % (BAG, out))
