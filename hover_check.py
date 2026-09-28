#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
悬停测试分析: 不给任何指令时, 飞控自己能不能稳住.

关键区分:
  位置有【趋势(漂移)】  -> 定位(FAST-LIO/vision_pose)在飘  ✗
  位置无趋势但【抖动】  -> 控制/机体                    ✗
  两者都很小           -> 飞控没问题, 问题在上层        ✓

用法: python2 hover_check.py <bag>
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
T, X, Y, Z, YW = [], [], [], [], []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    R = q2R((q.x, q.y, q.z, q.w))
    T.append(t.to_sec()); X.append(p.x); Y.append(p.y); Z.append(p.z)
    YW.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
bag.close()
T = np.array(T); X = np.array(X); Y = np.array(Y); Z = np.array(Z)
YW = np.degrees(np.unwrap(np.radians(np.array(YW))))
print('总长 %.0fs   z %.2f~%.2f' % (T[-1] - T[0], Z.min(), Z.max()))

# 找悬停段: z 高且变化小, 取最长的一段
alt = np.percentile(Z, 90)
thr = 0.75 * alt
cand = Z > thr
best = (0, 0, 0)
i = 0
while i < len(cand):
    if cand[i]:
        j = i
        while j < len(cand) and cand[j]:
            j += 1
        if T[j - 1] - T[i] > best[0]:
            best = (T[j - 1] - T[i], i, j)
        i = j
    else:
        i += 1
dur, i0, i1 = best
if dur < 5:
    sys.exit('没找到 >5s 的悬停段 (z 最高才 %.2f)' % Z.max())
t = T[i0:i1] - T[i0]
x = X[i0:i1]; y = Y[i0:i1]; z = Z[i0:i1]; yw = YW[i0:i1]
print('悬停段: %.1fs  起点 z=%.2f' % (dur, z[0]))


def rpt(nm, v, dt):
    """同时给: 总波动(std), 趋势(线性斜率), 去趋势后的抖动"""
    A = np.column_stack([dt, np.ones(len(v))])
    s, _, _, _ = np.linalg.lstsq(A, v)
    resid = v - A.dot(s)
    print('  %-6s 总std %7.1f mm | 漂移 %+7.1f mm/s (共 %+7.0f mm) | 去趋势抖动 %6.1f mm | 范围 %+.0f~%+.0f' % (
        nm, 1000 * v.std(), 1000 * s[0], 1000 * s[0] * dur, 1000 * resid.std(),
        1000 * (v - v.mean()).min(), 1000 * (v - v.mean()).max()))
    return 1000 * s[0], 1000 * resid.std()


print()
print('=== ★ 地面静止段 (飞机根本没动, 任何读数变化都是雷达估计漂移) ===')
g = Z < 0.10
# 取最长的静止段
best2 = (0, 0, 0); i = 0
while i < len(g):
    if g[i]:
        j = i
        while j < len(g) and g[j]:
            j += 1
        if T[j-1]-T[i] > best2[0]:
            best2 = (T[j-1]-T[i], i, j)
        i = j
    else:
        i += 1
d2, a0, a1 = best2
if d2 < 5:
    print('  地面段太短 (%.1fs)' % d2)
else:
    tt = T[a0:a1] - T[a0]
    xx = X[a0:a1]; yy = Y[a0:a1]; zz = Z[a0:a1]; ww = YW[a0:a1]
    print('  静止 %.1fs' % d2)
    for nm, v in (('x', xx), ('y', yy), ('z', zz), ('yaw', ww)):
        A = np.column_stack([tt, np.ones(len(v))])
        sl, _, _, _ = np.linalg.lstsq(A, v)
        res = v - A.dot(sl)
        sc = 1.0 if nm == 'yaw' else 1000.0
        print('  %-4s 漂移 %+8.2f /s (共 %+8.1f) | 抖动 %7.2f | 总变化 %+8.1f' % (
            nm, sl[0]*sc, sl[0]*sc*d2, res.std()*sc, (v[-1]-v[0])*sc))
    print()
    print('  => 地面静止时若 x/y/yaw 还在明显变化, 那就是【雷达估计在漂】✗')
    print('     把 vision_pose 喂给飞控 => 飞控以为自己在动 => 真的去修正 => 飞机真的动 ✗')

print()
print('=== 悬停段 (不给任何指令, 时间 %.0f~%.0fs) ===' % (t[0], t[-1]))
rpt('x', x, t)
rpt('y', y, t)
rpt('z', z, t)
rpt('yaw', yw, t)
print()
print('  首尾位移: dx %+.0f mm  dy %+.0f mm' % (dx, 1000 * (y[-1] - y[0])))
d0 = np.hypot(x - x[0], y - y[0])
print('  离起点距离: 中位 %.0f mm  max %.0f mm' % (1000 * np.median(d0), 1000 * d0.max()))
print()
print('=== 判定 ===')
A = np.column_stack([t, np.ones(len(x))]); sxs, _, _, _ = np.linalg.lstsq(A, x)
A = np.column_stack([t, np.ones(len(y))]); sys_, _, _, _ = np.linalg.lstsq(A, y)
drift = 1000 * max(abs(sxs[0]), abs(sys_[0])) * dur
jit = 1000 * max((x - A.dot(np.linalg.lstsq(A, x)[0])).std(),
                 (y - A.dot(np.linalg.lstsq(A, y)[0])).std())
print('  水平: 漂移 %.0f mm / 抖动 %.0f mm  (悬停 %.0fs)' % (drift, jit, dur))
if drift < 50 and jit < 30:
    print('  => 飞控稳 ✓  问题在上层(路径/相机/控制律) ✓')
elif drift > jit * 2:
    print('  => ★定位在飘★ (趋势明显大于抖动) -> 查 FAST-LIO / vision_pose ✗')
else:
    print('  => ★飞控/机体控制不住★ (抖动为主) -> 查振动/重心/PX4 参数 ✗')

print()
print('=== 悬停段 x-y 轨迹 (相对起点, 每格 50mm) ===')
W, H = 61, 21
sx = max(np.ptp(x), 1e-6); sy = max(np.ptp(y), 1e-6)
g = [[' '] * W for _ in range(H)]
for k in range(len(x)):
    gx = int((x[k] - x.min()) / sx * (W - 1))
    gy = H - 1 - int((y[k] - y.min()) / sy * (H - 1))
    g[gy][gx] = '.'
g[H - 1 - int((y[0] - y.min()) / sy * (H - 1))][int((x[0] - x.min()) / sx * (W - 1))] = 'S'
for r in g:
    print('  |' + ''.join(r) + '|')
print('  x %.0f~%.0f mm (%.0f mm),  y %.0f~%.0f mm (%.0f mm),  S=起点' % (
    1000 * (x.min() - x[0]), 1000 * (x.max() - x[0]), 1000 * (x.max() - x.min()),
    1000 * (y.min() - y[0]), 1000 * (y.max() - y[0]), 1000 * (y.max() - y.min())))
