#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
标定「相机夹角 -> 航向」的系数 (用一趟带雷达真值的飞行数据).

几何动机: 画面里的槽倾角 tilt 里混着横向偏差的贡献, 两者量级相同:
    tan(tilt) = A*tan(psi) + B*(d/h) + C
  psi = 机头相对槽的航向误差,  d = 横向偏差(mm),  h = 高度
  理论值 A=sin(alpha), B=-cos(alpha); 但实测的 mask 中心线不等于槽真实中心线,
  所以 A,B 必须**实测标定**, 不能靠理论.

真值: psi_true = 轨迹主方向 - 机头 yaw   (飞机沿槽飞, 轨迹方向≈槽方向)

输出 ~/guttermap/ang_cal.json, 并做 2 折交叉验证(一半拟合一半检验), 报检验残差.

用法: python2 ang_cal.py <bag> <detect.csv> [模型名]
"""
import json
import os
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


OFF_MAX = 250.0      # 偏差物理上限(相机横向视野)
TILT_MAX = 45.0      # 夹角物理上限(超过必是坏 mask)
CONF_MIN = 0.5

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
print('轨迹主方向 %.1f deg  奇异值 %s  (第二奇异值越小=越直)' % (
    np.degrees(traf), np.round(s, 2)))

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
    if det != 1 or conf < CONF_MIN:
        continue
    if not (T[0] <= t <= T[-1]):
        continue
    if abs(off) > OFF_MAX or abs(ang) > TILT_MAX:
        continue
    A.append((t, off, ang))
A = np.array(A)
print('纳入标定的帧: %d' % len(A))
if len(A) < 80:
    sys.exit('样本太少')

h = np.interp(A[:, 0], T, P[:, 2])
yaw = np.interp(A[:, 0], T, Y)
psi_true = (traf - yaw + np.pi) % (2 * np.pi) - np.pi
tilt = np.radians(A[:, 2])
dh = (A[:, 1] / 1000.0) / np.maximum(h, 0.05)
Yv = np.tan(tilt)


def fit(idx):
    X = np.column_stack([np.tan(psi_true[idx]), dh[idx], np.ones(len(idx))])
    sol, _, _, _ = np.linalg.lstsq(X, Yv[idx])
    return sol


def apply_psi(sol, y, dhv):
    return np.arctan((y - sol[1] * dhv - sol[2]) / sol[0])


sol = fit(np.arange(len(A)))
print()
print('全量拟合: tan(tilt) = %.3f*tan(psi) + %.3f*(d/h) + %.3f' % tuple(sol))
print('  理论参考:           %.3f            %+.3f' % (
    np.sin(np.radians(43.6)), -np.cos(np.radians(43.6))))

# --- 2 折交叉验证 (按时间前后半分, 避免同段自相关) ---
n = len(A)
k = n // 2
e = []
for tr, te in ((np.arange(k), np.arange(k, n)), (np.arange(k, n), np.arange(k))):
    s2 = fit(tr)
    pr = apply_psi(s2, Yv[te], dh[te])
    e.append(np.degrees(pr - psi_true[te]))
e = np.concatenate(e)
print()
print('=== 交叉验证(一半拟合/一半检验) ===')
print('  航向误差: 中位 %.1f deg   P90 %.1f   std %.1f   max %.1f' % (
    np.median(np.abs(e)), np.percentile(np.abs(e), 90), e.std(), np.abs(e).max()))

# 对照: 不扣偏差项(只用夹角)
e0 = []
for tr, te in ((np.arange(k), np.arange(k, n)), (np.arange(k, n), np.arange(k))):
    X = np.column_stack([np.tan(psi_true[tr]), np.ones(len(tr))])
    s0, _, _, _ = np.linalg.lstsq(X, Yv[tr])
    pr = np.arctan((Yv[te] - s0[1]) / s0[0])
    e0.append(np.degrees(pr - psi_true[te]))
e0 = np.concatenate(e0)
print('  对照[只用夹角]: 中位 %.1f  P90 %.1f' % (
    np.median(np.abs(e0)), np.percentile(np.abs(e0), 90)))

out = {'A': float(sol[0]), 'B': float(sol[1]), 'C': float(sol[2]),
       'model': sys.argv[3] if len(sys.argv) > 3 else 'unknown',
       'n': int(len(A)), 'cv_med_deg': float(np.median(np.abs(e))),
       'cv_p90_deg': float(np.percentile(np.abs(e), 90)),
       'h_med': float(np.median(h))}
p = os.path.expanduser('~/guttermap/ang_cal.json')
json.dump(out, open(p, 'w'), indent=1)
print()
print('已存 %s' % p)
print('  => 运行时: psi = atan((tan(tilt) - B*(d/h) - C) / A)')
