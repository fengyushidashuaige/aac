#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
路径清洗: 去掉重建路径里的"假拐点"(局部方向跳变), 这类噪声不会拉高中位残差,
但会让纯追踪把飞机带出槽.

  ① 中值滤波(去尖刺)  ② 滑动平均(抹折角)  ③ 输出方向剖面供核对

用法: python2 clean_path.py <in.csv> [out.csv] [中值窗=15] [平均窗=9]
"""
import sys
import numpy as np

IN = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else IN.replace('.csv', '_clean.csv')
WM = int(sys.argv[3]) if len(sys.argv) > 3 else 15     # 中值窗(点数)
WA = int(sys.argv[4]) if len(sys.argv) > 4 else 9      # 平均窗(点数)

W = np.loadtxt(IN, delimiter=',', skiprows=1)
if W.ndim == 1:
    W = W.reshape(1, -1)
P0 = W[:, :2].copy()
n = len(W)


def medfilt(v, k):
    if k < 3 or k % 2 == 0:
        return v
    h = k // 2
    out = np.empty_like(v)
    for i in range(len(v)):
        a, b = max(0, i - h), min(len(v), i + h + 1)
        out[i] = np.median(v[a:b])
    return out


def movavg(v, k, passes=2):
    if k < 2:
        return v
    h = k // 2
    for _ in range(passes):
        out = np.empty_like(v)
        for i in range(len(v)):
            a, b = max(0, i - h), min(len(v), i + h + 1)
            out[i] = v[a:b].mean()
        v = out
    return v


# ③ 先处理"折返": 沿主方向投影, 一旦明显倒退就在那里截断
c0 = W[:, :2].mean(axis=0)
_u, _sv, _vt = np.linalg.svd(W[:, :2] - c0, full_matrices=False)
_d = _vt[0]
_t = (W[:, :2] - c0).dot(_d)
_rmax = np.maximum.accumulate(_t)
_bad = _t < _rmax - 0.05            # 倒退超过 50mm 就认为开始折返
if _bad.any():
    k = int(np.argmax(_bad))
    print('  ★ 检测到折返: 在第 %d/%d 点(弧长约 %.2fm)处截断, 后面 %.2fm 丢弃' % (
        k, n, np.sum(np.linalg.norm(np.diff(W[:k+1, :2], axis=0), axis=1)),
        np.sum(np.linalg.norm(np.diff(W[k:, :2], axis=0), axis=1))))
    W = W[:k + 1].copy()
    P0 = W[:, :2].copy()
    n = len(W)
else:
    print('  (没有检测到折返)')

X = W[:, 0].copy()
Y = W[:, 1].copy()
X = medfilt(X, WM)          # ① 去尖刺
Y = medfilt(Y, WM)
X = movavg(X, WA)           # ② 抹折角
Y = movavg(Y, WA)
W2 = W.copy()
W2[:, 0] = X
W2[:, 1] = Y


def dirprof(P, tag):
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    if s[-1] < 1e-6:
        print('%s: 长度 0' % tag)
        return
    worst = (0, 0)
    vals = []
    for i in range(0, len(P) - 16, 4):
        d = P[i + 15] - P[i]
        if np.linalg.norm(d) < 1e-4:
            continue
        vals.append(np.degrees(np.arctan2(d[1], d[0])))
    if not vals:
        return
    vals = np.array(vals)
    # 相邻(每 4 点)方向变化的最大值 —— 拐点会给一个尖峰
    dv = np.abs(np.diff(vals))
    print('  %-10s 弧长 %.2fm   相邻方向跳变: 中位 %.1f°  P90 %.1f°  **max %.1f°**' % (
        tag, s[-1], np.median(dv), np.percentile(dv, 90), dv.max()))
    return dv.max()


print('点数 %d  中值窗 %d  平均窗 %d' % (n, WM, WA))
d0 = dirprof(P0, '清洗前')
d1 = dirprof(W2[:, :2], '清洗后')
print()
print('  位移量: 中位 %.1f mm  max %.1f mm' % (
    1000 * np.median(np.linalg.norm(W2[:, :2] - P0, axis=1)),
    1000 * np.linalg.norm(W2[:, :2] - P0, axis=1).max()))

# 残差
for tag, P in (('清洗前', P0), ('清洗后', W2[:, :2])):
    c = P.mean(axis=0)
    u, sv, vt = np.linalg.svd(P - c, full_matrices=False)
    d = vt[0]
    perp = np.abs((P - c).dot(np.array([-d[1], d[0]])))
    print('  %s: 直线残差 中位 %.1f  P90 %.1f  max %.1f mm' % (
        tag, 1000 * np.median(perp), 1000 * np.percentile(perp, 90), 1000 * perp.max()))

np.savetxt(OUT, W2, delimiter=',', header='x,y,z', comments='', fmt='%.4f')
print()
print('已存 %s' % OUT)
if d0 and d1:
    print('方向跳变: %.1f° -> %.1f°' % (d0, d1))
