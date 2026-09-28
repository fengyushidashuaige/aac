#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看相机原始 off 有多噪: 帧间跳变 + 高频成分, 以及平滑后能降多少.
用法: python3 off_noise.py <detect.csv>
"""
import sys
import numpy as np

f = sys.argv[1]
rows = []
for i, ln in enumerate(open(f)):
    if i == 0:
        continue
    p = ln.strip().split(',')
    if len(p) < 7:
        continue
    try:
        t = float(p[0]); o = float(p[1]); cf = float(p[3]); de = int(p[4])
    except Exception:
        continue
    if de == 1 and cf >= 0.5:
        rows.append((t, o))
r = np.array(rows)
t = r[:, 0]; o = r[:, 1]
print('检出帧 n=%d   时长 %.0fs   (%.0f Hz)' % (len(o), t[-1] - t[0], len(o) / max(t[-1] - t[0], 1e-6)))
print()
print('off 原始值:  std %.0f mm   范围 %+.0f ~ %+.0f' % (o.std(), o.min(), o.max()))
d = np.diff(o)
print('帧间跳变 |d_off|:  中位 %.1f mm   P90 %.1f   max %.1f' % (
    np.median(np.abs(d)), np.percentile(np.abs(d), 90), np.abs(d).max()))
print()
print('  窗宽   平滑后std   帧间跳变中位   高频成分std')
for w in (3, 5, 7, 15, 31):
    k = np.ones(w) / w
    sm = np.convolve(o, k, mode='same')
    hf = o - sm
    print('  %3d   %8.0f   %10.1f   %10.0f' % (
        w, sm.std(), np.median(np.abs(np.diff(sm))), hf.std()))
print()
print('判读:')
print('  高频成分 std 大  -> 原始 off 噪声大, 伺服直接用它做闭环 = 在追噪声')
print('  平滑后 std 明显小 -> 平滑能救, 伺服该用平滑值(或自带滤波)')
