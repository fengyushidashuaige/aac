#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线回放验证"慢相机修正": 用飞行日志里的 (cte, cam) 时序, 模拟新的修正逻辑,
看它会不会收敛、要多久、会不会跑飞. 不用飞.

用法: python3 replay_trim.py <track.log>
"""
import re
import sys
import math
import numpy as np

F = sys.argv[1]
raw = open(F, 'rb').read().decode('utf-8', 'ignore').replace('\x00', '')
lines = raw.split('\n')

# 配对: 同一次循环打两行(第一行含 cte, 第二行含 cam), 按时间就近配
S = []          # (t, cte_raw, cam)
cur = None
pat1 = re.compile(r'\[(\d+\.\d+)\].*?s=\s*([\d.]+)/[\d.]+\s+cte=([+-][\d.]+)mm.*?(vy=|\()')
for ln in lines:
    m = re.search(r'\[(\d+\.\d+)\].*?s=\s*([\d.]+)/[\d.]+\s+cte=([+-][\d.]+)mm', ln)
    if m and 'vy=' in ln:
        # 第一行(使能行): 记下 cte
        cur = [float(m.group(1)), float(m.group(3)), None]
        # 同一行末尾也可能有 cam
        mc = re.search(r'cam=([+-][\d.]+)', ln)
        if mc:
            cur[2] = float(mc.group(1))
            S.append(tuple(cur)); cur = None
        continue
    m2 = re.search(r'\[(\d+\.\d+)\].*?idle.*?cam=([+-][\d.]+)', ln)
    if m2 and cur is not None and abs(float(m2.group(1)) - cur[0]) < 2.0:
        cur[2] = float(m2.group(2))
        S.append(tuple(cur)); cur = None

if len(S) < 30:
    print('可用样本太少 (%d) —— 这个日志里 cte 和 cam 没成对出现' % len(S))
    print('（回放需要"跟路径飞"那段日志, 且同一时刻有 cte 和 cam）')
    sys.exit(0)
S = np.array(S)
T = S[:, 0] - S[:, 0][0]
cte = S[:, 1] / 1000.0
off = S[:, 2] / 1000.0
print('配对样本 %d  时长 %.0fs  采样 %.1f Hz' % (len(S), T[-1], len(S) / max(T[-1], 1e-6)))
print('  cte 中位 %+.0f mm    cam 中位 %+.0f mm' % (1000 * np.median(cte), 1000 * np.median(off)))


# ★ 插值到控制频率 20Hz (修正环是按 20Hz 跑的, 不是按日志的 1Hz)
ti = np.arange(T[0], T[-1], 0.05)
cte = np.interp(ti, T, cte); off = np.interp(ti, T, off); T = ti
print('插值到 20Hz: %d 点' % len(T))


def sim(TAU, CAP, LIM, skip_bad=True):
    off_lp = cte_lp = lat = 0.0
    hist = []
    for i in range(len(T)):
        dt = max(T[i] - T[i - 1], 1e-3) if i else 0.05
        if skip_bad and abs(off[i]) > 0.250:
            hist.append(lat); continue
        a = 1.0 - math.exp(-dt / TAU)
        off_lp += a * (off[i] - off_lp)
        cte_lp += a * (cte[i] - cte_lp)
        tgt = cte_lp - off_lp
        d = a * (tgt - lat)
        c = CAP * dt
        lat += max(-c, min(c, d))
        lat = max(-LIM, min(LIM, lat))
        hist.append(lat)
    return np.array(hist)


for TAU, CAP, LIM, tag in ((2.5, 0.10, 0.8, '旧(τ2.5s/100mm/s/±0.8)'),
                           (15.0, 0.01, 0.5, '新(τ15s/10mm/s/±0.5)'),
                           (30.0, 0.01, 0.5, '更慢(τ30s/10mm/s/±0.5)')):
    h = sim(TAU, CAP, LIM)
    dif = np.abs(np.diff(h))
    print()
    print('=== %s ===' % tag)
    print('  修正量 lat_shift: 终值 %+.0f mm   最大 %+.0f   最小 %+.0f' % (
        1000 * h[-1], 1000 * h.max(), 1000 * h.min()))
    print('  变化率 max %.1f mm/s (限速 %.0f)' % (1000 * dif.max() / 0.05, 1000 * CAP))
    # 收敛: 后 1/3 的平均变化量
    k = len(h) // 3
    print('  后1/3 段平均每样本变化 %.2f mm  (%s)' % (
        1000 * np.abs(np.diff(h[-k:])).mean(),
        '已收敛' if np.abs(np.diff(h[-k:])).mean() < 0.0005 else '还在动'))
    print('  序列(每 10%%取1): ' + ' '.join('%+.0f' % (1000 * h[i]) for i in range(0, len(h), max(1, len(h) // 10))))
