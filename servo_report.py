#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
伺服飞行数据自动分析: 把 run/servo.log 算成客观指标, 并画出 off 波形(ASCII).
用法: python3 ~/guttermap/servo_report.py [日志路径]
"""
import re
import sys
import os
import numpy as np

F = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/guttermap/run/servo.log')
T, O, VY, VX, YA = [], [], [], [], []
raw = open(F, 'rb').read().decode('utf-8', 'ignore').replace('\x00', '')
for line in raw.split('\n'):
    m = re.search(r'\[(\d+\.\d+)\].*?servo\] off=\s*([+-][\d.]+)mm.*?vx=([\d.-]+)\s+vy=([+-][\d.]+)\s+yaw=([+-][\d.]+)', line)
    if m:
        T.append(float(m.group(1))); O.append(float(m.group(2)))
        VX.append(float(m.group(3))); VY.append(float(m.group(4))); YA.append(float(m.group(5)))
if len(T) < 20:
    sys.exit('样本太少 (%d) —— 日志是空的? 确认路径: %s' % (len(T), F))
T = np.array(T) - T[0]; O = np.array(O); VY = np.array(VY); VX = np.array(VX); YA = np.array(YA)
dt = np.median(np.diff(T))

print('=' * 62)
print('伺服飞行分析   样本 %d   时长 %.1fs   采样 %.0f Hz (间隔 %.3fs)' % (len(T), T[-1], 1.0 / max(dt, 1e-6), dt))
print('=' * 62)
print()
print('【1】偏差 off')
print('   中位 %+.0f   std %.0f   min %+.0f   max %+.0f' % (np.median(O), O.std(), O.min(), O.max()))
for lim in (25, 50, 100, 200):
    print('   |off| <= %3dmm 的时间占比: %5.1f%%' % (lim, 100.0 * np.mean(np.abs(O) <= lim)))
print('   ★ "锁住"的判据: |off|<=50mm 占比越高越好')
print()
print('【2】横移指令 vy')
mx = np.percentile(np.abs(VY), 99)
print('   中位 %+.3f  std %.3f  min %+.3f  max %+.3f' % (np.median(VY), VY.std(), VY.min(), VY.max()))
print('   ★ 打到上限(取 |vy|>0.9*max)的时间占比: %.1f%%   <- 超过 10%% 说明增益还是大' % (
    100.0 * np.mean(np.abs(VY) > 0.9 * max(np.abs(VY).max(), 1e-6))))
print()
print('【3】振荡特性')
sgn = np.sign(O)
zc = np.where(np.diff(sgn) != 0)[0]
if len(zc) >= 4:
    per = np.diff(T[zc])
    print('   过零 %d 次   周期(中位) %.2f s   相当于 %.2f Hz' % (
        len(zc), np.median(per), 1.0 / max(np.median(per), 1e-6)))
    seg = np.abs(O)
    print('   幅值(整体 std) %.0f mm' % O.std())
    # 最近 1/3 段的震荡幅度 vs 前 1/3 —— 看有没有收敛
    k = len(O) // 3
    print('   前 1/3 段 std %.0f mm   ->   后 1/3 段 std %.0f mm   (%s)' % (
        O[:k].std(), O[-k:].std(),
        '在收敛 ↓' if O[-k:].std() < O[:k].std() * 0.8 else ('基本不变 →' if O[-k:].std() < O[:k].std() * 1.2 else '在发散 ↑')))
else:
    print('   过零太少(%d) —— 没有明显振荡 ✓' % len(zc))
print()
print('【4】off 波形 (ASCII, 每格 %.1fs, 竖线=0, ±200mm 满格)' % max(dt * max(1, len(O) // 100), dt))
n = len(O)
step = max(1, n // 100)
W = 100
rows = 9
grid = [[' '] * W for _ in range(rows)]
for j, i in enumerate(range(0, n, step)):
    if j >= W:
        break
    v = int(np.clip((O[i] / 200.0 + 1) * (rows - 1) / 2, 0, rows - 1))
    grid[rows - 1 - v][j] = '*'
for r in range(rows):
    lab = '%+4d' % int(200 * (1 - 2 * r / (rows - 1)))
    print('   %s |%s|' % (lab, ''.join(grid[r])))
print('        ' + ' ' * 0 + '+' + '-' * W + '+')
print('        0s' + ' ' * (W - 10) + '%.0fs' % T[-1])
print()
print('【5】其它')
print('   vx 中位 %.3f (巡航)   yaw 中位 %+.3f  max|yaw| %.3f' % (
    np.median(VX), np.median(YA), np.abs(YA).max()))
