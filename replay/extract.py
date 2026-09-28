#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 track log 抽成时序数据集, 并诊断摆动出自哪个回路。
用法: python3 extract.py <track.log> [...]
输出: <log>.npz 字段 t,s,cte,herr,vy,yaw   + 摆动诊断
"""
import re, sys, os, math
import numpy as np

LINE = re.compile(r'\[(\d+\.\d+)\]: \[track\] s=\s*([0-9.]+)/([0-9.]+)\s+cte=\s*([-+0-9.]+)mm\s+'
                  r'herr=\s*([-+0-9.]+)deg\s+vx=([0-9.]+)\s+vy=([-+0-9.]+)\s+yaw=([-+0-9.]+)')

def load(p):
    rows = []
    for ln in open(p, errors='replace'):
        m = LINE.search(ln)
        if not m: continue
        cte = float(m.group(4))
        if abs(cte) > 5000: continue
        rows.append([float(m.group(1)), float(m.group(2)), cte,
                     math.radians(float(m.group(5))), float(m.group(6)), float(m.group(7))])
    a = np.array(rows)
    if len(a) < 20: return None
    imax = int(np.argmax(a[:, 1]))
    return a[:imax+1]                     # 出航段

def zc_period(x, dt):
    """过零法估周期(秒)"""
    x = x - x.mean()
    idx = [i for i in range(1, len(x)) if x[i-1] < 0 <= x[i]]
    if len(idx) < 3: return float('nan')
    return 2.0 * dt * (idx[-1] - idx[0]) / float(len(idx) - 1)

def diagnose(tag, a):
    t, s, cte, herr, vy, yaw = [a[:, i] for i in range(6)]
    dt = float(np.median(np.diff(t)))
    # 去趋势后看摆动 (摆动 = 原始 - 低频趋势)
    # 去趋势窗口要 >> 摆动周期(实测~23s=160帧), 用 400 帧; 够长就退化为只去均值
    def detrend(x, w=400):
        if len(x) < w: return x - x.mean()
        k = np.ones(w) / w
        return x - np.convolve(x, k, mode='same')
    cd, hd, vd, yd = detrend(cte), detrend(herr), detrend(vy), detrend(yaw)
    print('== %s  n=%d  %.0fs (dt=%.2fs, %.1fHz)' % (tag, len(t), t[-1]-t[0], dt, 1.0/dt))
    print('   cte   std %6.1fmm  峰峰 %6.1f  周期 %5.2fs' % (cd.std(), cd.max()-cd.min(), zc_period(cte, dt)))
    print('   herr  std %6.2f°   峰峰 %6.2f  周期 %5.2fs' % (hd.std(), hd.max()-hd.min(), zc_period(herr, dt)))
    print('   vy    std %6.3f    峰峰 %6.3f  饱和占比 %.0f%% (>0.149)' % (
          vd.std(), vd.max()-vd.min(), 100.0*np.mean(np.abs(vy) > 0.149)))
    print('   yaw   std %6.3f    峰峰 %6.3f  饱和占比 %.0f%% (>0.399)' % (
          yd.std(), yd.max()-yd.min(), 100.0*np.mean(np.abs(yaw) > 0.399)))
    # 相位: cte 与 herr / vy / yaw 的互相关峰值位置
    def best_lag(x, y, maxlag=25):
        x = x - x.mean(); y = y - y.mean()
        if x.std() < 1e-9 or y.std() < 1e-9: return 0, 0.0
        c = [np.corrcoef(x[maxlag:len(x)-maxlag], np.roll(y, L)[maxlag:len(x)-maxlag])[0,1]
             for L in range(-maxlag, maxlag+1)]
        k = int(np.argmax(np.abs(c))) - maxlag
        return k, c[int(np.argmax(np.abs(c)))]
    for name, sig in [('herr', herr), ('vy', vy), ('yaw', yaw)]:
        k, r = best_lag(cte, sig)
        print('   cte vs %-4s  最佳滞后 %+3d 帧 = %+5.2fs   相关 %+.2f   %s' % (
              name, k, k*dt, r,
              '同相(该量在推 cte)' if abs(k) <= 2 else ('滞后' if k > 0 else '超前')))
    print()

for p in sys.argv[1:]:
    a = load(p)
    if a is None:
        print('== %s  样本太少, 跳过\n' % os.path.basename(p)); continue
    diagnose(os.path.basename(p).replace('track_', '').replace('.log', ''),
             a)
    np.savez(p.replace('.log', '') + '.npz',
             t=a[:,0], s=a[:,1], cte=a[:,2], herr=a[:,3], vy=a[:,4], yaw=a[:,5])
