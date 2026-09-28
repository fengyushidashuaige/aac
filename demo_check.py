#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看使能段到底发生了什么. 用法: python3 demo_check.py <track.log>"""
import re
import sys
import numpy as np

F = sys.argv[1]
L = open(F, 'rb').read().decode('utf-8', 'ignore').replace('\x00', '').split('\n')
pat = re.compile(r's=\s*([\d.]+)/([\d.]+)\s+cte=([+-][\d.]+)mm\s+herr=\s*([+-][\d.]+)deg'
                 r'.*?vx=([\d.-]+)\s+vy=([+-][\d.]+)\s+yaw=([+-][\d.]+)\s+fix=([+-][\d.]+)mm.*?cam=([+-][\d.]+|--)')
E = []
for l in L:
    if 'vy=' not in l:
        continue
    m = pat.search(l)
    if not m:
        continue
    g = m.groups()
    cam = None if g[8].startswith('-') and len(g[8]) <= 2 else float(g[8])
    E.append([float(g[0]), float(g[1]), float(g[2]), float(g[3]),
              float(g[4]), float(g[5]), float(g[6]), float(g[7]), cam])
print('使能行 %d' % len(E))
if len(E) < 5:
    sys.exit('太少')
E = np.array([[x if x is not None else np.nan for x in r] for r in E])
t = E[:, 0] - E[0, 0]
print('时长 %.0fs    s: %.2f -> %.2f (max %.2f)   路径长 %.2f' % (
    t[-1], E[0, 1], E[-1, 1], np.nanmax(E[:, 1]), E[0, 2]))
print('cte : 中位 %+.0f   min %+.0f   max %+.0f' % (
    np.median(E[:, 3]), np.nanmin(E[:, 3]), np.nanmax(E[:, 3])))
print('vx  : 中位 %.3f    vy: 中位 %+.3f (%)' % (np.median(E[:, 5]), np.median(E[:, 6])))
print('fix : 起 %+.0f -> 终 %+.0f' % (E[0, 7], E[-1, 7]))
cam = E[:, 8][~np.isnan(E[:, 8])]
if len(cam):
    print('cam : n=%d  中位 %+.0f  P10 %+.0f  P90 %+.0f  |cam|<=50 占 %.0f%%  <=100 占 %.0f%%' % (
        len(cam), np.median(cam), np.percentile(cam, 10), np.percentile(cam, 90),
        100.0 * np.mean(np.abs(cam) <= 50), 100.0 * np.mean(np.abs(cam) <= 100)))
print()
print(' %7s %6s %8s %7s %7s %7s %8s %7s' % ('t', 's', 'cte', 'herr', 'vx', 'vy', 'fix', 'cam'))
for k in list(range(0, min(15, len(E)))) + list(range(max(15, len(E) - 15), len(E))):
    r = E[k]
    print(' %7.1f %6.2f %8.0f %7.1f %7.3f %+7.3f %8.0f %7s' % (
        t[k], r[1], r[3], r[4], r[5], r[6], r[7], ('%.0f' % r[8]) if not np.isnan(r[8]) else '--'))
