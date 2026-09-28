#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离槽距离报告 —— 这是"洗干净没有"的真指标（track_report.py 的 |cte| 只是"跟路径"）。

用法: python3 ~/guttermap/off_report.py [track.log路径] [detect.csv路径]
窗口自动从 track.log 的使能段起止时间取(rospy 时间戳 ≈ time.time())。
"""
import re, sys, os, glob, time

TR = sys.argv[1] if len(sys.argv) > 1 else '/home/amov/guttermap/run/track.log'
if len(sys.argv) > 2:
    DC = sys.argv[2]
else:
    cands = sorted(glob.glob('/home/amov/logs/guttermap_*/detect.csv'), key=os.path.getmtime)
    if not cands:
        print('找不到 detect.csv'); sys.exit(1)
    DC = cands[-1]

LINE = re.compile(r'\[(\d+\.\d+)\]: \[track\] s=\s*([0-9.]+)/[0-9.]+\s+cte=\s*([-+0-9.]+)mm')
t0 = t1 = None
for ln in open(TR, errors='replace'):
    m = LINE.search(ln)
    if not m:
        continue
    if abs(float(m.group(3))) > 5000:
        continue
    t = float(m.group(1))
    s = float(m.group(2))
    if t0 is None:
        t0, smax, t1 = t, s, t
    if s >= smax:                      # 只统计出航段: 跟踪到最大 s 的时刻为止
        smax, t1 = s, t
if t0 is None:
    print('track log 里没有使能样本'); sys.exit(1)


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else float('nan')


off = []
with open(DC) as f:
    f.readline()
    for ln in f:
        p = ln.strip().split(',')
        if len(p) < 8:
            continue
        t = float(p[0])
        if t < t0 or t > t1:
            continue
        if int(p[4]) == 1:
            off.append(float(p[1]))

print('track log : %s' % TR)
print('detect.csv: %s' % DC)
print('窗口      : %s ~ %s  (%.0f s)'
      % (time.strftime('%H:%M:%S', time.localtime(t0)),
         time.strftime('%H:%M:%S', time.localtime(t1)), t1 - t0))
if not off:
    print('窗口内无检出'); sys.exit(0)
a = sorted(abs(x) for x in off)
n = len(a)
print('检出 %d 帧' % n)
print('  |off| 中位 %.1fmm   P90 %.1fmm   最大 %.1fmm'
      % (a[n // 2], a[int(n * 0.9)], a[-1]))
print('  |off|<=25 %.1f%%   <=50 %.1f%%   <=100 %.1f%%'
      % (100.0 * sum(1 for x in a if x <= 25) / n,
         100.0 * sum(1 for x in a if x <= 50) / n,
         100.0 * sum(1 for x in a if x <= 100) / n))
print('  off 中位 %+.1fmm  均值 %+.1fmm   (零偏说明路径系统性偏一侧)'
      % (med(off), sum(off) / n))
