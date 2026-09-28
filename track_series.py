#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把跟踪日志按时间打成序列: s / cte / herr / fix / cam, 看哪一段出了问题."""
import re
import sys
import numpy as np

F = sys.argv[1]
raw = open(F, 'rb').read().decode('utf-8', 'ignore').replace('\x00', '')
L = raw.split('\n')
rows = []          # (t, s, cte, herr, enabled)
fx = {}            # t -> (fix, cam)
for ln in L:
    m = re.search(r'\[(\d+\.\d+)\].*?s=\s*([\d.]+)/([\d.]+)\s+cte=([+-][\d.]+)mm\s+herr=\s*([+-][\d.]+)deg', ln)
    if m:
        rows.append((float(m.group(1)), float(m.group(2)), float(m.group(4)),
                     float(m.group(5)), 'vy=' in ln))
    m2 = re.search(r'\[(\d+\.\d+)\].*?\(idle\)\s+fix=([+-][\d.]+)mm\s+cam=([+-][\d.]+|--)', ln)
    if m2:
        fx[round(float(m2.group(1)), 2)] = (float(m2.group(2)), m2.group(3))
if not rows:
    sys.exit('没解析到 s/cte 行')
R = np.array(rows)
en = R[R[:, 4] == 1]
print('总行 %d   使能行 %d' % (len(R), len(en)))
if len(en) == 0:
    print('★ 没有使能的记录 —— 这趟没真正飞')
    sys.exit(0)
t0 = en[0, 0]
print('使能段 %.0fs   s %.2f -> %.2f   cte 中位 %+.0f   herr 中位 %+.1f' % (
    en[-1, 0] - t0, en[0, 1], en[-1, 1], np.median(en[:, 2]), np.median(en[:, 3])))
print()
print('%7s %6s %8s %7s   %8s %6s' % ('t(s)', 's', 'cte_mm', 'herr', 'fix_mm', 'cam'))
step = max(1, len(en) // 30)
for i in range(0, len(en), step):
    t, s, cte, herr, _ = en[i]
    # 找最近的 fix/cam
    k = min(fx.keys(), key=lambda z: abs(z - t)) if fx else None
    f = fx.get(k, ('', '')) if k is not None else ('', '')
    print('%7.1f %6.2f %8.0f %7.1f   %8s %6s' % (t - t0, s, cte, herr, f[0], f[1]))
print()
print('★ cte 分段统计:')
k = len(en) // 3
for tag, seg in (('前1/3', en[:k]), ('中1/3', en[k:2 * k]), ('后1/3', en[2 * k:])):
    print('   %s: cte 中位 %+.0f   |cte|>200mm 占比 %.0f%%   s 范围 %.2f~%.2f' % (
        tag, np.median(seg[:, 2]), 100.0 * np.mean(np.abs(seg[:, 2]) > 200),
        seg[:, 1].min(), seg[:, 1].max()))
