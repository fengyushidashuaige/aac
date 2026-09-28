#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 track.log 的 cte(飞机相对路径) 和 detect.csv 的 off_mm(飞机相对真实水槽)
按时间对齐. 两者之差 = 路径相对水槽中心的横向偏差.

  off_mm  : 相机看到的槽中心相对机身的位置 (+ = 槽在右侧)
  cte     : 飞机相对路径的横向偏差 (+ = 飞机在路径左侧)
  若飞机正压在槽上, 则 off≈0, 此时 cte 就暴露了路径本身的偏差.
"""
import re
import sys
import numpy as np

LOG = '/home/amov/guttermap/run/track.log'
CSV = '/home/amov/logs/guttermap_20260922_112200/detect.csv'

# --- 解析日志 ---
pat = re.compile(r'\[(\d+\.\d+)\]: \[track\] (\(未使用\))? ?s=\s*([\d.]+)/([\d.]+)\s+'
                 r'cte=([+-][\d.]+)mm\s+herr=\s*([+-][\d.]+)deg')
L = []
raw = open(LOG, 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore')
for ln in raw.splitlines():
    m = pat.search(ln)
    if m:
        L.append((float(m.group(1)), 1 if m.group(2) else 0, float(m.group(3)),
                  float(m.group(5)) / 1000.0, float(m.group(6))))
L = np.array(L)
print('日志 %d 条 (t %.0f ~ %.0f)' % (len(L), L[0, 0], L[-1, 0]))
print('  未使能 %d 条, 已使能 %d 条' % (int(L[:, 1].sum() == 0), int(L[:, 1].sum())))

# --- 解析检测 ---
D = []
for i, ln in enumerate(open(CSV)):
    if i == 0:
        continue
    p = ln.strip().split(',')
    if len(p) < 7:
        continue
    try:
        D.append((float(p[0]), float(p[1]) / 1000.0, float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
D = np.array(D)
print('检测 %d 帧 (t %.0f ~ %.0f)' % (len(D), D[0, 0], D[-1, 0]))

# --- 按 2 秒分箱对齐 ---
t0 = max(L[0, 0], D[0, 0])
t1 = min(L[-1, 0], D[-1, 0])
print()
print('=== 每 2 秒: 日志 cte vs 相机 off ===')
print('  %8s %4s %9s %9s %9s %9s %7s' % ('t', '使能', '日志s', 'cte_mm', 'off_mm', '检出率', 'wpx'))
print('  (%8s %4s %9s %9s %9s %9s %7s)' % ('相对起点', '', '', '飞机↔路径', '飞机↔槽', '', ''))
bins = []
x = t0
while x < t1:
    lm = (L[:, 0] >= x) & (L[:, 0] < x + 2)
    dm = (D[:, 0] >= x) & (D[:, 0] < x + 2)
    if lm.sum() and dm.sum():
        cte = np.median(L[lm, 3])
        en = int(np.median(L[lm, 1]))
        s = np.median(L[lm, 2])
        dd = D[dm]
        det = dd[dd[:, 3] == 1]
        offm = np.median(det[:, 1]) if len(det) else float('nan')
        rate = 100.0 * len(det) / len(dd)
        wpx = np.median(det[:, 4]) if len(det) else float('nan')
        bias = (cte + offm) * 1000 if len(det) else float('nan')
        bins.append((x - t0, en, s, cte * 1000, offm * 1000 if len(det) else float('nan'),
                     rate, wpx, bias))
    x += 2

for b in bins:
    print('  %8.0f %4s %9.2f %9.0f %9.0f %8.0f%% %7.0f' % (
        b[0], '是' if b[1] else '否', b[2], b[3],
        b[4] if not np.isnan(b[4]) else -9999, b[5], b[6] if not np.isnan(b[6]) else 0))

# --- 只在"飞机确实压在槽上"的时段算路径偏差 ---
print()
print('=== 用"相机看到槽居中"的帧估路径偏差 ===')
okk = []
for b in bins:
    if not np.isnan(b[4]) and abs(b[4]) < 150 and not b[1]:
        okk.append(b[7])
if okk:
    print('  未使能 且 |off|<150mm 的时段 %d 个: 路径偏差 (cte+off) 中位 %+.0f mm  P10 %+.0f  P90 %+.0f' % (
        len(okk), np.median(okk), np.percentile(okk, 10), np.percentile(okk, 90)))
    print('  -> 这个值就是路径相对水槽中心的横向偏差(正=路径偏向飞机左侧)')
else:
    print('  没有满足条件的时段')

# 全局: 用所有检出帧 + 插值 cte
print()
print('=== 全部检出帧 (插值 cte) ===')
ti = L[:, 0]
sgn = []
for d in D[D[:, 3] == 1]:
    k = np.argmin(np.abs(ti - d[0]))
    if abs(ti[k] - d[0]) < 1.5:
        sgn.append((L[k, 3] + d[1]) * 1000)
sgn = np.array(sgn)
if len(sgn):
    print('  n=%d  路径偏差 中位 %+.0f mm  P10 %+.0f  P90 %+.0f' % (
        len(sgn), np.median(sgn), np.percentile(sgn, 10), np.percentile(sgn, 90)))
