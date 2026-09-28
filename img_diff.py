#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""对比 overlay 帧与 raw 帧, 差值 = 检测器画上去的东西.
用法: python2 img_diff.py <overlay.png> <raw.jpg> [搜索窗口]
"""
import sys
import numpy as np
import cv2

ovf = sys.argv[1]
rawf = sys.argv[2]
win = int(sys.argv[3]) if len(sys.argv) > 3 else 0

ov = cv2.imread(ovf)
raw = cv2.imread(rawf)
print('overlay %s %s | raw %s %s' % (ov.shape, ovf.split('/')[-1], raw.shape, rawf.split('/')[-1]))
if ov is None or raw is None:
    sys.exit('读图失败')

d = cv2.absdiff(ov, raw).astype(np.float32).max(axis=2)
print('整图差值: 中位 %.1f  P90 %.1f  P99.9 %.1f  max %.1f' % (
    np.median(d), np.percentile(d, 90), np.percentile(d, 99.9), d.max()))
print('差值>40 的像素: %d (%.2f%%)' % ((d > 40).sum(), 100.0 * (d > 40).mean()))

mask = d > 40
if mask.sum() < 50:
    print('!! 差值太小 -> overlay 与 raw 不是同一帧/或叠加是半透明的')
    sys.exit(0)

ys, xs = np.where(mask)
print('差异区域 bbox: x[%d,%d] y[%d,%d]' % (xs.min(), xs.max(), ys.min(), ys.max()))

# 差异像素的颜色分布 (BGR)
sel = ov[mask].astype(np.float32)
print('差异像素平均 BGR = [%.0f %.0f %.0f]' % tuple(sel.mean(axis=0)))
# 归一化色相判断: 哪个通道最突出
mb, mg, mr = sel.mean(axis=0)
print('  -> 主色: %s' % ('蓝' if mb > mr + 20 and mb > mg + 20 else
                        '红/粉' if mr > mb + 20 else
                        '绿' if mg > mr + 20 and mg > mb + 20 else '混/白'))

ramp = ' .:-=+*#%@'
cx, cy = 80, 24
print('--- 差值图 (画上去的东西) ---')
sm = cv2.resize(d, (cx, cy), interpolation=cv2.INTER_AREA)
N = (np.clip(sm / max(sm.max(), 1e-6), 0, 1) * 9).astype(int)
for row in N:
    print('  |' + ''.join(ramp[v] for v in row) + '|')
print('--- 原始帧灰度 (对照) ---')
g = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
sm2 = cv2.resize(g, (cx, cy), interpolation=cv2.INTER_AREA)
lo, hi = np.percentile(sm2, 2), np.percentile(sm2, 98)
N2 = (np.clip((sm2 - lo) / max(hi - lo, 1e-6), 0, 1) * 9).astype(int)
for row in N2:
    print('  |' + ''.join(ramp[v] for v in row) + '|')

# 逐行量画出来的中心线
print('--- 每行画出像素的 x 范围/中点 (取每 40 行) ---')
h, w = mask.shape
print('   %5s %6s %6s %6s %6s' % ('y', 'xmin', 'xmax', 'mid', 'width'))
pts = []
for yy in range(0, h, 40):
    band = mask[max(0, yy - 20):yy + 20]
    cols = np.where(band.any(axis=0))[0]
    if cols.size == 0:
        print('   %5d  (空)' % yy)
        continue
    # 分成若干段
    brk = np.where(np.diff(cols) > 12)[0] + 1
    segs = np.split(cols, brk)
    segs = [s for s in segs if s.size >= 3]
    out = []
    for s in segs:
        out.append('[%d,%d]mid=%.0f w=%d' % (s.min(), s.max(), 0.5 * (s.min() + s.max()), s.max() - s.min()))
        pts.append((yy, 0.5 * (s.min() + s.max())))
    print('   %5d  %s' % (yy, '  '.join(out)))
