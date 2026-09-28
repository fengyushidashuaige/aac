#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把建图 png 用 ASCII 打出来: R=红线(槽线) T=灰线(轨迹)"""
import sys
import numpy as np
import cv2

f = sys.argv[1]
CW = int(sys.argv[2]) if len(sys.argv) > 2 else 110
CH = int(sys.argv[3]) if len(sys.argv) > 3 else 44

img = cv2.imread(f)
h, w = img.shape[:2]
b, g, r = img[:, :, 0].astype(np.int32), img[:, :, 1].astype(np.int32), img[:, :, 2].astype(np.int32)
isred = (r > 150) & (r > b + 60)
ismark = (np.abs(b - g) < 30) & (np.abs(g - r) < 30) & (b > 30) & (b < 110)
# 飞机轨迹: BGR(200,140,60) 蓝
isgray = (b > 150) & (b > r + 60)
isgrid = ismark

ys, xs = np.where(isred | isgray)
print('%s %dx%d  红像素 %d 灰像素 %d' % (f, w, h, isred.sum(), isgray.sum()))
if len(xs) == 0:
    sys.exit('没有线条像素')
x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
print('线条范围 x[%d,%d] y[%d,%d]  (宽 %d 高 %d)' % (x0, x1, y0, y1, x1 - x0, y1 - y0))

out = []
for i in range(CH):
    yy0 = y0 + (y1 - y0 + 1) * i // CH
    yy1 = y0 + (y1 - y0 + 1) * (i + 1) // CH
    line = ''
    for j in range(CW):
        xx0 = x0 + (x1 - x0 + 1) * j // CW
        xx1 = x0 + (x1 - x0 + 1) * (j + 1) // CW
        nr = isred[yy0:max(yy1, yy0 + 1), xx0:max(xx1, xx0 + 1)].sum()
        ng = isgray[yy0:max(yy1, yy0 + 1), xx0:max(xx1, xx0 + 1)].sum()
        if nr > 0 and ng > 0:
            line += 'X'
        elif nr > 0:
            line += 'R'
        elif ng > 0:
            line += 'T'
        else:
            line += ' '
    out.append(line)
print('  +' + '-' * CW + '+')
for line in out:
    print('  |' + line + '|')
print('  +' + '-' * CW + '+')
print('  R=红线(槽线)  T=灰线(飞行轨迹)  X=重合')
print('  横轴 = 图 x (%.2f m 跨度), 纵轴 = 图 y' % 0)
