#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""把画面降采样成 ASCII 图打印出来, 用于人肉判断水槽的位置/走向/宽度.
用法: python2 img_ascii.py <jpg> [列数] [行数]
"""
import sys
import numpy as np
import cv2

f = sys.argv[1]
CW = int(sys.argv[2]) if len(sys.argv) > 2 else 80
CH = int(sys.argv[3]) if len(sys.argv) > 3 else 30

img = cv2.imread(f)
if img is None:
    print('!! 读不到 %s' % f)
    sys.exit(1)
h, w = img.shape[:2]
print('图像 %dx%d' % (w, h))

small = cv2.resize(img, (CW, CH), interpolation=cv2.INTER_AREA).astype(np.float32)
b, g, r = small[:, :, 0], small[:, :, 1], small[:, :, 2]
gray = 0.114 * b + 0.587 * g + 0.299 * r
mx = np.max(small, axis=2); mn = np.min(small, axis=2)
sat = mx - mn

ramp = ' .:-=+*#%@'


def show(M, title):
    print('--- %s (min %.0f max %.0f) ---' % (title, M.min(), M.max()))
    lo, hi = np.percentile(M, 2), np.percentile(M, 98)
    N = (np.clip((M - lo) / max(hi - lo, 1e-6), 0, 1) * (len(ramp) - 1)).astype(int)
    for row in N:
        print('  |' + ''.join(ramp[v] for v in row) + '|')


show(gray, '亮度 gray')
show(sat, '饱和度 sat')

# 逐行找"亮带": 对若干采样行打印该行的亮度曲线峰值位置
print('--- 采样行的亮度剖面 (峰值位置 = 槽中心附近) ---')
gy = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
for frac in (0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85):
    yy = int(frac * h)
    row = cv2.GaussianBlur(gy[yy], (1, 9), 0)[0]
    # 找所有局部极大, 报告前 3 个
    idx = []
    for x in range(6, w - 6):
        if row[x] == row[x - 6:x + 7].max() and row[x] > row.mean() + 0.3 * row.std():
            idx.append((float(row[x]), x))
    idx.sort(reverse=True)
    top = idx[:3]
    print('   y=%3d (%.0f%%): 行均值 %5.1f std %5.1f | 亮峰: %s' % (
        yy, frac * 100, row.mean(), row.std(),
        '  '.join('x=%d(v=%.0f)' % (x, v) for v, x in top)))
