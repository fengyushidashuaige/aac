#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把建图 png 用 ASCII 打出来, 按颜色分类 (红=槽线/蓝=轨迹/其它)."""
import sys
import numpy as np
import cv2

f = sys.argv[1]
CW = int(sys.argv[2]) if len(sys.argv) > 2 else 100
CH = int(sys.argv[3]) if len(sys.argv) > 3 else 42

img = cv2.imread(f)
print('%s  %s' % (f, img.shape))
h, w = img.shape[:2]
s = cv2.resize(img, (CW, CH), interpolation=cv2.INTER_AREA).astype(np.float32)
b, g, r = s[:, :, 0], s[:, :, 1], s[:, :, 2]

ink = (255.0 - (b + g + r) / 3.0)          # 非白底
red = np.clip(r - np.maximum(b, g), 0, 255)
blue = np.clip(b - np.maximum(r, g), 0, 255)
green = np.clip(g - np.maximum(r, b), 0, 255)
print('红像素 %d, 蓝像素 %d, 绿像素 %d' % ((red > 25).sum(), (blue > 25).sum(), (green > 25).sum()))

out = []
for yy in range(CH):
    line = ''
    for xx in range(CW):
        if red[yy, xx] > 25:
            line += 'R'
        elif blue[yy, xx] > 25:
            line += 'B'
        elif green[yy, xx] > 25:
            line += 'G'
        elif ink[yy, xx] > 90:
            line += '#'
        elif ink[yy, xx] > 30:
            line += '.'
        else:
            line += ' '
    out.append(line)
print('--- 图 (R=红 B=蓝 G=绿 #=深  .=浅) ---')
for line in out:
    print('  |' + line + '|')
print('   上=图上方, 左=图左侧')
