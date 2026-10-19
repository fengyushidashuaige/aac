#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证"深度不匹配"机制: 检测器算 offset 用的行, 和它算 wpx 用的行, 深度不一样吗?

detector.py _detect_yolo_seg 里:
  offset: 在 y_ctrl = clip(0.40*H, y0b, y1b) 附近的小窗口取中位
  wpx   : 在整个 mask 的 [p40..p96] 行带里取宽度中位
若这两处的水槽像宽差得多 -> mm_per_px = 110/wpx 就系统性偏大.

用法: cd ~/gutter_uav && python3 width_profile.py <raw_dir> [每N帧取1]
"""
import sys
import os
import glob

sys.path.insert(0, os.path.expanduser('~/gutter_uav'))
os.chdir(os.path.expanduser('~/gutter_uav'))

import numpy as np
import cv2
from config import default_config
from config_jetson import apply_jetson_px4
from detector import TroughDetector

raw_dir = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 25

cfg = default_config
apply_jetson_px4(cfg)
det = TroughDetector(cfg)

files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))[::step]
print('处理 %d 帧' % len(files))
H = 480
y_ctrl = cfg.yolo.ctrl_y_frac * H
print('ctrl_y_frac=%.2f -> y_ctrl=%.0f' % (cfg.yolo.ctrl_y_frac, y_ctrl))
print()
print('%6s %8s %8s %8s %8s %8s %7s %7s' % (
    'idx', 'y0', 'y1', 'w@ctrl', 'w_band中位', 'w_mask中位', '比值', 'off_mm'))
ratios = []
for f in files:
    img = cv2.imread(f)
    if img is None:
        continue
    res = det.detect(img)
    m = getattr(det, '_last_seg_mask', None)
    if m is None or m.sum() == 0 or not res.detected:
        continue
    wpf = []
    for yy in range(m.shape[0]):
        cc = np.where(m[yy] > 0)[0]
        wpf.append(cc.max() - cc.min() + 1 if cc.size >= 3 else 0)
    wpf = np.array(wpf, dtype=np.float64)
    rows = np.where(wpf > 0)[0]
    if rows.size < 20:
        continue
    y0, y1 = rows.min(), rows.max()
    # offset 实际采样行 (clamp 进 mask)
    yc = int(np.clip(y_ctrl, y0, y1))
    band_w = int(max(0.12 * (y1 - y0), 4.0))
    win = wpf[max(0, yc - band_w):yc + band_w + 1]
    win = win[win > 0]
    w_ctrl = np.median(win) if win.size else 0.0
    # detector 用的带宽: ys 的 40%~96% (角度<=8度时) 或 55%~98%
    lo, hi = (0.40, 0.96)
    ya, yb = int(y0 + lo * (y1 - y0)), int(y0 + hi * (y1 - y0))
    seg = wpf[ya:yb + 1]
    seg = seg[seg > 0]
    w_band = np.median(seg) if seg.size else 0.0
    if w_ctrl <= 0 or w_band <= 0:
        continue
    r = w_ctrl / w_band
    ratios.append(r)
    print('%6s %8d %8d %8.1f %8.1f %8.1f %7.3f %7.1f' % (
        os.path.basename(f)[4:10], y0, y1, w_ctrl, w_band,
        np.median(wpf[wpf > 0]), r, res.offset_mm))

ratios = np.array(ratios)
if len(ratios):
    print()
    print('w@ctrl / w_band 比值: 中位 %.3f  均值 %.3f  P10 %.3f  P90 %.3f  (n=%d)' % (
        np.median(ratios), ratios.mean(), np.percentile(ratios, 10),
        np.percentile(ratios, 90), len(ratios)))
    print('若 ~0.74: 说明 mm_per_px 系统性偏大 1/0.74=1.35 倍 -> 就是它!')
