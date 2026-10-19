#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线跑真检测器 (TensorRT), 把内部量全部打出来. 不碰相机.
用法: cd ~/gutter_uav && python3 diag_det.py <jpg> [<jpg> ...]
"""
import sys
import os

sys.path.insert(0, os.path.expanduser('~/gutter_uav'))
os.chdir(os.path.expanduser('~/gutter_uav'))

import numpy as np
import cv2
from config import default_config
from config_jetson import apply_jetson_px4
from detector import TroughDetector

cfg = default_config
apply_jetson_px4(cfg)
print('weights=%s imgsz=%d conf=%.3f dev=%s' % (
    cfg.yolo.weights, cfg.yolo.imgsz, cfg.yolo.conf, cfg.yolo.device))
print('trough_width_mm=%.1f  mm_per_pixel=%s  mask_bottom_ignore=%.2f ctrl_y_frac=%.2f' % (
    cfg.trough.trough_width_mm, cfg.trough.mm_per_pixel,
    cfg.yolo.mask_bottom_ignore, cfg.yolo.ctrl_y_frac))

det = TroughDetector(cfg)

for f in sys.argv[1:]:
    img = cv2.imread(f)
    if img is None:
        print('!! 读不到 %s' % f)
        continue
    H, W = img.shape[:2]
    print('=' * 100)
    print('图 %s  %dx%d' % (os.path.basename(f), W, H))
    res = det.detect(img)
    m = getattr(det, '_last_seg_mask', None)
    if m is not None and m.sum() > 0:
        ys, xs = np.where(m > 0)
        print('  mask: 面积 %d  bbox x[%d,%d] y[%d,%d] span_x %d span_y %d  model_s=%.3f' % (
            int(m.sum() // 255), xs.min(), xs.max(), ys.min(), ys.max(),
            xs.max() - xs.min(), ys.max() - ys.min(),
            float(getattr(det, '_last_seg_score', 0))))
        print('  mask 逐行宽度(每20行):')
        for yy in range(0, H, 20):
            row = np.where(m[yy] > 0)[0]
            if row.size == 0:
                continue
            brk = np.where(np.diff(row) > 1)[0] + 1
            segs = np.split(row, brk)
            out = ' '.join('[%d-%d w=%d]' % (s.min(), s.max(), s.max() - s.min() + 1)
                           for s in segs if s.size >= 2)
            print('     y=%3d : %s' % (yy, out))
    else:
        print('  mask: 空')
    print('  >>> detected=%s held=%s conf=%.3f' % (
        res.detected, getattr(res, 'held', '?'), res.confidence))
    print('      offset_px=%8.1f   offset_mm=%9.1f   offset_mm_raw=%9.1f' % (
        res.offset_px, res.offset_mm, res.offset_mm_raw))
    print('      angle_deg=%7.2f   trough_width_px=%6.1f   -> mm_per_px=%s' % (
        res.angle_deg, res.trough_width_px,
        ('%.3f' % (cfg.trough.trough_width_mm / res.trough_width_px))
        if res.trough_width_px else 'n/a'))
    print('      (画面中心 x=%.0f -> 槽中心在画面 x=%.1f, 偏离中心 %.1f px)' % (
        W * 0.5, W * 0.5 + res.offset_px, res.offset_px))
