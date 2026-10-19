#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 raw/ 下所有帧离线跑一遍, 落盘内部量, 便于和 detect.csv 对照.
用法: cd ~/gutter_uav && python3 diag_all.py <raw_dir> <out.csv> [起始帧号] [帧步长]
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
out_csv = sys.argv[2]
cfg = default_config
apply_jetson_px4(cfg)
cfg.yolo.device = "cuda"

det = TroughDetector(cfg)

files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))
print('共 %d 帧' % len(files), flush=True)

out = open(out_csv, 'w')
out.write('idx,file,mask_area,span_x,span_y,n_pts,wpx,off_px,off_mm,ang_deg,conf,detected,'
          'mm_per_px,midx_at_ctrl,ctrl_row\n')

for k, f in enumerate(files):
    img = cv2.imread(f)
    if img is None:
        continue
    H, W = img.shape[:2]
    res = det.detect(img)
    m = getattr(det, '_last_seg_mask', None)
    area = int(m.sum() // 255) if m is not None else 0
    if m is not None and area > 0:
        ys, xs = np.where(m > 0)
        sx, sy = int(xs.max() - xs.min()), int(ys.max() - ys.min())
    else:
        sx = sy = 0
    ctrl = int(cfg.yolo.ctrl_y_frac * H)
    # 控制行处 mask 的中点
    midx = -1.0
    if m is not None and m[ctrl].sum() > 0:
        cc = np.where(m[ctrl] > 0)[0]
        midx = 0.5 * (cc.min() + cc.max())
    mpx = (cfg.trough.trough_width_mm / res.trough_width_px) if res.trough_width_px else 0.0
    n = k + 1
    out.write('%d,%s,%d,%d,%d,%d,%.1f,%.1f,%.2f,%.2f,%.3f,%d,%.4f,%.1f,%d\n' % (
        n, os.path.basename(f), area, sx, sy,
        len(res.left_points or []) + len(res.right_points or []),
        res.trough_width_px, res.offset_px, res.offset_mm, res.angle_deg,
        res.confidence, 1 if res.detected else 0, mpx, midx, ctrl))
    if k % 50 == 0:
        print('  %d/%d' % (k, len(files)), flush=True)
out.close()
print('写出 %s' % out_csv)
