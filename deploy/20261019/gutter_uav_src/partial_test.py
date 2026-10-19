#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Translate a real frame so the trough progressively leaves the right edge.

Emulates "aircraft drifts left -> trough slides right out of frame".
A correct detector should report off_mm linear in dx (gutter offset in mm)
until the trough is no longer measurable, and then report detected=False.
A broken one explodes / saturates.
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

frame_path = sys.argv[1]
dxs = ([int(x) for x in sys.argv[2].split(',')]
       if len(sys.argv) > 2 else list(range(0, 321, 20)))

cfg = default_config
apply_jetson_px4(cfg)
cfg.yolo.device = "cuda"
det = TroughDetector(cfg)

img = cv2.imread(frame_path)
if img is None:
    sys.exit('cannot read ' + frame_path)
H, W = img.shape[:2]
print('frame %s  %dx%d' % (os.path.basename(frame_path), W, H))
print()
print('%6s %4s %8s %8s %9s %8s %7s %6s %7s %7s %7s' % (
    'dx', 'det', 'wpx', 'off_px', 'off_mm', 'mm/px', 'ang', 'conf',
    'mask_sx', 'mask_sy', 'ctrl_in'))
for dx in dxs:
    M = np.float32([[1, 0, dx], [0, 1, 0]])
    im2 = cv2.warpAffine(img, M, (W, H), borderValue=(0, 0, 0))
    det.reset()
    r = det.detect(im2)
    m = getattr(det, '_last_seg_mask', None)
    sx = sy = 0
    ctrl_in = -1
    ctrl = int(cfg.yolo.ctrl_y_frac * H)
    if m is not None and m.sum() > 0:
        ys, xs = np.where(m > 0)
        sx = int(xs.max() - xs.min())
        sy = int(ys.max() - ys.min())
        ctrl_in = 1 if (ys.min() <= ctrl <= ys.max()) else 0
    mp = (cfg.trough.trough_width_mm / r.trough_width_px) if r.trough_width_px else 0.0
    print('%6d %4d %8.1f %8.1f %9.1f %8.3f %7.2f %6.2f %7d %7d %7d' % (
        dx, 1 if r.detected else 0, r.trough_width_px, r.offset_px,
        r.offset_mm, mp, r.angle_deg, r.confidence, sx, sy, ctrl_in))
