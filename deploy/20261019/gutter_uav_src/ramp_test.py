#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reproduce the off_mm ramp by translating the frame VERTICALLY.

Hypothesis: the control row y_ctrl=0.40*480=192 gets clipped into the mask's
y-range, so the measurement point is "the bottom edge of the mask" and moves
into the distance as the trough recedes -> mm/px drifts -> off_mm ramps up.

Vertical shift emulates the trough receding toward the horizon.
Prints the internals needed to see the clamp: mask y range, y_ctrl, off_px,
wpx, mm/px, and whether the mask touches the image border.
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
mode = sys.argv[2] if len(sys.argv) > 2 else 'y'
vals = ([int(x) for x in sys.argv[3].split(',')]
        if len(sys.argv) > 3 else list(range(0, -161, -10)))

cfg = default_config
apply_jetson_px4(cfg)
cfg.yolo.device = "cuda"
det = TroughDetector(cfg)

img = cv2.imread(frame_path)
if img is None:
    sys.exit('cannot read ' + frame_path)
H, W = img.shape[:2]
ctrl = int(cfg.yolo.ctrl_y_frac * H)
print('frame %s  %dx%d   ctrl_y_frac=%.2f -> y_ctrl=%d' % (
    os.path.basename(frame_path), W, H, cfg.yolo.ctrl_y_frac, ctrl))
print()
print('%6s %4s %7s %7s %7s %9s %8s %6s %8s %8s %7s %7s' % (
    'shift', 'det', 'mask_y0', 'mask_y1', 'wpx', 'off_px', 'off_mm',
    'conf', 'mm/px', 'y_ctrl*', 'clipL', 'clipR'))
for v in vals:
    dx, dy = (0, v) if mode == 'y' else (v, 0)
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    im2 = cv2.warpAffine(img, M, (W, H), borderValue=(0, 0, 0))
    det.reset()
    r = det.detect(im2)
    m = getattr(det, '_last_seg_mask', None)
    y0 = y1 = -1
    clipL = clipR = 0
    if m is not None and m.sum() > 0:
        ys, xs = np.where(m > 0)
        y0, y1 = int(ys.min()), int(ys.max())
        clipL = 1 if xs.min() <= 1 else 0
        clipR = 1 if xs.max() >= W - 2 else 0
    # the row the detector actually measured at = clip(y_ctrl, y0, y1)
    y_used = -1
    if y0 >= 0:
        y_used = int(min(max(ctrl, y0), y1))
    mp = (cfg.trough.trough_width_mm / r.trough_width_px) if r.trough_width_px else 0.0
    print('%6d %4d %7d %7d %7.1f %7.1f %9.1f %6.2f %8.3f %8d %7d %7d' % (
        v, 1 if r.detected else 0, y0, y1, r.trough_width_px, r.offset_px,
        r.offset_mm, r.confidence, mp, y_used, clipL, clipR))
