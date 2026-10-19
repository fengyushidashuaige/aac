#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Replay raw frames sequentially through the detector and report the gate.

Frames are fed in order so the detector's internal state (incl. the new
rolling width reference _w_buf) builds up exactly as it would in flight.
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
stride = int(sys.argv[3]) if len(sys.argv) > 3 else 1

cfg = default_config
apply_jetson_px4(cfg)
cfg.yolo.device = "cuda"
det = TroughDetector(cfg)

files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))[::stride]
print('replaying %d frames from %s' % (len(files), raw_dir), flush=True)

out = open(out_csv, 'w')
out.write('idx,wpx,off_px,off_mm,conf,detected,valid,reason,model_s\n')

n_det = n_valid = n_reason = 0
reasons = {}
bad_survived = []
for k, f in enumerate(files):
    img = cv2.imread(f)
    if img is None:
        continue
    r = det.detect(img)
    out.write('%d,%.1f,%.1f,%.2f,%.3f,%d,%d,%s,%.3f\n' % (
        k, r.trough_width_px, r.offset_px, r.offset_mm, r.confidence,
        1 if r.detected else 0, 1 if r.valid else 0,
        r.invalid_reason or '-',
        float(getattr(det, '_last_seg_score', 0.0))))
    if r.detected:
        n_det += 1
        if r.valid:
            n_valid += 1
        if r.invalid_reason:
            n_reason += 1
            reasons[r.invalid_reason] = reasons.get(r.invalid_reason, 0) + 1
        if abs(r.offset_mm) > 300:
            bad_survived.append((k, r.offset_mm, r.trough_width_px, r.valid))
    if k % 500 == 0:
        print('  %d/%d' % (k, len(files)), flush=True)
out.close()

print()
print('frames              : %d' % len(files))
print('detected            : %d (%.1f%%)' % (n_det, 100.0 * n_det / max(len(files), 1)))
print('  of which valid    : %d' % n_valid)
print('  invalid_reason    : %s' % (reasons or 'none'))
print('! |off|>300 survived: %d' % len(bad_survived))
for b in bad_survived[:10]:
    print('   ', b)
print('wrote', out_csv)
