#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A/B the NEW detector against the pre-patch backup on the same frames.

Answers one question only: did the validity gate reduce detections?
Frames are fed in order so internal state builds up the same way for both.
"""
import sys
import os
import glob
import importlib.util

sys.path.insert(0, os.path.expanduser('~/gutter_uav'))
os.chdir(os.path.expanduser('~/gutter_uav'))

import numpy as np
import cv2
from config import default_config
from config_jetson import apply_jetson_px4

raw_dir = sys.argv[1]
stride = int(sys.argv[2]) if len(sys.argv) > 2 else 1
files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))[::stride]
print('A/B on %d frames from %s' % (len(files), raw_dir), flush=True)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod.TroughDetector


def run(cls, tag):
    cfg = default_config
    apply_jetson_px4(cfg)
    cfg.yolo.device = "cuda"
    det = cls(cfg)
    n = 0
    offs = []
    wpxs = []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        r = det.detect(img)
        if r.detected:
            n += 1
            offs.append(r.offset_mm)
            wpxs.append(r.trough_width_px)
    offs = np.array(offs) if offs else np.array([0.0])
    wpxs = np.array(wpxs) if wpxs else np.array([0.0])
    print('%-8s detected %4d/%d (%.1f%%)  |off| med %6.1f  max %7.1f  wpx med %5.1f' % (
        tag, n, len(files), 100.0 * n / max(len(files), 1),
        np.median(np.abs(offs)), np.abs(offs).max(), np.median(wpxs)), flush=True)
    return n


old = load(os.path.expanduser('~/gutter_uav/old_detector.py'), 'old_detector')
print('--- OLD (pre-patch) ---', flush=True)
n_old = run(old, 'OLD')
del sys.modules['old_detector']
print('--- NEW (with gate) ---', flush=True)
from detector import TroughDetector as NewDet
n_new = run(NewDet, 'NEW')

print()
print('OLD %d  ->  NEW %d    diff %+d (%.1f%%)' % (
    n_old, n_new, n_new - n_old, 100.0 * (n_new - n_old) / max(n_old, 1)))
