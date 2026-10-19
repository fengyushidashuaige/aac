#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
同一批画面, 分别跑 旧模型 / v2 模型, 对比 off / ang / wpx.
用来判定 "伺服变差" 是不是 v2 微调模型造成的 (控制变量).
用法: cd ~/gutter_uav && python3 ab_model.py <raw目录> [每N帧取1]
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
step = int(sys.argv[2]) if len(sys.argv) > 2 else 30
OLDM = 'models/gutter_seg_320.engine'
NEWM = 'models/gutter_seg_320_v2.engine'

files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))[::step]
print('取 %d 帧' % len(files))


def make(weights):
    import copy
    cfg = copy.deepcopy(default_config)      # ★ 必须深拷: default_config 是共享单例
    apply_jetson_px4(cfg)
    cfg.yolo.weights = weights
    cfg.yolo.device = 'cuda'
    d = TroughDetector(cfg)
    print('   权重 = %s' % cfg.yolo.weights)
    return d


detA = make(OLDM)
print('--- 旧模型 %s 已加载 ---' % OLDM)
detB = make(NEWM)


def rund(det, img):
    r = det.detect(img)
    return (bool(r.detected), float(r.offset_mm), float(r.angle_deg),
            float(r.trough_width_px), float(r.confidence))


print()
print('%-16s | %-28s | %-28s' % ('', '旧模型 gutter_seg_320', 'v2 模型 gutter_seg_320_v2'))
print('%-16s | %7s %7s %6s %5s | %7s %7s %6s %5s' % (
    '帧', 'off', 'ang', 'wpx', 'conf', 'off', 'ang', 'wpx', 'conf'))
print('-' * 78)
A, B = [], []
for f in files:
    img = cv2.imread(f)
    if img is None:
        continue
    a = rund(detA, img)
    b = rund(detB, img)
    if a[0]:
        A.append(a)
    if b[0]:
        B.append(b)
    print('%-16s | %7.0f %7.1f %6.0f %5.2f | %7.0f %7.1f %6.0f %5.2f' % (
        os.path.basename(f)[4:10],
        a[1] if a[0] else 0, a[2] if a[0] else 0, a[3] if a[0] else 0, a[4],
        b[1] if b[0] else 0, b[2] if b[0] else 0, b[3] if b[0] else 0, b[4]))

A = np.array(A) if A else np.zeros((0, 5))
B = np.array(B) if B else np.zeros((0, 5))
print()
print('检出: 旧 %d/%d (%.0f%%)   v2 %d/%d (%.0f%%)' % (
    len(A), len(files), 100.0 * len(A) / max(len(files), 1),
    len(B), len(files), 100.0 * len(B) / max(len(files), 1)))
for tag, X in (('旧模型', A), ('v2模型', B)):
    if len(X) < 5:
        print('  %s 样本不足' % tag)
        continue
    print('  %-6s off 中位 %+8.1f (P10 %+8.1f P90 %+8.1f)  ang 中位 %+7.1f (P10 %+6.1f P90 %+6.1f)  wpx 中位 %6.1f  conf 中位 %.2f' % (
        tag, np.median(X[:, 1]), np.percentile(X[:, 1], 10), np.percentile(X[:, 1], 90),
        np.median(X[:, 2]), np.percentile(X[:, 2], 10), np.percentile(X[:, 2], 90),
        np.median(X[:, 3]), np.median(X[:, 4])))
