#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 mask 的中心行, 打出图像的亮度剖面 + mask 覆盖范围.
看模型 mask 是不是比画面上"真实水槽"那一条窄 -> 若是, 说明
trough_width_mm=110 用错了(模型只分割了槽的一部分, 实际有效宽度 ~81mm).

用法: cd ~/gutter_uav && python3 mask_vs_bright.py <raw_dir> [每N帧]
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
step = int(sys.argv[2]) if len(sys.argv) > 2 else 40
cfg = default_config
apply_jetson_px4(cfg)
det = TroughDetector(cfg)

files = sorted(glob.glob(os.path.join(raw_dir, 'raw_*.jpg')))[::step]
print('处理 %d 帧' % len(files))
ok = 0
for f in files:
    img = cv2.imread(f)
    if img is None:
        continue
    res = det.detect(img)
    m = getattr(det, '_last_seg_mask', None)
    if m is None or m.sum() == 0 or not res.detected:
        continue
    rows = np.where(m.any(axis=1))[0]
    if rows.size == 0:
        continue
    yc = int(rows[len(rows) // 2])
    xs = np.where(m[yc] > 0)[0]
    if xs.size < 5:
        continue
    m0, m1 = int(xs.min()), int(xs.max())
    ok += 1
    if ok > 12:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    # 沿该行做平滑, 去掉噪声
    prof = cv2.GaussianBlur(gray[yc:yc + 1, :], (1, 11), 0)[0]
    lo, hi = int(max(0, m0 - 120)), int(min(639, m1 + 120))
    seg = prof[lo:hi + 1]
    # 用 16 个字符把这段压成条带 (每个字符 ~ (hi-lo)/16 像素)
    N = 64
    idx = np.linspace(0, len(seg) - 1, N).astype(int)
    v = seg[idx]
    vmin, vmax = np.percentile(seg, 5), np.percentile(seg, 99)
    ramp = ' .:-=+*#%@'
    bar = ''.join(ramp[int(np.clip((x - vmin) / max(vmax - vmin, 1e-6), 0, 1) * 9)] for x in v)
    # 标注 mask 覆盖的字符区间
    k0 = int((m0 - lo) / max(hi - lo, 1) * (N - 1))
    k1 = int((m1 - lo) / max(hi - lo, 1) * (N - 1))
    mark = ''.join('^' if (k0 <= i <= k1) else ' ' for i in range(N))
    print('%-16s y=%3d  mask x=[%3d,%3d] w=%3d  亮度 %.0f~%.0f  对槽宽 %d->wpx报告%.0f' % (
        os.path.basename(f), yc, m0, m1, m1 - m0 + 1, seg.min(), seg.max(),
        cfg.trough.trough_width_mm, res.trough_width_px))
    print('   亮度|' + bar + '|')
    print('   mask|' + mark + '|')
    print('   x 范围 %d..%d (每字符 %.1f px)' % (lo, hi, (hi - lo) / float(N - 1)))
print()
print('共 %d 帧有检出' % ok)
print('判读: 若"亮度条带"里水槽那一段(亮且连续)比 mask 的 ^^^ 明显更宽,')
print('      说明 mask 只覆盖了槽的一部分 -> trough_width_mm=110 偏大,')
print('      有效宽度应为 110 * (mask宽/真实槽宽).')
