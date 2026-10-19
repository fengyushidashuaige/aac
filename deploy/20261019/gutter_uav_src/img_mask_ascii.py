#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整幅画面 ASCII + 叠上检测器 mask. 用法: cd ~/gutter_uav && python3 img_mask_ascii.py <jpg> [列] [行]"""
import sys
import os

sys.path.insert(0, os.path.expanduser('~/gutter_uav'))
os.chdir(os.path.expanduser('~/gutter_uav'))
import numpy as np
import cv2
from config import default_config
from config_jetson import apply_jetson_px4
from detector import TroughDetector

f = sys.argv[1]
CW = int(sys.argv[2]) if len(sys.argv) > 2 else 100
CH = int(sys.argv[3]) if len(sys.argv) > 3 else 32
cfg = default_config
apply_jetson_px4(cfg)
det = TroughDetector(cfg)

img = cv2.imread(f)
res = det.detect(img)
m = getattr(det, '_last_seg_mask', None)
H, W = img.shape[:2]
print('%s  %dx%d  detected=%s off_mm=%.1f ang=%.1f wpx=%.1f conf=%.2f' % (
    os.path.basename(f), W, H, res.detected, res.offset_mm, res.angle_deg,
    res.trough_width_px, res.confidence))

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 先把底部 ROI 涂黑, 复现检测器看到的输入
img2 = gray.copy()
bot = int(H * (1.0 - cfg.yolo.mask_bottom_ignore))
print('底部 ROI 屏蔽从 y=%d 开始 (mask_bottom_ignore=%.2f)' % (bot, cfg.yolo.mask_bottom_ignore))
img2[bot:, :] = 0

sm = cv2.resize(img2, (CW, CH), interpolation=cv2.INTER_AREA).astype(np.float32)
mk = cv2.resize((m if m is not None else np.zeros((H, W), np.uint8)).astype(np.float32),
                (CW, CH), interpolation=cv2.INTER_AREA)
ramp = ' .:-=+*#%@'
lo, hi = np.percentile(sm, 2), np.percentile(sm, 98)
print('--- 画面 (M=模型mask, *=mask+亮) 底部 %d 行是 ROI 屏蔽区 ---' % (CH - int(bot / H * CH)))
for j in range(CH):
    line = ''
    for i in range(CW):
        v = int(np.clip((sm[j, i] - lo) / max(hi - lo, 1e-6), 0, 1) * 9)
        if mk[j, i] > 0.4:
            line += ('*' if v >= 6 else 'M')
        else:
            line += ramp[v]
    tag = '  <-- ROI 屏蔽' if j >= int(bot / H * CH) else ''
    print('  |' + line + '|' + tag)
print('  列: x=0..%d 每字符 %.1f px;  行: y=0..%d 每字符 %.1f px' % (
    W, W / float(CW), H, H / float(CH)))
if res.detected:
    print('  offset_px=%.1f -> 槽中心在画面 x=%.1f (中心线 x=%.0f)' % (
        res.offset_px, W * 0.5 + res.offset_px, W * 0.5))
