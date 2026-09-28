#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析离线检测结果 offline.csv: 找出 off_mm 爆表的帧的内部特征."""
import sys
import numpy as np

f = sys.argv[1]
rows = []
with open(f) as fh:
    hdr = fh.readline().strip().split(',')
    for ln in fh:
        p = ln.strip().split(',')
        if len(p) < len(hdr):
            continue
        rows.append(p)
def _col(i):
    v = [r[i] for r in rows]
    try:
        return np.array(v, dtype=float)
    except ValueError:
        return np.array(v, dtype=object)


C = {h: _col(i) for i, h in enumerate(hdr)}

det = C['detected'] > 0.5
print('总帧 %d, 检出 %d (%.0f%%)' % (len(det), det.sum(), 100.0 * det.mean()))
for nm in ('mask_area', 'span_x', 'span_y', 'n_pts', 'wpx', 'off_px', 'off_mm', 'ang_deg',
           'conf', 'mm_per_px', 'midx_at_ctrl'):
    v = C[nm][det]
    if len(v) == 0:
        continue
    print('  %-12s 中位 %9.2f  P10 %9.2f  P90 %9.2f  min %9.2f  max %9.2f' % (
        nm, np.median(v), np.percentile(v, 10), np.percentile(v, 90), v.min(), v.max()))

# 关键: wpx 与 |off_mm| 的关系
print()
print('== 按 wpx 分档看 off_mm (检出帧) ==')
idx = np.where(det)[0]
w = C['wpx'][idx]
o = C['off_mm'][idx]
for lo, hi in ((0, 40), (40, 60), (60, 80), (80, 100), (100, 115), (115, 130), (130, 150), (150, 999)):
    m = (w >= lo) & (w < hi)
    if m.sum() == 0:
        continue
    print('  wpx [%3d,%3d): %3d 帧  |off_mm| 中位 %7.1f  P90 %7.1f  max %8.1f' % (
        lo, hi, m.sum(), np.median(np.abs(o[m])), np.percentile(np.abs(o[m]), 90), np.abs(o[m]).max()))

# 控制行中点 vs 中心线偏移: 两者应该一致
print()
print('== 一致性检查: offset_px 应该 ≈ midx_at_ctrl - 320 ==')
d = C['midx_at_ctrl'] - 320.0
m = det & (C['midx_at_ctrl'] > 0)
if m.sum():
    err = C['off_px'][m] - d[m]
    print('   n=%d  差(off_px - (midx-320)): 中位 %+.1f  P10 %+.1f  P90 %+.1f' % (
        m.sum(), np.median(err), np.percentile(err, 10), np.percentile(err, 90)))

# 最离谱的帧
print()
print('== |off_mm| 最大的 15 帧 ==')
order = np.argsort(-np.abs(C['off_mm']))[:15]
print('  %4s %6s %8s %8s %7s %7s %6s %7s %7s' % (
    'idx', 'area', 'span_x', 'span_y', 'wpx', 'off_px', 'ang', 'off_mm', 'mm/px'))
for i in order:
    print('  %4d %6.0f %8.0f %8.0f %7.1f %7.1f %6.1f %7.1f %7.3f  %s' % (
        C['idx'][i], C['mask_area'][i], C['span_x'][i], C['span_y'][i], C['wpx'][i],
        C['off_px'][i], C['ang_deg'][i], C['off_mm'][i], C['mm_per_px'][i], C['file'][i]))

print()
print('== wpx 最小的 15 帧 ==')
order = np.argsort(C['wpx'])[:15]
print('  %4s %6s %8s %8s %7s %7s %6s %7s %7s %s' % (
    'idx', 'area', 'span_x', 'span_y', 'wpx', 'off_px', 'ang', 'off_mm', 'mm/px', 'det'))
for i in order:
    print('  %4d %6.0f %8.0f %8.0f %7.1f %7.1f %6.1f %7.1f %7.3f  %d' % (
        C['idx'][i], C['mask_area'][i], C['span_x'][i], C['span_y'][i], C['wpx'][i],
        C['off_px'][i], C['ang_deg'][i], C['off_mm'][i], C['mm_per_px'][i], C['detected'][i]))

# 与在线 CSV 的对照需要时间, 这里先看整体: wpx 稳定吗
print()
print('== wpx 时间稳定性 (检出帧, 每 20 个取 1) ==')
ii = idx[::max(1, len(idx) // 25)]
for i in ii:
    print('   idx %4d  wpx %6.1f  off_px %7.1f  off_mm %8.1f  ang %6.1f  conf %.2f  area %6.0f' % (
        C['idx'][i], C['wpx'][i], C['off_px'][i], C['off_mm'][i], C['ang_deg'][i],
        C['conf'][i], C['mask_area'][i]))
