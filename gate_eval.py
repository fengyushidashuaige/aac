#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline evaluation of a proposed validity gate on logged detect.csv series.

Current guard in detector.py is *relative to the previous accepted frame*
(wpx < 0.60 * last_ok_width); a gradual ramp walks the reference down.
Proposed fix: reference = rolling 85th percentile of recent widths, plus an
absolute physical bound on |off_mm| derived from that reference.

Reports: how much of the ramp it catches, and how many GOOD frames it kills.
"""
import sys
import numpy as np

TROUGH_MM = 110.0
IMG_W = 640.0
REF_WIN = 300          # rolling window (frames) ~10s at 30fps
REF_MIN = 40           # need at least this many samples
SHRINK = 0.55          # reject if wpx < SHRINK * ref
OFF_LIMIT_FRAC = 0.5   # |off| <= OFF_LIMIT_FRAC * IMG_W * mm_per_px_ref


def load(path):
    rows = []
    for ln in open(path, 'rb').read().replace(b'\x00', b'').decode(
            'utf-8', 'ignore').splitlines()[1:]:
        p = ln.split(',')
        if len(p) < 7:
            continue
        try:
            rows.append((float(p[1]), float(p[3]), int(p[4]), float(p[6])))
        except Exception:
            rows.append((0.0, 0.0, 0, 0.0))
    return np.array(rows)


def evaluate(r, label):
    n = len(r)
    buf = []
    out_off = np.zeros(n)
    out_ok = np.zeros(n, dtype=bool)
    refs = np.zeros(n)
    for i in range(n):
        off, conf, det, wpx = r[i]
        ref = float(np.percentile(buf, 85)) if len(buf) >= REF_MIN else 0.0
        refs[i] = ref
        ok = False
        if det == 1 and wpx > 1.0:
            if ref <= 0.0:
                ok = True                      # bootstrap: trust the first frames
            else:
                mm_ref = TROUGH_MM / ref
                off_lim = OFF_LIMIT_FRAC * IMG_W * mm_ref
                ok = (wpx >= SHRINK * ref) and (abs(off) <= off_lim)
        out_ok[i] = ok
        out_off[i] = off if ok else 0.0
        if ok and wpx > 1.0:
            buf.append(wpx)
            if len(buf) > REF_WIN:
                buf.pop(0)

    bad_in = (r[:, 2] == 1) & (np.abs(r[:, 0]) > 300)      # logged garbage
    det_in = r[:, 2] == 1
    print('=== %s  (n=%d, logged detected=%d) ===' % (label, n, det_in.sum()))
    print('  logged |off|>300 : %5d' % bad_in.sum())
    print('  gate rejects     : %5d  (%.1f%% of logged-detected)' % (
        (det_in & ~out_ok).sum(), 100.0 * (det_in & ~out_ok).sum() / max(det_in.sum(), 1)))
    print('  garbage caught   : %5d / %d  (%.1f%%)' % (
        (bad_in & ~out_ok).sum(), bad_in.sum(),
        100.0 * (bad_in & ~out_ok).sum() / max(bad_in.sum(), 1)))
    # good frames = logged detected, |off| small, and width healthy
    good = det_in & (np.abs(r[:, 0]) <= 300)
    print('  good frames      : %5d' % good.sum())
    print('  good killed      : %5d  (%.1f%% false reject)' % (
        (good & ~out_ok).sum(), 100.0 * (good & ~out_ok).sum() / max(good.sum(), 1)))
    if good.sum():
        print('  |off| before: med %6.1f  P90 %6.1f  max %6.1f' % (
            np.median(np.abs(r[good, 0])), np.percentile(np.abs(r[good, 0]), 90),
            np.abs(r[good, 0]).max()))
        ok2 = good & out_ok
        if ok2.sum():
            print('  |off| after : med %6.1f  P90 %6.1f  max %6.1f' % (
                np.median(np.abs(r[ok2, 0])), np.percentile(np.abs(r[ok2, 0]), 90),
                np.abs(r[ok2, 0]).max()))
    return out_ok


for path in sys.argv[1:]:
    r = load(path)
    evaluate(r, path.split('/')[-2])
    print()
