#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage 1b: make the validity gate active from frame 1, and stop a rejected
frame from poisoning the following ~40 frames via the 15 mm/frame slew clamp.

Found by replaying the demo raw frames (2026-09-22):
  idx 660  wpx 47.7  off_px 287  -> off_mm 662.9  (edge sliver, let through
           because _w_buf needed 40 accepted frames before the gate armed)
  idx 661-667  held, repeating 662.9
  idx 673-690  width back to 116-174, off_px 121-268 (a normal trough!) but
           off_mm still walking down 15.0 mm/frame from the 662.9 spike.

Fixes:
  1. trough.ref_width_px (config default 130px) seeds the reference so the gate
     is armed on the very first frame; the rolling buffer still adapts it.
  2. lower the buffer requirement 40 -> 8.
  3. clear _last_clamp_mm whenever the frame is not accepted, so the slew
     clamp re-anchors to the next good measurement instead of slewing back
     from a rejected one.
"""
import os
import sys

HOME = os.path.expanduser('~')
n_ok = n_bad = 0


def fail(m):
    global n_bad
    n_bad += 1
    print('!! ' + m)


def edit(path, pairs):
    global n_ok, n_bad
    s = open(path, encoding='utf-8').read()
    good = True
    for old, new, tag in pairs:
        c = s.count(old)
        if c != 1:
            fail('%s: %s -> %d matches' % (os.path.basename(path), tag, c))
            good = False
            continue
        s = s.replace(old, new)
        print('   %-22s ok' % tag)
    if good:
        open(path, 'w', encoding='utf-8').write(s)
        n_ok += 1
    return good


# ---- config.py : new field -------------------------------------------------
edit(HOME + '/gutter_uav/config.py', [
    ("    trough_width_mm: float = 110.0\n",
     "    trough_width_mm: float = 110.0\n"
     "\n"
     "    # 2026-09-22: seed for the rolling validity reference (px). Arms the gate\n"
     "    # on frame 1, before the rolling window has any samples. 0 = disabled.\n"
     "    ref_width_px: float = 0.0\n",
     'config.py field'),
])

# ---- config_jetson.py : set it --------------------------------------------
edit(HOME + '/gutter_uav/config_jetson.py', [
    ("    cfg.yolo.ctrl_y_frac = 0.40\n",
     "    cfg.yolo.ctrl_y_frac = 0.40\n"
     "    # typical trough width in px at flight altitude; seeds the validity gate\n"
     "    cfg.trough.ref_width_px = 130.0\n",
     'jetson seed'),
])

# ---- detector.py : gate uses the seed + smaller window ---------------------
edit(HOME + '/gutter_uav/detector.py', [
    ("        ref = float(np.percentile(self._w_buf, 85)) if len(self._w_buf) >= 40 else 0.0\n",
     "        ref = float(np.percentile(self._w_buf, 85)) if len(self._w_buf) >= 8 else 0.0\n"
     "        if ref <= 1.0:\n"
     "            ref = float(getattr(self.config.trough, 'ref_width_px', 0.0) or 0.0)\n",
     'gate seed'),
])

# ---- detector.py : re-anchor the slew clamp on a rejected frame ------------
edit(HOME + '/gutter_uav/detector.py', [
    ("        if result.detected:\n"
     "            self._last_clamp_mm = result.offset_mm\n",
     "        if result.detected:\n"
     "            self._last_clamp_mm = result.offset_mm\n"
     "        else:\n"
     "            # 2026-09-22: do not slew back from a rejected measurement -- one\n"
     "            # bad frame would otherwise walk 15 mm/frame for ~40 frames.\n"
     "            self._last_clamp_mm = None\n",
     'clamp re-anchor'),
])

print()
print('%d file(s) ok, %d failed' % (n_ok, n_bad))
sys.exit(1 if n_bad else 0)
