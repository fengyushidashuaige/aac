#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patch ~/gutter_uav/detector.py: absolute-reference geometric validity gate.

Problem (measured 2026-09-22):
  w_shrink = (wpx < 0.60 * self._last_ok_width)  compares against the PREVIOUS
  accepted frame. When the trough slowly leaves the frame each step is < 40%,
  so the guard never fires and the reference width is walked down with the
  garbage. Logged ramp: off 143 -> 1225 mm over ~25 frames, conf 0.86~0.95.

Fix:
  1. reference width = 85th percentile of a rolling ~300-frame buffer of
     accepted widths (robust while bad frames are a minority);
  2. reject if wpx < 0.55 * ref  (real shrink);
  3. reject if |off_mm| > 0.5 * img_w * (trough_mm / ref)  (beyond lateral FOV,
     i.e. physically impossible);
  4. expose result.valid / result.invalid_reason so consumers stop relying on
     `conf`, which is anti-correlated with quality (higher when off-centre).
"""
import os
import sys

P = os.path.expanduser('~/gutter_uav/detector.py')
s = open(P, encoding='utf-8').read()
orig = s
n = 0


def rep(old, new, tag):
    global s, n
    c = s.count(old)
    if c != 1:
        print('!! %-22s expected 1 match, got %d -> SKIP' % (tag, c))
        return False
    s = s.replace(old, new)
    n += 1
    print('   %-22s ok' % tag)
    return True


# --- 1. result fields -------------------------------------------------------
ok1 = rep(
    "    detected: bool = False\n",
    "    detected: bool = False\n"
    "\n"
    "    # --- 2026-09-22: geometry validity of THIS frame's offset ---\n"
    "    # valid=False means offset_mm is not trustworthy (trough partly out of\n"
    "    # frame / beyond lateral FOV), NOT merely low model confidence.\n"
    "    valid: bool = False\n"
    "    invalid_reason: str = ''\n",
    'result fields')

# --- 2. state: rolling width buffer -----------------------------------------
ok2 = rep(
    "        self._last_ok_width: float = 0.0\n",
    "        self._last_ok_width: float = 0.0\n"
    "        # 2026-09-22: rolling width reference (85th pct). Deliberately NOT\n"
    "        # cleared by reset(): it is a slow, absolute reference.\n"
    "        self._w_buf = []\n",
    'state _w_buf')

# --- 3. gate method ---------------------------------------------------------
GATE = '''    def _gate_validity(self, R, img_w: float) -> None:
        """2026-09-22: reject geometrically impossible offsets.

        The old guard compared wpx against the previous accepted frame only;
        a slow drift out of frame passes every step and walks the reference
        down with it. Use an absolute rolling reference instead.
        """
        if not R.detected:
            return
        tw = float(getattr(self.config.trough, "trough_width_mm", 110.0) or 110.0)
        w = float(R.trough_width_px or 0.0)
        ref = float(np.percentile(self._w_buf, 85)) if len(self._w_buf) >= 40 else 0.0
        reason = ''
        if ref > 1.0 and w > 0.0:
            off_lim = 0.5 * float(img_w) * (tw / ref)
            if w < 0.55 * ref:
                reason = 'width_shrink'
            elif abs(float(R.offset_mm or 0.0)) > off_lim:
                reason = 'off_over_fov'
        if reason:
            R.detected = False
            R.valid = False
            R.invalid_reason = reason
            R.confidence = 0.0
            return
        R.valid = True
        R.invalid_reason = ''
        if w > 1.0:
            self._w_buf.append(w)
            if len(self._w_buf) > 300:
                self._w_buf.pop(0)

'''
ok3 = rep("    def _refine_mask_to_trough(", GATE + "    def _refine_mask_to_trough(",
          'gate method')

# --- 4. call the gate at the end of _detect_yolo_seg ------------------------
ok4 = rep(
    '        R.confidence = float(getattr(self, "_last_seg_score", 0.5))\n'
    '        return R\n',
    '        R.confidence = float(getattr(self, "_last_seg_score", 0.5))\n'
    '        R.offset_mm = self._px_to_mm(R.offset_px, R.trough_width_px)\n'
    '        self._gate_validity(R, float(w))\n'
    '        return R\n',
    'gate call')

# --- 5. held frames carry a good value: mark valid --------------------------
ok5 = rep(
    "                    detected=True,\n"
    "                    held=True,\n",
    "                    detected=True,\n"
    "                    held=True,\n"
    "                    valid=True,\n",
    'held valid')

print()
if n == 5 and s != orig:
    open(P, 'w', encoding='utf-8').write(s)
    print('patched %d/5 -> %s' % (n, P))
else:
    print('!! %d/5 applied -- NOT written' % n)
    sys.exit(1)
