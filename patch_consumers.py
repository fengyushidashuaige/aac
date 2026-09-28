#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Carry the detector's `valid` flag through to the controllers.

gutter_udp.py  : add 'valid'/'reason' to the UDP payload + STATE, add a
                 `valid` column at the END of detect.csv (existing readers
                 index by position and tolerate extra trailing columns).
servo_track.py : require pkt_valid, so an untrustworthy offset can never
                 command a lateral velocity. `conf` is NOT usable for this:
                 it is anti-correlated with measurement quality (higher when
                 the trough is off-centre).
"""
import os
import sys

HOME = os.path.expanduser('~')
n_ok = 0
n_bad = 0


def fail(msg):
    global n_bad
    n_bad += 1
    print('!! ' + msg)


# --------------------------------------------------------------------------
# gutter_udp.py  -- line based edits (file has Chinese comments)
# --------------------------------------------------------------------------
P = HOME + '/guttermap/gutter_udp.py'
lines = open(P, encoding='utf-8').read().split('\n')
out = []
done = {'msg': 0, 'state': 0, 'hdr': 0, 'row': 0}
for ln in lines:
    if "'wpx': float(getattr(r, 'trough_width_px', 0.0))}" in ln and not done['msg']:
        i = ln.rindex('}')
        head, tail = ln[:i], ln[i + 1:]
        out.append(head.rstrip() + ',')
        out.append("           'valid': bool(getattr(r, 'valid', True)),")
        out.append("           'reason': str(getattr(r, 'invalid_reason', '') or '')}"
                   + tail)
        done['msg'] = 1
        continue
    # STATE.update(...) spans two source lines; hook the tail of the second
    if "fps=round(fps, 1))" in ln and not done['state']:
        out.append(ln.replace("fps=round(fps, 1))",
                              "fps=round(fps, 1), valid=msg.get('valid', True))"))
        done['state'] = 1
        continue
    if "csvf.write('t,offset_mm,angle_deg,conf,detected,model_s,wpx" in ln and not done['hdr']:
        out.append(ln.replace('model_s,wpx\\n', 'model_s,wpx,valid\\n'))
        done['hdr'] = 1
        continue
    if "csvf.write('%.4f,%.2f,%.2f,%.3f,%d,%.3f,%.1f\\n'" in ln and not done['row']:
        out.append(ln.replace("%.3f,%.1f\\n'", "%.3f,%.1f,%d\\n'"))
        done['row'] = 1
        continue
    if "msg.get('wpx', 0.0)))" in ln and done['row'] == 1:
        out.append(ln.replace("msg.get('wpx', 0.0)))",
                              "msg.get('wpx', 0.0), int(msg.get('valid', True))))"))
        done['row'] = 2
        continue
    out.append(ln)

if all(v for v in (done['msg'], done['state'], done['hdr'])) and done['row'] == 2:
    open(P, 'w', encoding='utf-8').write('\n'.join(out))
    print('gutter_udp.py : msg+state+csv ok')
    n_ok += 1
else:
    fail('gutter_udp.py edits incomplete: %s' % done)

# --------------------------------------------------------------------------
# servo_track.py
# --------------------------------------------------------------------------
P = HOME + '/guttermap/servo_track.py'
s = open(P, encoding='utf-8').read()
edits = [
    ("        self.off = self.ang = self.conf = 0.0\n",
     "        self.off = self.ang = self.conf = 0.0\n"
     "        self.pkt_valid = True     # 2026-09-22: geometry validity from detector\n"),
    ("                self.detected = bool(j.get('detected', False))\n",
     "                self.detected = bool(j.get('detected', False))\n"
     "                self.pkt_valid = bool(j.get('valid', True))\n"),
    ("        valid = fresh and self.detected and (not self.held) and self.conf >= MIN_CONF\n",
     "        valid = (fresh and self.detected and (not self.held)\n"
     "                 and self.pkt_valid and self.conf >= MIN_CONF)\n"),
]
cnt = 0
for old, new in edits:
    c = s.count(old)
    if c != 1:
        fail('servo_track.py: %d matches for %r' % (c, old[:48]))
        continue
    s = s.replace(old, new)
    cnt += 1
if cnt == 3:
    open(P, 'w', encoding='utf-8').write(s)
    print('servo_track.py: 3/3 ok')
    n_ok += 1
else:
    fail('servo_track.py: only %d/3' % cnt)

print()
print('done: %d ok, %d failed' % (n_ok, n_bad))
sys.exit(1 if n_bad else 0)
