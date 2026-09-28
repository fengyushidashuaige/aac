#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补完 use_cam 剩下 4 处 (上次中文注释没匹配上). 纯 ASCII 锚点."""
import os
import shutil
import difflib

B = os.path.expanduser('~/guttermap')
p = os.path.join(B, 'gutter_track.py')
src = open(p, encoding='utf-8').read()
out = src

METHOD = (
    "    def update_cam_fix(self, c):\n"
    "        TAU = 2.5\n"
    "        dt = 1.0 / max(self.rate_hz, 1.0)\n"
    "        if self.use_cam and self.cam_off is not None:\n"
    "            a = 1.0 - math.exp(-dt / TAU)\n"
    "            self.off_lp += a * (self.cam_off / 1000.0 - self.off_lp)\n"
    "            if self.cte_lp is None:\n"
    "                self.cte_lp = c.cte_raw\n"
    "            else:\n"
    "                self.cte_lp += a * (c.cte_raw - self.cte_lp)\n"
    "            tgt = self.cte_lp - self.off_lp\n"
    "            self.lat_shift += a * (tgt - self.lat_shift)\n"
    "            self.lat_shift = max(-0.8, min(0.8, self.lat_shift))\n"
    "\n")

edits = [
    ("        self.cam_t = 0.0\n",
     "        self.cam_t = 0.0\n"
     "        self.off_lp = 0.0\n"
     "        self.cte_lp = None\n"
     "        self.lat_shift = 0.0\n"),
    ("    def on_odom(self, m):\n", METHOD + "    def on_odom(self, m):\n"),
    ("'vx=%.2f vy=%.2f yaw=%.3f  cam=%s'\n",
     "'vx=%.2f vy=%.2f yaw=%.3f  fix=%+.0fmm cam=%s'\n"),
    ("                             c.vx, c.vy, c.yaw_rate,\n",
     "                             c.vx, c.vy, c.yaw_rate, 1000 * self.lat_shift,\n"),
]
for old, new in edits:
    n = out.count(old)
    if n != 1:
        print('!! 匹配 %d 次: %r' % (n, old[:60]))
        continue
    out = out.replace(old, new)

if out != src:
    open(p, 'w', encoding='utf-8').write(out)
    print('已写入')
    for l in difflib.unified_diff(src.split('\n'), out.split('\n'), lineterm='',
                                  fromfile='a', tofile='b', n=1):
        print(' ' + l)
