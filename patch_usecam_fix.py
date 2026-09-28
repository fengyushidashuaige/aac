#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修 use_cam 闭环被垃圾数据带飞的问题 (三道门 + 变化率限幅). 板上运行."""
import os
import difflib

p = os.path.expanduser('~/guttermap/gutter_track.py')
src = open(p, encoding='utf-8').read()
out = src

OLD_UDP = (
    "                if j.get('detected') and not j.get('held') and j.get('conf', 0) >= 0.6:\n"
    "                    self.cam_off = j.get('offset_mm', 0.0)\n"
    "                    self.cam_t = time.time()\n"
    "                elif time.time() - self.cam_t > 1.0:\n"
    "                    self.cam_off = None\n")
NEW_UDP = (
    "                ok = (j.get('detected') and not j.get('held')\n"
    "                      and j.get('conf', 0) >= 0.6)\n"
    "                _off = float(j.get('offset_mm', 0.0) or 0.0)\n"
    "                _w = float(j.get('wpx', 0.0) or 0.0)\n"
    "                # 门1: 物理上限. mask 被截断时检测器给出 -846/-1252mm 这种\n"
    "                #   不可能的值(相机横向视野只有 ±250mm), 而 conf 是模型分主导挡不住.\n"
    "                if ok and abs(_off) > 250.0:\n"
    "                    ok = False\n"
    "                # 门2: wpx 相对滑动参考的带宽(mask 截断会让 wpx 偏小)\n"
    "                if ok and _w > 1.0:\n"
    "                    self.wpx_lp = _w if self.wpx_lp is None else 0.98 * self.wpx_lp + 0.02 * _w\n"
    "                    if not (0.80 * self.wpx_lp <= _w <= 1.25 * self.wpx_lp):\n"
    "                        ok = False\n"
    "                if ok:\n"
    "                    self.cam_off = _off\n"
    "                    self.cam_t = time.time()\n"
    "                elif time.time() - self.cam_t > 1.0:\n"
    "                    self.cam_off = None\n")

OLD_FIX = (
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
    "            self.lat_shift = max(-0.8, min(0.8, self.lat_shift))\n")
NEW_FIX = (
    "        if not (self.use_cam and self.cam_off is not None):\n"
    "            return                        # 无有效数据: 冻结上一值, 不归零\n"
    "        TAU = 2.5\n"
    "        dt = 1.0 / max(self.rate_hz, 1.0)\n"
    "        # 门3: 飞机不在路径附近时不许学(停在一边时看到的槽不能用)\n"
    "        if abs(self.cam_off) > 250.0 or abs(c.cte_raw) > 0.40:\n"
    "            return\n"
    "        a = 1.0 - math.exp(-dt / TAU)\n"
    "        self.off_lp += a * (self.cam_off / 1000.0 - self.off_lp)\n"
    "        if self.cte_lp is None:\n"
    "            self.cte_lp = c.cte_raw\n"
    "        else:\n"
    "            self.cte_lp += a * (c.cte_raw - self.cte_lp)\n"
    "        tgt = self.cte_lp - self.off_lp\n"
    "        _d = a * (tgt - self.lat_shift)\n"
    "        _cap = 0.10 * dt                  # 变化率限幅 100mm/s\n"
    "        self.lat_shift += max(-_cap, min(_cap, _d))\n"
    "        self.lat_shift = max(-0.5, min(0.5, self.lat_shift))\n")

edits = [
    (OLD_UDP, NEW_UDP),
    (OLD_FIX, NEW_FIX),
    ("        self.lat_shift = 0.0\n", "        self.lat_shift = 0.0\n        self.wpx_lp = None\n"),
]
for old, new in edits:
    n = out.count(old)
    if n != 1:
        print('!! 匹配 %d 次:\n%s' % (n, old[:120]))
        continue
    out = out.replace(old, new)

if out != src:
    open(p, 'w', encoding='utf-8').write(out)
    print('已写入')
    for l in difflib.unified_diff(src.split('\n'), out.split('\n'), lineterm='',
                                  fromfile='a', tofile='b', n=1):
        print(' ' + l)
else:
    print('无改动')
