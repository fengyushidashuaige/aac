#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线仿真验证 use_cam 闭环的符号和收敛性.
场景: 路径沿 x 轴 (y=0, y 为"左"正). 真实水槽在路径右侧 100mm -> y_trough = -0.1.
      相机给 off = y_air - y_trough (+, 槽在右侧), 飞机在路径上时 off=+100mm.
期望: 不带修正 -> 飞机稳在 y=0 (路径上, 但离槽偏 100mm);
      带修正   -> 飞机收敛到 y=-0.1 (压在槽中心), 且 lat_shift -> -0.1.
"""
import math
import os
import sys

sys.path.insert(0, os.path.expanduser('~/guttermap'))
from pursuit import Polyline, PurePursuit

PATH = [(i * 0.1, 0.0) for i in range(0, 61)]      # (0,0)->(6,0)
Y_TROUGH = -0.1


def sim(use_fix, x0=0.0, y0=0.0, yaw0=0.0, T=40.0, dt=0.05):
    pl = Polyline(PATH).smooth(win=5, passes=2)
    pp = PurePursuit(pl, v_cruise=0.15, lookahead=0.8, slow_radius=1.0,
                     max_yaw_rate=0.4, max_vy=0.15, cte_gain=0.5)
    x, y, yaw = x0, y0, yaw0
    off_lp, cte_lp, lat = 0.0, None, 0.0
    TAU = 2.5
    a = 1.0 - math.exp(-dt / TAU)
    hist = []
    t = 0.0
    while t < T:
        c = pp.update(x, y, yaw, 0.15, lat_shift=lat)
        # 相机: 真实观测 + 小噪声
        off = y - Y_TROUGH
        if use_fix:
            off_lp += a * (off - off_lp)
            cte_lp = c.cte_raw if cte_lp is None else cte_lp + a * (c.cte_raw - cte_lp)
            tgt = cte_lp - off_lp
            lat += a * (tgt - lat)
            lat = max(-0.8, min(0.8, lat))
        # 简易运动学
        x += c.vx * dt * math.cos(yaw) - c.vy * dt * math.sin(yaw)
        y += c.vx * dt * math.sin(yaw) + c.vy * dt * math.cos(yaw)
        yaw += c.yaw_rate * dt
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        hist.append((t, x, y, c.s, c.cte, lat, off))
        t += dt
    return hist


def report(tag, hist):
    last = hist[-1]
    seg = hist[len(hist) // 2:]
    ys = [h[2] for h in seg]
    cs = [h[4] for h in seg]
    print('%-22s 终态: x=%.2f s=%.2f  y=%+.3f (离槽 %+.0fmm)  lat_shift=%+.3f  |cte|中位 %.0fmm'
          % (tag, last[1], last[3], last[2], 1000 * (last[2] - Y_TROUGH), last[5],
             1000 * sum(abs(v) for v in cs) / len(cs)))
    return last[2]


h0 = sim(False)
h1 = sim(True)
print('目标: 真实水槽在 y=%.3f (路径右侧 100mm)' % Y_TROUGH)
print()
y_no = report('不带相机修正', h0)
y_fix = report('带相机修正(新)', h1)
print()
print('轨迹采样 (带修正):')
for h in h1[::100]:
    print('   t=%4.1f x=%.2f y=%+.3f s=%.2f cte=%+.0fmm lat=%+.3f off=%+.0fmm'
          % (h[0], h[1], h[2], h[3], 1000 * h[4], h[5], 1000 * h[6]))
print()
ok = abs(y_fix - Y_TROUGH) < 0.03
print('结论: %s  (收敛到槽中心 %s, 偏 %+.0fmm)'
      % ('✅ 符号正确, 能压在槽中心' if ok else '❌ 符号反了/不收敛',
         'Y' if ok else 'N', 1000 * (y_fix - Y_TROUGH)))
print('      不带修正停在 y=%+.3f, 离槽 %+.0fmm (说明修正真的在起作用)' % (y_no, 1000 * (y_no - Y_TROUGH)))
