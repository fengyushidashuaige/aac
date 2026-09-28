#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
回放测试: 用上次把系统带飞的那份真实数据, 对比 旧逻辑 / 新逻辑 的 lat_shift 轨迹.
旧: conf>=0.6 就采纳  -> 应该被 -846mm 的垃圾带飞到限幅
新: 加 |off|<=250 / wpx带宽 / |cte|<=400mm / 变化率限幅 -> 应该纹丝不动
"""
import math
import re
import numpy as np

LOG = '/home/amov/guttermap/run/track.log'
CSV = '/home/amov/logs/guttermap_20260922_115649/detect.csv'

# --- 读 track.log: 取 cte (idle 行和使能行都有 cte) ---
pat = re.compile(r'\[(\d+\.\d+)\]: \[track\].*?s=\s*([\d.]+)/([\d.]+)\s+cte=([+-][\d.]+)mm')
L = []
for ln in open(LOG, 'rb').read().decode('utf-8', 'ignore').split('\n'):
    ln = ln.replace('\x00', '').strip()
    m = pat.search(ln)
    if m:
        L.append((float(m.group(1)), float(m.group(2)), float(m.group(4)) / 1000.0,
                  1 if 'vy=' in ln else 0))
L = np.array(L)
print('日志 %d 条  cte 范围 %+.2f ~ %+.2f m  使能行 %d' % (
    len(L), L[:, 2].min(), L[:, 2].max(), int(L[:, 3].sum())))

# --- 读检测 ---
D = []
for i, ln in enumerate(open(CSV)):
    if i == 0:
        continue
    p = ln.strip().split(',')
    if len(p) < 7:
        continue
    try:
        D.append((float(p[0]), float(p[1]), float(p[3]), int(p[4]), float(p[6])))
    except Exception:
        pass
D = np.array(D)
print('检测 %d 帧' % len(D))


def run(new_logic):
    TAU, dt = 2.5, 0.05
    a = 1.0 - math.exp(-dt / TAU)
    off_lp, cte_lp, lat, wpx_lp = 0.0, None, 0.0, None
    cam_off, cam_t = None, -99.0
    trace = []
    # 按时间合并: 每个检测帧用最近(<=0.5s)的日志 cte
    j = 0
    for d in D:
        t, off_mm, conf, det, w = d
        # 对齐日志
        while j + 1 < len(L) and L[j + 1][0] <= t:
            j += 1
        if abs(L[j][0] - t) > 0.5:
            continue
        cte_now = L[j][2]
        # ---- 复现 udp_loop 的采纳逻辑 ----
        if new_logic:
            ok = det == 1 and conf >= 0.6 and abs(off_mm) <= 250.0
            if ok and w > 1.0:
                wpx_lp = w if wpx_lp is None else 0.98 * wpx_lp + 0.02 * w
                if not (0.80 * wpx_lp <= w <= 1.25 * wpx_lp):
                    ok = False
            if ok:
                cam_off, cam_t = off_mm, t
            elif t - cam_t > 1.0:
                cam_off = None
        else:
            if det == 1 and conf >= 0.6:
                cam_off, cam_t = off_mm, t
            elif t - cam_t > 1.0:
                cam_off = None
        # ---- 复现 update_cam_fix ----
        if cam_off is not None:
            if new_logic and abs(cam_off) > 250.0:
                pass
            elif new_logic and abs(cte_now) > 0.40:
                pass
            else:
                off_lp += a * (cam_off / 1000.0 - off_lp)
                cte_lp = cte_now if cte_lp is None else cte_lp + a * (cte_now - cte_lp)
                tgt = cte_lp - off_lp
                dd = a * (tgt - lat)
                if new_logic:
                    cap = 0.10 * dt
                    dd = max(-cap, min(cap, dd))
                    lat += dd
                    lat = max(-0.5, min(0.5, lat))
                else:
                    lat += dd
                    lat = max(-0.8, min(0.8, lat))
        trace.append((t, lat, cam_off, cte_now))
    return trace


for tag, nl in (('旧逻辑(无门控)', False), ('新逻辑(三道门+限幅)', True)):
    tr = run(nl)
    lats = np.array([x[1] for x in tr])
    print()
    print('== %s ==' % tag)
    print('   lat_shift: 终值 %+.0fmm  最大 %+.0f  最小 %+.0f' % (
        1000 * lats[-1], 1000 * lats.max(), 1000 * lats.min()))
    for x in tr[::max(1, len(tr) // 12)]:
        print('     t=%.0f  lat=%+6.0fmm  cam=%s  cte=%+.0fmm' % (
            x[0], 1000 * x[1],
            ('%+.0f' % x[2]) if x[2] is not None else '--', 1000 * x[3]))
