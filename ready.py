#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""起飞前"能不能使能"看板 —— 1 秒一刷, 大字, 一眼看懂。
用法: 起飞前悬停时, 另开一个终端跑
      python3 ~/guttermap/ready.py
判定"可以起飞"的三个条件:
  ① cam 不是 -- (相机在喂数据)          <- flyL 就死在这条
  ② fix 已经走到 25mm 以上且不再动      <- 相机预收敛完成, 一起飞就是中的
  ③ |cte| < 100mm 且 |herr| < 15deg     <- 飞机离路径别太远
"""
import re, sys, time, os

def _cls():
    sys.stdout.write('\033[2J\033[H')   # ANSI 清屏, 不依赖 TERM
    sys.stdout.flush()

L = '/home/amov/guttermap/run/track.log'
IDLE = re.compile(r'\(idle\) fix=([-+0-9]+)mm\s+cam=([-+0-9]+|--)')
NE = re.compile(r'\(未使能\) s=\s*([0-9.]+)/[0-9.]+\s+cte=\s*([-+0-9.]+)mm\s+herr=\s*([-+0-9.]+)deg')
EN = re.compile(r'\[track\] s=\s*([0-9.]+)/[0-9.]+\s+cte=\s*([-+0-9.]+)mm\s+herr=\s*([-+0-9.]+)deg')
R = '\033[0m'; G = '\033[92m'; RED = '\033[91m'; Y = '\033[93m'; B = '\033[1m'


def tail(n=400):
    try:
        with open(L, errors='replace') as f:
            return f.readlines()[-n:]
    except Exception:
        return []


hist = []
while True:
    lines = tail()
    fix = cam = cte = herr = sdash = None
    enabled = False
    for ln in reversed(lines):
        if fix is None:
            m = IDLE.search(ln)
            if m:
                fix = float(m.group(1)); cam = m.group(2)
        if cte is None:
            m = NE.search(ln)
            if m:
                cte = float(m.group(2)); herr = float(m.group(3))
            else:
                m = EN.search(ln)
                if m:
                    sdash = float(m.group(1)); cte = float(m.group(2))
                    herr = float(m.group(3)); enabled = True
        if fix is not None and cte is not None:
            break
    hist.append(fix)
    if len(hist) > 6:
        hist.pop(0)

    _cls()
    print('=' * 52)
    if enabled:
        print(B + '        跟踪【已使能】—— 松开看板, 专心飞' + R)
        print('=' * 52)
        print('   进度 s=%.2f/5.34m    cte=%+.0fmm    herr=%+.1fdeg'
              % (sdash if sdash is not None else 0.0,
                 cte if cte is not None else 0.0,
                 herr if herr is not None else 0.0))
        time.sleep(1); continue

    ok_cam = cam is not None and cam != '--'
    moved = fix is not None and abs(fix) >= 25.0
    still = len(hist) >= 4 and max(hist[-4:]) - min(hist[-4:]) <= 3.0
    ok_fix = moved and still
    ok_pose = cte is not None and abs(cte) < 100 and herr is not None and abs(herr) < 15
    ready = ok_cam and ok_fix and ok_pose

    if fix is None and cte is None:
        print(Y + '   等数据... 确认 track 节点起了没' + R)
        print('=' * 52)
        print('   (节点没在跑? bash ~/guttermap/track_start.sh ...)')
        time.sleep(1); continue

    print((G + B + '      ★★ 可以按 set_enable true 了 ★★' + R) if ready
          else (RED + B + '      ✗  还不行 —— 别使能, 继续悬停' + R))
    print('=' * 52)
    print()
    print('  相机   cam=%s        %s' % (cam or '?',
          (G + '✓ 在喂数据' + R) if ok_cam else (RED + '✗ 相机瞎了! 平移/转向找槽' + R)))
    print('  fix    %smm         %s' % ('%+.0f' % fix if fix is not None else '?',
          (G + '✓ 已收敛' + R) if ok_fix else
          ((Y + '… 还在走' + R) if moved else (RED + '✗ 没在走 = 相机没数据' + R))))
    print('  位置   cte=%smm herr=%sdeg  %s' % (
          '%.0f' % cte if cte is not None else '?',
          '%.1f' % herr if herr is not None else '?',
          (G + '✓' + R) if ok_pose else (Y + '… 偏得有点多' + R)))
    print()
    print('-' * 52)
    print('  判据: ①cam 有数字 ②fix 走到 ±25mm 以上且停住 ③cte<100 herr<15')
    print('  三条全绿才使能。若 90 秒后仍未就绪, 闸门会兜底放行(会瞎飞)')
    time.sleep(1)
