#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""track.log 快速验收报告。
用法: python3 ~/guttermap/track_report.py [log路径]
自动把"出航段"(到最大 s 为止) 与"回程/手飞/落地"分开, 只对出航段算 |cte|<=50。
FROZEN: 连续相同 (s,cte) 的尾巴视为落地冻结, 剔掉。
"""
import re, sys
P = sys.argv[1] if len(sys.argv) > 1 else '/home/amov/guttermap/run/track.log'
LINE = re.compile(r'\[track\] s=\s*([0-9.]+)/([0-9.]+)\s+cte=\s*([-+0-9.]+)mm\s+'
                  r'herr=\s*([-+0-9.]+)deg.*?vx=([0-9.]+).*?cam=([-+0-9]+|--)')
rows = []
for ln in open(P, errors='replace'):
    m = LINE.search(ln)
    if not m:
        continue
    s, tot, cte, herr, vx, cam = m.groups()
    cte = float(cte)
    if abs(cte) > 5000:
        continue
    rows.append([float(s), float(tot), cte, float(herr),
                 None if cam == '--' else float(cam)])
if not rows:
    print('没有使能样本'); sys.exit(0)
tot = rows[0][1]
fz = 0
for i in range(len(rows) - 1, 0, -1):
    if abs(rows[i][2] - rows[i-1][2]) < 1e-6 and abs(rows[i][0] - rows[i-1][0]) < 1e-6:
        fz += 1
    else:
        break
live = rows[:len(rows) - fz]
imax = max(range(len(live)), key=lambda i: live[i][0])


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else float('nan')


def stat(tag, g):
    if not g:
        print('%-8s (无)' % tag); return
    c = [r[2] for r in g]
    cm = [r[4] for r in g if r[4] is not None]
    ok = 100.0 * sum(1 for x in c if abs(x) <= 50) / len(c)
    print('%-8s n=%4d  s %.2f..%.2f  走完 %3.0f%%  cte中位 %+7.1f  |cte|<=50 %5.1f%%  '
          'cam中位 %s (n=%d, %d%%)'
          % (tag, len(g), min(r[0] for r in g), max(r[0] for r in g),
             100.0 * max(r[0] for r in g) / tot, med(c), ok,
             ('%+.1f' % med(cm)) if cm else '--', len(cm),
             100 * len(cm) // len(g)))


print('log: %s   路径全长 %.2f m   使能 %d 帧 (冻结尾 %d 帧已剔除)'
      % (P, tot, len(rows), fz))
stat('出航段', live[:imax + 1])
stat('回程段', live[imax + 1:])
