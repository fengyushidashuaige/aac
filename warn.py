import os, shutil
p = os.path.expanduser('~/guttermap/cam_tune.sh')
s = open(p, encoding='utf-8').read()
if 'DEPRECATED' not in s:
    shutil.copy2(p, p + '.bak_deprecated')
    warn = """#!/bin/bash
# ============================================================================
#  !! DEPRECATED / 别用 !!  (2026-09-22 加)
#  这是【换镜头之前】的手动测光方案: 它会把
#      exposure_auto=1(手动曝光) / brightness=-64 / gamma=64 / WB=2800
#  写死, 再往上爬 gain. 现役镜头【带 IR-CUT】, 这套值会把画面压糊压灰,
#  而且手动曝光远不如自动曝光稳(光照一变就废)。
#  正确做法: 什么都别做 —— gutter_udp.py 上电时的 cam_init() 会自动应用
#      exposure_auto=3 / gamma=100 / brightness=0 / gain=60 / WB自动
#  即: 想恢复就是 bash ~/guttermap/start_detect.sh
#  实测: 跑过本脚本后画面亮度/对比确实变差(2026-09-22)。
# ============================================================================
"""
    lines = s.split('\n')
    body = '\n'.join(lines[1:])          # 去掉原来的 shebang
    open(p, 'w', encoding='utf-8').write(warn + body)
    print('已加 DEPRECATED 警告')
else:
    print('已有警告')
