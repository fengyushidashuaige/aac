#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 gutter_track.py + pursuit.py 打补丁: 相机进闭环 (use_cam 的 TODO).
在板上运行: cd ~/guttermap && python3 patch_usecam.py
"""
import os
import shutil
import difflib

B = os.path.expanduser('~/guttermap')


def patch(fn, edits):
    p = os.path.join(B, fn)
    src = open(p, encoding='utf-8').read()
    bak = p + '.bak_pre_usecam'
    if not os.path.exists(bak):
        shutil.copy2(p, bak)
    out = src
    for old, new in edits:
        n = out.count(old)
        if n != 1:
            print('  !! %s: 匹配 %d 次, 跳过这段:\n     %r' % (fn, n, old[:70]))
            continue
        out = out.replace(old, new)
    if out != src:
        open(p, 'w', encoding='utf-8').write(out)
        print('  已改 %s (备份 %s)' % (fn, os.path.basename(bak)))
        for l in difflib.unified_diff(src.split('\n'), out.split('\n'), lineterm='',
                                      fromfile=fn + '.orig', tofile=fn + '.new', n=1):
            print('   ' + l)
    else:
        print('  %s 无改动' % fn)


# ---------------- pursuit.py ----------------
print('== pursuit.py ==')
patch('pursuit.py', [
    ("    def update(self, x, y, yaw, v_now=0.0):\n        c = Command()\n"
     "        s, cte, _ = self.pl.project(x, y)\n        c.s, c.cte = s, cte\n",
     "    def update(self, x, y, yaw, v_now=0.0, lat_shift=0.0):\n        c = Command()\n"
     "        s, cte, _ = self.pl.project(x, y)\n"
     "        c.cte_raw = cte                 # 未经相机修正的原始 cte\n"
     "        cte = cte - lat_shift           # 路径整体横向平移(相机慢修正)\n"
     "        c.s, c.cte = s, cte\n"),
    ("        tx, ty = self.pl.point_at(s + L); c.lookahead = (tx, ty)\n",
     "        tx, ty = self.pl.point_at(s + L)\n"
     "        if abs(lat_shift) > 1e-6:\n"
     "            # 前视目标点也沿路径法向平移, 否则偏航仍会把飞机拉回原路径\n"
     "            _a = self.pl.point_at(s)\n"
     "            _b = self.pl.point_at(min(s + 0.3, self.pl.length))\n"
     "            _dx, _dy = _b[0] - _a[0], _b[1] - _a[1]\n"
     "            _n = math.hypot(_dx, _dy) or 1.0\n"
     "            tx += (-_dy / _n) * lat_shift\n"
     "            ty += (_dx / _n) * lat_shift\n"
     "        c.lookahead = (tx, ty)\n"),
    ("        self.s = self.cte = self.heading_err = 0.0\n",
     "        self.s = self.cte = self.heading_err = 0.0\n        self.cte_raw = 0.0\n"),
])

# ---------------- gutter_track.py ----------------
print()
print('== gutter_track.py ==')
patch('gutter_track.py', [
    # 1) 端口: 45678 被 gutter_map 占了, 检测输出在 45679
    ("UDP_PORT = 45678", "UDP_PORT = 45679"),
    # 2) use_cam 默认打开
    ("rospy.get_param('~use_cam', False)", "rospy.get_param('~use_cam', True)"),
    # 3) 新增慢修正状态
    ("        self.cam_off = None       # 相机偏差(mm), 阶段2\n"
     "        self.cam_t = 0.0\n",
     "        self.cam_off = None       # 相机偏差(mm), + = 槽在右侧\n"
     "        self.cam_t = 0.0\n"
     "        self.off_lp = 0.0         # 相机偏差低通\n"
     "        self.cte_lp = None        # cte 低通(用未修正的 cte_raw)\n"
     "        self.lat_shift = 0.0      # 路径相对真实槽中心的横向偏差估计(m)\n"),
    # 4) 未使能时的预览也带上修正(只为打印)
    ("            _c = self.pp.update(x, y, yaw, self.speed)\n",
     "            _c = self.pp.update(x, y, yaw, self.speed, lat_shift=self.lat_shift)\n"
     "            self.update_cam_fix(_c)\n"),
    # 5) 使能分支: 传入修正 + 更新慢滤波
    ("        c = self.pp.update(x, y, yaw, self.speed)\n"
     "        if c.finished:\n",
     "        c = self.pp.update(x, y, yaw, self.speed, lat_shift=self.lat_shift)\n"
     "        self.update_cam_fix(c)\n"
     "        if c.finished:\n"),
    # 6) 日志里带上修正量, 便于复飞判读
    ("  vx=%.2f vy=%.2f yaw=%.3f  cam=%s'",
     "  vx=%.2f vy=%.2f yaw=%.3f  fix=%+.0fmm  cam=%s'"),
    ("                                 c.vx, c.vy, c.yaw_rate,\n"
     "                                 ('%+.0f' % self.cam_off) if self.cam_off is not None else '--'))\n",
     "                                 c.vx, c.vy, c.yaw_rate, 1000 * self.lat_shift,\n"
     "                                 ('%+.0f' % self.cam_off) if self.cam_off is not None else '--'))\n"),
    # 7) 实现 update_cam_fix (插在 on_odom 之前)
    ("    # ---------- 回调 ----------\n    def on_odom(self, m):\n",
     "    # ---------- 相机慢修正 ----------\n"
     "    def update_cam_fix(self, c):\n"
     "        \"\"\"用相机 offset_mm 把路径整体横向平移, 补偿路径的绝对横向误差.\n"
     "        路径直得很(残差~7mm)但绝对位置差 ±300mm —— 相机的横向/偏航安装角、\n"
     "        offset 尺度这些量在\"直线拟合\"里不可观, 只能靠相机闭环消掉.\n"
     "        低带宽(TAU=2.5s): 快动态仍由雷达路径负责, 相机只管慢漂移, 两者不打架.\n"
     "        \"\"\"\n"
     "        TAU = 2.5\n"
     "        dt = 1.0 / max(self.rate_hz, 1.0)\n"
     "        if self.use_cam and self.cam_off is not None:\n"
     "            a = 1.0 - math.exp(-dt / TAU)\n"
     "            self.off_lp += a * (self.cam_off / 1000.0 - self.off_lp)\n"
     "            if self.cte_lp is None:\n"
     "                self.cte_lp = c.cte_raw\n"
     "            else:\n"
     "                self.cte_lp += a * (c.cte_raw - self.cte_lp)\n"
     "            tgt = self.cte_lp - self.off_lp      # 路径相对槽中心的横向偏差\n"
     "            self.lat_shift += a * (tgt - self.lat_shift)\n"
     "            self.lat_shift = max(-0.8, min(0.8, self.lat_shift))\n"
     "        # 相机丢失时冻结 lat_shift (保持上次修正), 不归零\n"
     "\n"
     "    # ---------- 回调 ----------\n    def on_odom(self, m):\n"),
])
print()
print('完成. 检查语法:')
for fn in ('pursuit.py', 'gutter_track.py'):
    print('  %s: %s' % (fn, os.popen('python2 -m py_compile %s && echo OK' % os.path.join(B, fn)).read().strip() or 'FAIL'))
