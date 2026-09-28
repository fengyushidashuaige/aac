#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把"标定后的相机航向"接进 gutter_track: 相机直接测机头与槽的夹角, 补一个航向环.
板上运行: cd ~/guttermap && python3 patch_heading.py
"""
import os
import difflib

p = os.path.expanduser('~/guttermap/gutter_track.py')
src = open(p, encoding='utf-8').read()
out = src

edits = [
    # 1) 常数
    ("UDP_PORT = 45679\n",
     "UDP_PORT = 45679\n"
     "K_CAM = 0.5        # 相机航向项增益 (rad/s per rad). 实测 psi 精度 3~6deg\n"
     "PSI_TAU = 0.6      # 航向低通时间常数 s\n"),
    # 2) 状态 + 载入标定
    ("        self.wpx_lp = None\n",
     "        self.wpx_lp = None\n"
     "        self.cam_ang = None       # 相机倾角(度)\n"
     "        self.hgt = 0.4            # 当前高度(雷达 z)\n"
     "        self.psi_lp = 0.0\n"
     "        self.psi_cam = None\n"
     "        self.cal = None\n"
     "        try:\n"
     "            _cp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ang_cal.json')\n"
     "            if os.path.exists(_cp):\n"
     "                self.cal = json.load(open(_cp))\n"
     "                rospy.logwarn('[track] 角度标定已载入 A=%.3f B=%.3f C=%.3f (%s, CV中位 %.1f deg)'\n"
     "                              % (self.cal['A'], self.cal['B'], self.cal['C'],\n"
     "                                 self.cal.get('model', '?'), self.cal.get('cv_med_deg', -1)))\n"
     "        except Exception as _e:\n"
     "            rospy.logwarn('[track] 角度标定载入失败: %s' % _e)\n"),
    # 3) UDP: 存倾角 + 倾角门
    ("                _off = float(j.get('offset_mm', 0.0) or 0.0)\n"
     "                _w = float(j.get('wpx', 0.0) or 0.0)\n",
     "                _off = float(j.get('offset_mm', 0.0) or 0.0)\n"
     "                _w = float(j.get('wpx', 0.0) or 0.0)\n"
     "                _ang = float(j.get('angle_deg', 0.0) or 0.0)\n"
     "                if ok and abs(_ang) > 45.0:\n"
     "                    ok = False        # 倾角超物理上限 -> 坏 mask\n"),
    ("                if ok:\n"
     "                    self.cam_off = _off\n"
     "                    self.cam_t = time.time()\n",
     "                if ok:\n"
     "                    self.cam_off = _off\n"
     "                    self.cam_ang = _ang\n"
     "                    self.cam_t = time.time()\n"),
    # 4) 存高度
    ("        self.pose = (p.x, p.y, yaw)\n",
     "        self.pose = (p.x, p.y, yaw)\n        self.hgt = p.z\n"),
    # 5) 航向更新方法 (插在 update_cam_fix 之前)
    ("    def update_cam_fix(self, c):\n",
     "    def update_cam_heading(self, dt):\n"
     "        \"\"\"相机直接测的航向误差 psi(机头相对槽). 系数由 ang_cal.py 标定.\n"
     "        tan(tilt) = A*tan(psi) + B*(d/h) + C  =>  psi = atan((tan(tilt)-B*d/h-C)/A)\n"
     "        实测交叉验证: 中位 3.2deg / P90 5.6deg.\n"
     "        \"\"\"\n"
     "        if self.cal is None or self.cam_ang is None or self.cam_off is None:\n"
     "            self.psi_cam = None\n"
     "            return\n"
     "        A = float(self.cal['A']); B = float(self.cal['B']); C = float(self.cal['C'])\n"
     "        if abs(A) < 1e-6:\n"
     "            self.psi_cam = None\n"
     "            return\n"
     "        h = max(float(self.hgt), 0.10)\n"
     "        d = self.cam_off / 1000.0\n"
     "        t = math.tan(math.radians(self.cam_ang))\n"
     "        psi = math.atan((t - B * (d / h) - C) / A)\n"
     "        if abs(psi) > math.radians(45):\n"
     "            self.psi_cam = None          # 离谱, 不要\n"
     "            return\n"
     "        a = 1.0 - math.exp(-dt / max(PSI_TAU, 1e-3))\n"
     "        self.psi_lp += a * (psi - self.psi_lp)\n"
     "        self.psi_cam = psi\n"
     "        return\n"
     "\n"
     "    def update_cam_fix(self, c):\n"),
    # 6) 控制循环: 更新 + 叠加航向项
    ("        c = self.pp.update(x, y, yaw, self.speed, lat_shift=self.lat_shift)\n"
     "        self.update_cam_fix(c)\n",
     "        c = self.pp.update(x, y, yaw, self.speed, lat_shift=self.lat_shift)\n"
     "        self.update_cam_fix(c)\n"
     "        self.update_cam_heading(1.0 / max(self.rate_hz, 1.0))\n"
     "        if self.psi_cam is not None:\n"
     "            # 直接把机头压到与槽平行(喷管需要). psi=0 即机头与槽平行\n"
     "            lim = self.pp.max_yaw_rate\n"
     "            c.yaw_rate = max(-lim, min(lim, c.yaw_rate + K_CAM * self.psi_lp))\n"),
    # 7) 日志加 psi
    ("'vx=%.2f vy=%.2f yaw=%.3f  fix=%+.0fmm cam=%s'\n",
     "'vx=%.2f vy=%.2f yaw=%.3f  fix=%+.0fmm psi=%s cam=%s'\n"),
    ("                             c.vx, c.vy, c.yaw_rate, 1000 * self.lat_shift,\n",
     "                             c.vx, c.vy, c.yaw_rate, 1000 * self.lat_shift,\n"
     "                             ('%+.1f' % math.degrees(self.psi_lp)) if self.psi_cam is not None else '--',\n"),
]

for old, new in edits:
    n = out.count(old)
    if n != 1:
        print('!! 匹配 %d 次:\n%s\n' % (n, old[:100]))
        continue
    out = out.replace(old, new)

if out != src:
    open(p, 'w', encoding='utf-8').write(out)
    print('已写入 gutter_track.py')
    for l in difflib.unified_diff(src.split('\n'), out.split('\n'), lineterm='',
                                  fromfile='a', tofile='b', n=0):
        if l.startswith(('+', '-')) and not l.startswith(('+++', '---')):
            print(' ' + l)
else:
    print('无改动')
