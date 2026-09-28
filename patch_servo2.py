#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_track.py 改造: 分档(yaw_only/center_only/forward) + 符号可调 + psi 标定角.
只动这一个文件. 板上运行: cd ~/guttermap && python3 patch_servo2.py
"""
import os
import difflib

p = os.path.expanduser('~/guttermap/servo_track.py')
src = open(p, encoding='utf-8').read()
out = src

edits = [
    # 1) import os + Odometry
    ("import math, socket, json, time, threading\nimport rospy\n",
     "import math, socket, json, time, threading, os\nimport rospy\n"
     "from nav_msgs.msg import Odometry\n"),
    # 2) 常数
    ("DEAD_MM   = 12.0",
     "OFF_MAX   = 250.0       # 偏差物理上限 mm (相机横向视野); 超了必是坏 mask\n"
     "ANG_MAX   = 45.0        # 倾角物理上限 deg\n"
     "DEAD_MM   = 12.0"),
    # 3) __init__: 参数 + 标定 + 高度
    ("        self.vy_cmd = self.yaw_cmd = 0.0\n"
     "        self.last_t = time.time(); self.last_print = 0.0\n",
     "        self.vy_cmd = self.yaw_cmd = 0.0\n"
     "        self.last_t = time.time(); self.last_print = 0.0\n"
     "\n"
     "        # ---- 分档测试开关 (照老版本 single-line-working 的做法) ----\n"
     "        self.yaw_only    = bool(rospy.get_param('~yaw_only', False))\n"
     "        self.center_only = bool(rospy.get_param('~center_only', False))\n"
     "        self.forward     = bool(rospy.get_param('~forward', False))\n"
     "        # ---- 符号 (FRD->FLU 迁移可能导致反, 做成参数, 反了就改这里) ----\n"
     "        self.yaw_sign = float(rospy.get_param('~yaw_sign', +1.0))\n"
     "        self.vy_sign  = float(rospy.get_param('~vy_sign', +1.0))\n"
     "        self.use_psi  = bool(rospy.get_param('~use_psi', True))\n"
     "        self.hgt = 0.4\n"
     "        self.psi = None\n"
     "        self.cal = None\n"
     "        try:\n"
     "            _cp = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ang_cal.json')\n"
     "            if os.path.exists(_cp):\n"
     "                self.cal = json.load(open(_cp))\n"
     "        except Exception as _e:\n"
     "            rospy.logwarn('[servo] ang_cal.json 读取失败: %s' % _e)\n"
     "        rospy.Subscriber('/Odometry', Odometry, self.on_odom, queue_size=10)\n"),
    # 4) on_odom 回调
    ("    def udp_loop(self):\n",
     "    def on_odom(self, m):\n"
     "        self.hgt = m.pose.pose.position.z\n"
     "\n"
     "    def udp_loop(self):\n"),
    # 5) 控制律: 分档 + 符号 + psi
    ("        off, ang = self.off, self.ang\n"
     "        vy_t  = 0.0 if abs(off) <= DEAD_MM  else KP_OFF * off\n"
     "        yaw_t = 0.0 if abs(ang) <= DEAD_DEG else KP_ANG * ang\n"
     "        vy_t  = max(-MAX_VY,  min(MAX_VY,  vy_t))\n"
     "        yaw_t = max(-MAX_YAW, min(MAX_YAW, yaw_t))\n"
     "\n"
     "        self.vy_cmd  = self._slew(self.vy_cmd,  vy_t,  SLEW_VY,  dt)\n"
     "        self.yaw_cmd = self._slew(self.yaw_cmd, yaw_t, SLEW_YAW, dt)\n"
     "\n"
     "        a = min(1.0, abs(off) / 150.0); b = min(1.0, abs(ang) / 20.0)\n"
     "        vx = CRUISE * max(0.25, 1.0 - max(a, b))\n"
     "        self.publish(vx, self.vy_cmd, self.yaw_cmd)\n",
     "        off, ang = self.off, self.ang\n"
     "        if abs(off) > OFF_MAX or abs(ang) > ANG_MAX:\n"
     "            self.publish(0.0, 0.0, 0.0)\n"
     "            rospy.logwarn_throttle(2.0, '[servo] 超物理上限 off=%.0f ang=%.1f -> 停' % (off, ang))\n"
     "            return\n"
     "        # ---- 航向误差: 优先用标定后的 psi(已扣掉偏差项和模型偏置) ----\n"
     "        self.psi = None\n"
     "        if self.use_psi and self.cal is not None:\n"
     "            A = float(self.cal['A']); B = float(self.cal['B']); C = float(self.cal['C'])\n"
     "            if abs(A) > 1e-6:\n"
     "                h = max(self.hgt, 0.10)\n"
     "                d = off / 1000.0\n"
     "                self.psi = math.degrees(math.atan(\n"
     "                    (math.tan(math.radians(ang)) - B * (d / h) - C) / A))\n"
     "        head = self.psi if self.psi is not None else ang\n"
     "        if abs(head) < DEAD_DEG:\n"
     "            head = 0.0\n"
     "\n"
     "        vy_t  = 0.0 if abs(off) <= DEAD_MM else KP_OFF * off * self.vy_sign\n"
     "        yaw_t = 0.0 if head == 0.0 else KP_ANG * head * self.yaw_sign\n"
     "        vy_t  = max(-MAX_VY,  min(MAX_VY,  vy_t))\n"
     "        yaw_t = max(-MAX_YAW, min(MAX_YAW, yaw_t))\n"
     "\n"
     "        self.vy_cmd  = self._slew(self.vy_cmd,  vy_t,  SLEW_VY,  dt)\n"
     "        self.yaw_cmd = self._slew(self.yaw_cmd, yaw_t, SLEW_YAW, dt)\n"
     "\n"
     "        if self.yaw_only:\n"
     "            self.vy_cmd = 0.0; vx = 0.0\n"
     "        elif self.center_only or not self.forward:\n"
     "            vx = 0.0\n"
     "        else:\n"
     "            a = min(1.0, abs(off) / 150.0); b = min(1.0, abs(head) / 20.0)\n"
     "            vx = CRUISE * max(0.25, 1.0 - max(a, b))\n"
     "        self.publish(vx, self.vy_cmd, self.yaw_cmd)\n"),
    # 6) 日志加 psi
    ("            rospy.loginfo('[servo] off=%+7.1fmm ang=%+6.1f conf=%.2f  vx=%.3f vy=%+.3f yaw=%+.3f'\n"
     "                          % (off, ang, self.conf, vx, self.vy_cmd, self.yaw_cmd))\n",
     "            _ps = ('%+6.1f' % self.psi) if self.psi is not None else '  --  '\n"
     "            rospy.loginfo('[servo] off=%+7.1fmm ang=%+6.1f psi=%s conf=%.2f  vx=%.3f vy=%+.3f yaw=%+.3f'\n"
     "                          % (off, ang, _ps, self.conf, vx, self.vy_cmd, self.yaw_cmd))\n"),
]

for old, new in edits:
    n = out.count(old)
    if n != 1:
        print('!! 匹配 %d 次:\n%s\n' % (n, old[:90]))
        continue
    out = out.replace(old, new)

if out != src:
    open(p, 'w', encoding='utf-8').write(out)
    print('已写入 servo_track.py')
else:
    print('无改动')
