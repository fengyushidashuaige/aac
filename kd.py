import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
edits = [
    ("DEAD_MM   = 25.0", "KD_OFF    = 0.0001     # 微分增益 (m/s per mm/s), 抑制过冲\nDEAD_MM   = 25.0"),
    ("        self.kp_off   = float(rospy.get_param('~kp_off', KP_OFF))",
     "        self.kp_off   = float(rospy.get_param('~kp_off', KP_OFF))\n"
     "        self.kd_off   = float(rospy.get_param('~kd_off', KD_OFF))\n"
     "        self.off_prev = None\n"
     "        self.off_dot  = 0.0"),
    ("        vy_t  = 0.0 if abs(off) <= self.dead_mm else self.kp_off * off * self.vy_sign",
     "        # D 项: 偏差变化率(低通后), 给横向环加阻尼 —— 抑制'打满->过冲->反向'的极限环\n"
     "        _af = 1.0 - math.exp(-dt / 0.5)\n"
     "        if self.off_prev is None:\n"
     "            self.off_prev = off\n"
     "        _d = (off - self.off_prev) / dt\n"
     "        self.off_prev = off\n"
     "        self.off_dot += _af * (_d - self.off_dot)\n"
     "        _vy_pd = self.kp_off * off + self.kd_off * self.off_dot\n"
     "        vy_t  = 0.0 if abs(off) <= self.dead_mm else _vy_pd * self.vy_sign"),
    ("        rospy.logwarn('[servo] 增益: kp_off=%.5f dead=%.0fmm max_vy=%.3f slew=%.2f cruise=%.3f kp_ang=%.4f' % (\n"
     "            self.kp_off, self.dead_mm, self.max_vy, self.slew_vy, self.cruise, self.kp_ang))",
     "        rospy.logwarn('[servo] 增益: kp_off=%.5f kd_off=%.5f dead=%.0fmm max_vy=%.3f slew=%.2f cruise=%.3f kp_ang=%.4f' % (\n"
     "            self.kp_off, self.kd_off, self.dead_mm, self.max_vy, self.slew_vy, self.cruise, self.kp_ang))"),
]
for old, new in edits:
    n = s.count(old)
    print('  匹配 %d 处 : %s' % (n, old.strip()[:44]))
    if n == 1:
        s = s.replace(old, new)
open(p, 'w', encoding='utf-8').write(s)
