import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
edits = [
    # 默认值: 保守一档(测量有延迟, 增益大了必过冲)
    ("KP_OFF    = 0.0008", "KP_OFF    = 0.0004"),
    ("MAX_VY    = 0.10",   "MAX_VY    = 0.06"),
    ("DEAD_MM   = 12.0",   "DEAD_MM   = 25.0"),
    ("SLEW_VY   = 0.25",   "SLEW_VY   = 0.15"),
    # 参数化
    ("        self.cruise   = float(rospy.get_param('~cruise', CRUISE))",
     "        self.cruise   = float(rospy.get_param('~cruise', CRUISE))\n"
     "        self.kp_off   = float(rospy.get_param('~kp_off', KP_OFF))\n"
     "        self.dead_mm  = float(rospy.get_param('~dead_mm', DEAD_MM))\n"
     "        self.max_vy   = float(rospy.get_param('~max_vy', MAX_VY))\n"
     "        self.slew_vy  = float(rospy.get_param('~slew_vy', SLEW_VY))\n"
     "        rospy.logwarn('[servo] 增益: kp_off=%.5f dead=%.0fmm max_vy=%.3f slew=%.2f cruise=%.3f kp_ang=%.4f' % (\n"
     "            self.kp_off, self.dead_mm, self.max_vy, self.slew_vy, self.cruise, self.kp_ang))"),
    # 控制律里用参数
    ("        vy_t  = 0.0 if abs(off) <= DEAD_MM else KP_OFF * off * self.vy_sign",
     "        vy_t  = 0.0 if abs(off) <= self.dead_mm else self.kp_off * off * self.vy_sign"),
    ("        vy_t  = max(-MAX_VY,  min(MAX_VY,  vy_t))",
     "        vy_t  = max(-self.max_vy, min(self.max_vy, vy_t))"),
    ("        self.vy_cmd  = self._slew(self.vy_cmd,  vy_t,  SLEW_VY,  dt)",
     "        self.vy_cmd  = self._slew(self.vy_cmd,  vy_t,  self.slew_vy,  dt)"),
]
for old, new in edits:
    n = s.count(old)
    print('  %s  <- 匹配 %d 处' % (old.strip()[:38], n))
    if n == 1:
        s = s.replace(old, new)
open(p, 'w', encoding='utf-8').write(s)
