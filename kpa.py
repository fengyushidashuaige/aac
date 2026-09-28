import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
n = 0
old = "        self.use_psi  = bool(rospy.get_param('~use_psi', True))"
new = ("        self.use_psi  = bool(rospy.get_param('~use_psi', True))\n"
       "        self.kp_ang   = float(rospy.get_param('~kp_ang', KP_ANG))\n")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
old2 = "        yaw_t = 0.0 if head == 0.0 else KP_ANG * head * self.yaw_sign"
new2 = "        yaw_t = 0.0 if head == 0.0 else self.kp_ang * head * self.yaw_sign"
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
old3 = "'[servo] off=%+7.1fmm ang=%+6.1f psi=%s conf=%.2f  vx=%.3f vy=%+.3f yaw=%+.3f'"
new3 = "'[servo] off=%+7.1fmm ang=%+6.1f psi=%s conf=%.2f  vx=%.3f vy=%+.3f yaw=%+.3f kpa=%.3f'"
if s.count(old3) == 1:
    s = s.replace(old3, new3); n += 1
old4 = "                          % (off, ang, _ps, self.conf, vx, self.vy_cmd, self.yaw_cmd))"
new4 = "                          % (off, ang, _ps, self.conf, vx, self.vy_cmd, self.yaw_cmd, self.kp_ang))"
if s.count(old4) == 1:
    s = s.replace(old4, new4); n += 1
if n > 0:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
