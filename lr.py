import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
n = 0
old = "        self.kd_off   = float(rospy.get_param('~kd_off', KD_OFF))"
new = ("        self.kd_off   = float(rospy.get_param('~kd_off', KD_OFF))\n"
       "        _lhz = float(rospy.get_param('~log_hz', 10.0))\n"
       "        self.log_dt = 1.0 / max(_lhz, 1.0)")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
old2 = "        if now - self.last_print > 1.0:"
new2 = "        if now - self.last_print > self.log_dt:"
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
if n:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
