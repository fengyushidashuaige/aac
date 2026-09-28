import os
p = os.path.expanduser('~/guttermap/gutter_track.py')
s = open(p, encoding='utf-8').read()
n = 0
old = "        self.use_cam_head = bool(rospy.get_param('~use_cam_heading', False))"
new = ("        self.use_cam_head = bool(rospy.get_param('~use_cam_heading', False))\n"
       "        self.log_dt = 1.0 / max(float(rospy.get_param('~log_hz', 10.0)), 1.0)")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
old2 = "        if now - self.last_print > 1.0:"
c = s.count(old2)
if c >= 1:
    s = s.replace(old2, "        if now - self.last_print > self.log_dt:"); n += 1
if n:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处 (log 行 %d 处)' % (n, c))
