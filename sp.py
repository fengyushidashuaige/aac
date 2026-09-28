import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
n = 0
old = "        self.kp_ang   = float(rospy.get_param('~kp_ang', KP_ANG))"
new = ("        self.kp_ang   = float(rospy.get_param('~kp_ang', KP_ANG))\n"
       "        self.cruise   = float(rospy.get_param('~cruise', CRUISE))")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
old2 = ("            a = min(1.0, abs(off) / 150.0); b = min(1.0, abs(head) / 20.0)\n"
        "            vx = CRUISE * max(0.25, 1.0 - max(a, b))")
new2 = ("            # 只用偏差减速(不用角度): v2 模型下 ang 常年偏大, 用它会永远卡在最低速\n"
        "            vx = self.cruise * max(0.5, 1.0 - min(1.0, abs(off) / 250.0))")
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
if n:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
