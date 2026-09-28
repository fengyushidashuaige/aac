import os
p = os.path.expanduser('~/guttermap/gutter_track.py')
s = open(p, encoding='utf-8').read()
n = 0

# 1) 常数补上(挂 UDP_PORT 那行后面, 用行内锚点)
old = "UDP_PORT = 45679"
if s.count(old) >= 1 and 'K_CAM' not in s.split('\n')[0:60].__str__():
    pass
if 'PSI_TAU' not in s.split('class Tracker')[0]:
    i = s.index('UDP_PORT = 45679')
    j = s.index('\n', i) + 1
    s = s[:j] + ("K_CAM = 0.5        # (停用中) 相机航向项增益\n"
                 "PSI_TAU = 0.6      # (停用中) 航向低通时间常数\n") + s[j:]
    n += 1

# 2) 加开关参数
old2 = "        self.psi_lp = 0.0"
if s.count(old2) == 1:
    s = s.replace(old2, old2 + "\n        self.use_cam_head = bool(rospy.get_param('~use_cam_heading', False))")
    n += 1

# 3) 控制循环里按开关调用
old3 = "        self.update_cam_heading(1.0 / max(self.rate_hz, 1.0))"
if s.count(old3) == 1:
    s = s.replace(old3, "        if self.use_cam_head:\n            self.update_cam_heading(1.0 / max(self.rate_hz, 1.0))")
    n += 1
open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
