import os, difflib
p = os.path.expanduser('~/guttermap/gutter_track.py')
src = open(p, encoding='utf-8').read()
old = "                                 math.degrees(_c.heading_err)))\n"
new = ("                                 math.degrees(_c.heading_err)))\n"
       "                rospy.loginfo('[track] (idle) fix=%+.0fmm  cam=%s  off_lp=%+.0fmm'\n"
       "                              % (1000 * self.lat_shift,\n"
       "                                 ('%+.0f' % self.cam_off) if self.cam_off is not None else '--',\n"
       "                                 1000 * self.off_lp))\n")
if src.count(old) != 1:
    print('!! 匹配 %d 次' % src.count(old))
else:
    open(p, 'w', encoding='utf-8').write(src.replace(old, new))
    print('已加入未使能时的 fix/cam 打印')
