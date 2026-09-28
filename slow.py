import os
p = os.path.expanduser('~/guttermap/gutter_track.py')
s = open(p, encoding='utf-8').read()
old = """        TAU = 2.5
        dt = 1.0 / max(self.rate_hz, 1.0)
        # 门3: 飞机不在路径附近时不许学(停在一边时看到的槽不能用)
        if abs(self.cam_off) > 250.0 or abs(c.cte_raw) > 0.40:
            return"""
new = """        TAU = float(rospy.get_param('~cam_tau', 15.0))   # 慢: 滞后~0.5s, 15:1 裕度足够
        dt = 1.0 / max(self.rate_hz, 1.0)
        # 门: 只卡"偏差本身不可信"(坏 mask); 不卡 cte —— 路径偏了也要能纠回来(鲁棒性)
        if abs(self.cam_off) > 250.0:
            return"""
n = 1 if s.count(old) == 1 else 0
if n: s = s.replace(old, new)
old2 = """        _d = a * (tgt - self.lat_shift)
        _cap = 0.10 * dt                  # 变化率限幅 100mm/s
        self.lat_shift += max(-_cap, min(_cap, _d))
        self.lat_shift = max(-0.5, min(0.5, self.lat_shift))"""
new2 = """        _d = a * (tgt - self.lat_shift)
        _cap = float(rospy.get_param('~cam_rate', 0.01)) * dt   # 10mm/s, 顶不上去
        self.lat_shift += max(-_cap, min(_cap, _d))
        _lim = float(rospy.get_param('~cam_lim', 0.5))
        self.lat_shift = max(-_lim, min(_lim, self.lat_shift))"""
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
# 默认开启(这才是鲁棒性的来源)
old3 = "rospy.get_param('~use_cam', False)"
if s.count(old3) == 1:
    s = s.replace(old3, "rospy.get_param('~use_cam', True)"); n += 1
if n:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
