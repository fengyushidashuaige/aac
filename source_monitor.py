#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
源监控与告警 —— GPS 与 视觉(雷达) 的一致性检查

原理: 两个源的坐标系原点不同, 所以不比对【绝对位置】,
      而比对【位移】(与初始位置对齐后的差), 这个与坐标系无关。

    Δgps = 当前GPS位置 - 初始GPS位置
    Δvis = 当前视觉位置 - 初始视觉位置
    diff = |Δgps - Δvis|      ← 两者对"走了多远"的分歧

分级:
    diff < WARN_M   : OK
    WARN_M ~ ALARM_M: ⚠ 偏大, 留意
    > ALARM_M       : ✗✗ 严重, 建议停飞

只告警, 不发控制指令 (判断权留给人)。
"""
import math, json, time
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from std_msgs.msg import String

WARN_M   = 1.0      # 分歧告警阈值 (m)
ALARM_M  = 3.0      # 分歧严重阈值 (m)
STALE_S  = 1.0      # 数据超时 (s)
MIN_SATS = 6        # GPS 最少星数
MIN_FIX  = 3        # GPS 最低 fix_type (3=3D)


class Monitor(object):
    def __init__(self):
        self.gps = None;  self.t_gps = 0.0
        self.vis = None;  self.t_vis = 0.0
        self.gps0 = None; self.vis0 = None
        self.sats = 0; self.fix = 0
        self.vis_rate = 0.0; self._vc = 0; self._vt = time.time()
        self.last_print = 0.0
        self.worst = 0.0

        rospy.Subscriber('/mavros/global_position/local', PoseStamped, self.on_gps, queue_size=5)
        rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, self.on_gpsraw, queue_size=5)
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.on_vis, queue_size=5)
        self.pub = rospy.Publisher('/source_monitor/status', String, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.tick)
        rospy.logwarn('=== 源监控启动: 告警阈值 %.1fm / 严重 %.1fm ===' % (WARN_M, ALARM_M))
        rospy.logwarn('    等待 GPS 与 视觉 数据...')

    def on_gps(self, m):
        p = m.pose.position
        self.gps = (p.x, p.y, p.z); self.t_gps = time.time()
        if self.gps0 is None:
            self.gps0 = self.gps
            rospy.loginfo('[monitor] 记下 GPS 初始位置 (%.2f, %.2f, %.2f)' % self.gps)

    def on_gpsraw(self, m):
        # mavros_msgs/GPSRAW: fix_type, satellites_visible
        self.fix = int(m.fix_type)
        self.sats = int(m.satellites_visible)

    def on_vis(self, m):
        p = m.pose.position
        self.vis = (p.x, p.y, p.z); self.t_vis = time.time()
        self._vc += 1
        if self.vis0 is None:
            self.vis0 = self.vis
            rospy.loginfo('[monitor] 记下 视觉 初始位置 (%.2f, %.2f, %.2f)' % self.vis)

    def tick(self, ev):
        now = time.time()
        if now - self._vt > 2.0:
            self.vis_rate = self._vc / (now - self._vt); self._vc = 0; self._vt = now

        g_ok = self.gps is not None and (now - self.t_gps) < STALE_S
        v_ok = self.vis is not None and (now - self.t_vis) < STALE_S

        # ---- 分歧计算 (只在两个源都新鲜时才有效) ----
        diff = None; dg = dv = None
        if g_ok and v_ok and self.gps0 and self.vis0:
            dg = math.hypot(self.gps[0]-self.gps0[0], self.gps[1]-self.gps0[1])
            dv = math.hypot(self.vis[0]-self.vis0[0], self.vis[1]-self.vis0[1])
            diff = abs(dg - dv)
            self.worst = max(self.worst, diff)

        # ---- 状态行 ----
        if now - self.last_print > 1.0:
            self.last_print = now
            s = []
            s.append('GPS:%s %d星 fix=%d' % ('OK' if g_ok else '✗无数据', self.sats, self.fix))
            s.append('视觉:%s(%.1fHz)' % ('OK %.1fs' % (now-self.t_vis) if v_ok else '✗无数据', self.vis_rate))
            if diff is not None:
                mark = '✓' if diff < WARN_M else ('⚠' if diff < ALARM_M else '✗✗')
                s.append('位移 Δgps=%.2fm Δvis=%.2fm 分歧=%.2fm %s' % (dg, dv, diff, mark))
            else:
                s.append('分歧: (等两个源都有数据)')
            rospy.loginfo('[monitor] ' + ' | '.join(s))
            _st = {'gps_ok': g_ok, 'vis_ok': v_ok, 'vis_hz': round(self.vis_rate, 2),
                   'sats': self.sats, 'fix': self.fix,
                   'dgps': dg, 'dvis': dv, 'diff': diff, 'worst': round(self.worst, 2),
                   't': now}
            self.pub.publish(String(data=json.dumps(_st)))
            try:                                  # 给预览页读
                with open('/home/amov/guttermap/run/monitor.json', 'w') as _f:
                    json.dump(_st, _f)
            except Exception:
                pass

        # ---- 分级告警 ----
        if diff is not None:
            if diff >= ALARM_M:
                rospy.logerr_throttle(2.0,
                    '[monitor] ✗✗ 分歧 %.2fm 严重! GPS说走%.1fm, 视觉说走%.1fm '
                    '—— 有一个飘了, 建议停飞' % (diff, dg, dv))
            elif diff >= WARN_M:
                rospy.logwarn_throttle(3.0,
                    '[monitor] ⚠ 分歧 %.2fm 偏大 (GPS %.1fm / 视觉 %.1fm), 留意' % (diff, dg, dv))
        if g_ok and (self.fix < MIN_FIX or self.sats < MIN_SATS):
            rospy.logwarn_throttle(5.0, '[monitor] ⚠ GPS 弱: %d星 fix=%d (需要 >=%d星 fix>=%d)'
                                   % (self.sats, self.fix, MIN_SATS, MIN_FIX))
        if not g_ok and not v_ok:
            rospy.logerr_throttle(2.0, '[monitor] ✗✗ 两个源都没数据! 位置不可信')
        elif not v_ok:
            rospy.logwarn_throttle(3.0, '[monitor] ⚠ 视觉无数据 (只剩 GPS, 局部精度丢失)')
        elif not g_ok:
            rospy.logwarn_throttle(3.0, '[monitor] ⚠ GPS 无数据 (只剩视觉, 会漂)')


if __name__ == '__main__':
    rospy.init_node('source_monitor')
    Monitor()
    rospy.spin()
