#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""记录飞控状态变化(模式/解锁/连接) + 时间轴, 供事后与轨迹对齐"""
import rospy, sys, time
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry

last = {}
start_wall = time.time()
def cb(m):
    cur = (m.mode, bool(m.armed), bool(m.connected))
    if cur != last.get('v'):
        last['v'] = cur
        print('[EVT] t_rel=%7.2fs  mode=%-10s armed=%-5s connected=%s' % (
            time.time()-start_wall, m.mode, m.armed, m.connected))
        sys.stdout.flush()

def cb_odom(m):
    p = m.pose.pose.position
    last['p'] = (p.x, p.y, p.z)

rospy.init_node('flight_events', anonymous=True, disable_signals=True)
rospy.Subscriber('/mavros/state', State, cb, queue_size=20)
rospy.Subscriber('/Odometry', Odometry, cb_odom, queue_size=20)
print('[EVT] 开始记录 (t_rel 基准 %.3f)' % start_wall); sys.stdout.flush()
r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
    if 'p' in last:
        p = last['p']
        print('[POS] t_rel=%7.2fs  x=%+.3f y=%+.3f z=%+.3f' % (
            time.time()-start_wall, p[0], p[1], p[2]))
        sys.stdout.flush()
