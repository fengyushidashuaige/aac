#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""满负载下 /Odometry 稳定性检测: 频率 / 最大间隔(掉帧) / 抖动"""
import rospy, sys, time, numpy as np
from nav_msgs.msg import Odometry
T = []
def cb(m): T.append(time.time())
rospy.init_node('check_odom', anonymous=True, disable_signals=True)
rospy.Subscriber('/Odometry', Odometry, cb, queue_size=100)
print('测 25 秒 ...'); sys.stdout.flush()
t0 = time.time(); r = rospy.Rate(5)
while time.time() - t0 < 25: r.sleep()
if len(T) < 20:
    print('!! 样本太少 %d —— /Odometry 可能已经停了' % len(T)); sys.exit(1)
A = np.array(T); d = np.diff(A)
print('样本 %d   时长 %.1fs   平均 %.2f Hz' % (len(A), A[-1]-A[0], len(A)/(A[-1]-A[0])))
print('间隔: 中位 %.4fs  均值 %.4fs  最大 %.4fs' % (np.median(d), d.mean(), d.max()))
print('抖动(间隔标准差) %.4f s' % d.std())
for thr in (0.12, 0.15, 0.20, 0.30):
    n = int((d > thr).sum())
    print('  间隔 > %.2fs 的次数: %d %s' % (thr, n, '← 掉帧!' if n else 'OK'))
print('判据: 10Hz 期望间隔 0.1s; 最大间隔 >0.2s 即为掉帧')
