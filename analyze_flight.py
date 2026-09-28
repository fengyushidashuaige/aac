#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""飞行轨迹分析: 读 bag -> 轨迹俯视图 / 闭合误差 / 漂移 / 高度剖面"""
import sys, os, math, numpy as np, rosbag
from nav_msgs.msg import Odometry

path = sys.argv[1] if len(sys.argv) > 1 else None
try:
    bag = rosbag.Bag(path)
except Exception:
    bag = rosbag.Bag(path, allow_unindexed=True)   # 读未收尾的 .active
P = []; Q = []; T = []
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    p = msg.pose.pose.position; q = msg.pose.pose.orientation
    P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
    T.append(t.to_sec())
bag.close()
if len(P) < 10:
    print('!! /Odometry 样本太少: %d' % len(P)); sys.exit(1)

A = np.array(P); T = np.array(T)
dur = T[-1] - T[0]
rate = len(A) / max(dur, 1e-6)
print('=' * 58)
print('轨迹分析  %s' % path)
print('=' * 58)
print('样本 %d   时长 %.1fs   采样率 %.1f Hz' % (len(A), dur, rate))
print()
print('--- 位置 ---')
for i, ax in enumerate('xyz'):
    print('  %s  起 %+.3f  终 %+.3f  范围 [%+.3f, %+.3f]' % (
        ax, A[0,i], A[-1,i], A[:,i].min(), A[:,i].max()))
d = np.linalg.norm(np.diff(A, axis=0), axis=1)
print('  路径总长 %.3f m   位移 %.3f m' % (d.sum(), np.linalg.norm(A[-1]-A[0])))
print()
print('--- 闭合误差 (终点 vs 起点) ---')
c = A[-1] - A[0]
print('  dx=%+.3f dy=%+.3f dz=%+.3f   |闭合|=%.3f m' % (c[0], c[1], c[2], np.linalg.norm(c)))
print()
print('--- 俯视图 (x-y, 上=+y) ---')
W, H = 58, 22
x0, x1 = A[:,0].min(), A[:,0].max(); y0, y1 = A[:,1].min(), A[:,1].max()
sx = max(x1-x0, 1e-6); sy = max(y1-y0, 1e-6)
grid = [[' ']*W for _ in range(H)]
for i, p in enumerate(A):
    gx = int((p[0]-x0)/sx*(W-1)); gy = int((p[1]-y0)/sy*(H-1))
    gy = H-1-gy
    grid[gy][gx] = 'o'
gs, ge = int((A[0,0]-x0)/sx*(W-1)), H-1-int((A[0,1]-y0)/sy*(H-1))
grid[ge][gs] = 'S'
gf, ge2 = int((A[-1,0]-x0)/sx*(W-1)), H-1-int((A[-1,1]-y0)/sy*(H-1))
grid[ge2][gf] = 'E'
print('  x: %.3f -> %.3f m   (一格 %.4f m)' % (x0, x1, sx/(W-1)))
for r in grid: print('  |' + ''.join(r) + '|')
print('  y: %.3f -> %.3f m   (一格 %.4f m)   S=起点 E=终点' % (y0, y1, sy/(H-1)))
print()
print('--- 漂移 vs 时间 (相对起点的水平距离) ---')
for f in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
    i = min(int(f*(len(A)-1)), len(A)-1)
    print('  t=%6.1fs  水平漂移 %.3f m   高度 %+.3f m' % (
        T[i]-T[0], math.hypot(A[i,0]-A[0,0], A[i,1]-A[0,1]), A[i,2]))
print()
z = A[:,2]
print('--- 高度剖面 ---')
print('  min %+.3f   max %+.3f   峰峰 %.3f m' % (z.min(), z.max(), z.max()-z.min()))
np.savetxt(os.path.join(os.path.dirname(os.path.abspath(path)), 'traj.csv'), np.column_stack([T-T[0], A]),
           delimiter=',', header='t,x,y,z', comments='', fmt='%.4f')
print()
print('轨迹已存 %s' % os.path.join(os.path.dirname(os.path.abspath(path)), 'traj.csv'))
