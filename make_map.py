#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""把飞行轨迹 + 水槽折线画成俯视图 PNG
用法: python2 make_map.py <bag> <polyline.csv> [输出png]"""
import sys, os
import numpy as np, rosbag, cv2
from nav_msgs.msg import Odometry
BAG = sys.argv[1]
PL  = sys.argv[2]
OUT = sys.argv[3] if len(sys.argv) > 3 else '/home/amov/guttermap/run/map_view_full.png'
bag = rosbag.Bag(BAG); P = []
for tp, m, t in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position; P.append((p.x, p.y))
bag.close()
P = np.array(P)
W = np.loadtxt(PL, delimiter=',', skiprows=1).reshape(-1, 3)
allx = np.concatenate([P[:,0], W[:,0]]); ally = np.concatenate([P[:,1], W[:,1]])
x0,x1 = allx.min()-0.5, allx.max()+0.5; y0,y1 = ally.min()-0.5, ally.max()+0.5
Wpx, Hpx = 1200, 900
sc = min(Wpx/(x1-x0), Hpx/(y1-y0))
ox = int((Wpx-(x1-x0)*sc)/2); oy = int((Hpx-(y1-y0)*sc)/2)
def tp(x,y): return (int(ox+(x-x0)*sc), int(Hpx-oy-(y-y0)*sc))
img = np.full((Hpx,Wpx,3), 28, np.uint8)
for g in np.arange(np.ceil(x0), x1, 1.0):
    cv2.line(img, tp(g,y0), tp(g,y1), (55,55,55), 1)
for g in np.arange(np.ceil(y0), y1, 1.0):
    cv2.line(img, tp(x0,g), tp(x1,g), (55,55,55), 1)
for g in np.arange(np.ceil(x0), x1, 1.0):
    cv2.putText(img, '%.0f'%g, (tp(g,y0)[0]-8, Hpx-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (110,110,110), 1)
for g in np.arange(np.ceil(y0), y1, 1.0):
    cv2.putText(img, '%.0f'%g, (4, tp(x0,g)[1]+4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (110,110,110), 1)
for i in range(1, len(P)):
    cv2.line(img, tp(P[i-1][0],P[i-1][1]), tp(P[i][0],P[i][1]), (200,140,60), 1)
for w in W:
    cv2.circle(img, tp(w[0],w[1]), 2, (60,60,240), -1)
cv2.circle(img, tp(*P[0]), 7, (80,220,80), 2)
cv2.putText(img, 'START', (tp(*P[0])[0]+10, tp(*P[0])[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80,220,80), 2)
cv2.circle(img, tp(*P[-1]), 7, (80,80,240), 2)
cv2.putText(img, 'END', (tp(*P[-1])[0]+10, tp(*P[-1])[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80,80,240), 2)
cv2.putText(img, 'drone path (LiDAR)', (14,26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,140,60), 2)
cv2.putText(img, 'gutter points (%d)' % len(W), (14,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60,60,240), 2)
cv2.putText(img, 'grid = 1 m', (14,74), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140,140,140), 2)
cv2.imwrite(OUT, img)
print('已存 %s   %dx%d   飞机 %d 点  水槽 %d 点' % (OUT,Wpx,Hpx,len(P),len(W)))
