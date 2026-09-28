#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""用与 build_polyline 完全相同的投影, 扫 CAM_YAW 与 SCALE 找误差来源"""
import math
import numpy as np, rosbag
BAG='/home/amov/logs/flight_20260916_143024/flight.bag.active'
CSV='/home/amov/logs/guttermap_20260916_142937/detect.csv'
CAM_DOWN=0.55
bag=rosbag.Bag(BAG); T=[];Q=[];P=[]
for tp,m,t in bag.read_messages(topics=['/Odometry']):
    p=m.pose.pose.position; q=m.pose.pose.orientation
    T.append(t.to_sec()); P.append((p.x,p.y,p.z)); Q.append((q.x,q.y,q.z,q.w))
bag.close()
T=np.array(T);P=np.array(P);Q=np.array(Q)
raw=open(CSV,'rb').read().replace(b'\x00',b'')
rows=[]
for ln in raw.decode('utf-8','ignore').splitlines()[1:]:
    p=ln.split(',')
    if len(p)<5: continue
    try: rows.append((float(p[0]),float(p[1]),float(p[2]),float(p[3]),int(p[4])))
    except: pass
good=[x for x in rows if x[3]>=0.6]

def project(off, ts):
    i=np.searchsorted(T,ts); i=min(max(i,1),len(T)-1)
    a=(ts-T[i-1])/max(T[i]-T[i-1],1e-6)
    pp=P[i-1]+a*(P[i]-P[i-1]); q=Q[i-1]+a*(Q[i]-Q[i-1])
    x,y,z,w=q/max(np.linalg.norm(q),1e-9)
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return R.dot(off)+pp

def resid(yaw_deg, scale):
    ca=math.cos(math.radians(yaw_deg)); sa=math.sin(math.radians(yaw_deg))
    W=[]
    for ts,off,ang,cf,_ in good:
        if ts<T[0] or ts>T[-1]: continue
        o=off*scale/1000.0
        # 机体系: x前 y左 z上; offset正值=槽在右侧 → y = -o
        # 相机安装偏航 yaw_deg: 绕机体 z 轴旋转
        pb=np.array([(-o)*(-sa), -o*ca, -CAM_DOWN])
        W.append(project(pb, ts))
    if len(W)<20: return 999.0
    A=np.array(W)[:,:2]; c=A.mean(axis=0)
    u,s,vt=np.linalg.svd(A-c, full_matrices=False)
    nrm=np.array([-vt[0][1],vt[0][0]])
    return float(np.median(np.abs((A-c).dot(nrm)))*1000.0)

print('基准 (yaw=0, scale=1): %.1f mm' % resid(0.0,1.0))
print()
print('=== 扫 CAM_YAW ===')
best=None
for y in range(-90,91,10):
    r=resid(float(y),1.0)
    print('  yaw=%+4d°  %.1f mm' % (y,r))
    if best is None or r<best[0]: best=(r,y)
print('  >>> yaw=%+d°  %.1f mm' % (best[1],best[0]))
print()
print('=== 在最优 yaw 下扫 SCALE ===')
b2=None
for sc in (0.6,0.7,0.8,0.85,0.9,0.95,1.0):
    r=resid(float(best[1]),sc)
    print('  scale=%.2f  %.1f mm' % (sc,r))
    if b2 is None or r<b2[0]: b2=(r,sc)
print('  >>> yaw=%+d° scale=%.2f  %.1f mm  (基准 %.1f mm)' % (best[1],b2[1],b2[0],resid(0.0,1.0)))
