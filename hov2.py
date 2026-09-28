import os
p = os.path.expanduser('~/guttermap/hover_still.py')
open(p,'w').write('''#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""只看【真正静止】的那段: 速度很小的最长窗口. 用法: python2 hover_still.py <bag> [v阈值mm/s]"""
import sys
import numpy as np
import rosbag

def q2R(q):
    x,y,z,w=q; n=max(np.linalg.norm(q),1e-9); x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

THRV = (float(sys.argv[2]) if len(sys.argv)>2 else 40.0)/1000.0
bag = rosbag.Bag(sys.argv[1])
T,X,Y,Z,YW = [],[],[],[],[]
for tp,m,t in bag.read_messages(topics=['/Odometry']):
    p=m.pose.pose.position; q=m.pose.pose.orientation
    R=q2R((q.x,q.y,q.z,q.w))
    T.append(t.to_sec()); X.append(p.x); Y.append(p.y); Z.append(p.z)
    YW.append(np.degrees(np.arctan2(R[1,0],R[0,0])))
bag.close()
T=np.array(T); X=np.array(X); Y=np.array(Y); Z=np.array(Z)
YW=np.degrees(np.unwrap(np.radians(np.array(YW))))
if len(T)<5: sys.exit('点太少')
dt=np.median(np.diff(T))
v=np.hypot(np.gradient(X,T),np.gradient(Y,T))
air = Z > 0.20
still = air & (v < THRV) & (np.abs(np.gradient(YW,T)) < 3.0)
print('总长 %.0fs  空中 %.0fs  速度<%dmm/s且转向<%d deg/s 的帧 %d'%(
    T[-1]-T[0], air.sum()*dt, THRV*1000, 3, still.sum()))
# 最长连续段
best=(0,0,0); i=0
while i<len(still):
    if still[i]:
        j=i
        while j<len(still) and still[j]: j+=1
        if T[j-1]-T[i]>best[0]: best=(T[j-1]-T[i],i,j)
        i=j
    else: i+=1
d,i0,i1=best
if d<5: sys.exit('没找到 >5s 的静止段')
t=T[i0:i1]-T[i0]; x=X[i0:i1]-X[i0]; y=Y[i0:i1]-Y[i0]; z=Z[i0:i1]; w=YW[i0:i1]
print()
print('=== ★真正静止段 %.1fs (OFFBOARD 零指令悬停) ==='%d)
for nm,vv,sc in (('x',x,1000),('y',y,1000),('z',z,1000),('yaw',w,1)):
    A=np.column_stack([t,np.ones(len(vv))]); sl,_,_,_=np.linalg.lstsq(A,vv)
    res=vv-A.dot(sl)
    print('  %-4s 漂移 %+7.2f /s (共 %+7.1f) | 抖动 %7.2f | 范围 %+7.1f ~ %+7.1f'%(
        nm, sl[0]*sc, sl[0]*sc*d, res.std()*sc, (vv-vv.mean()).min()*sc, (vv-vv.mean()).max()*sc))
print()
print('  水平位移: 总 %.0f mm   最大偏离 %.0f mm'%(
    1000*np.hypot(x[-1],y[-1]), 1000*np.hypot(x,y).max()))
print('  机头总转角: %.1f deg   抖动 %.2f deg'%(w[-1]-w[0], w.std()))
''')
print('ok')
