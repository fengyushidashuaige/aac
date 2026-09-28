#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""按指定时间窗分析. 用法: python2 hover_win.py <bag> <t0> <t1>"""
import sys
import numpy as np
import rosbag

def q2R(q):
    x,y,z,w=q; n=max(np.linalg.norm(q),1e-9); x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

t0=float(sys.argv[2]); t1=float(sys.argv[3])
bag=rosbag.Bag(sys.argv[1]); T,X,Y,Z,YW=[],[],[],[],[]
for tp,m,t in bag.read_messages(topics=['/Odometry']):
    p=m.pose.pose.position; q=m.pose.pose.orientation
    R=q2R((q.x,q.y,q.z,q.w))
    T.append(t.to_sec()); X.append(p.x); Y.append(p.y); Z.append(p.z)
    YW.append(np.degrees(np.arctan2(R[1,0],R[0,0])))
bag.close()
T=np.array(T); X=np.array(X); Y=np.array(Y); Z=np.array(Z)
YW=np.degrees(np.unwrap(np.radians(np.array(YW))))
m=(T-T[0]>=t0)&(T-T[0]<=t1)
t=T[m]-T[m][0]; x=X[m]; y=Y[m]; z=Z[m]; w=YW[m]
print('窗口 %.0f~%.0fs   样本 %d   z %.2f~%.2f'%(t0,t1,len(t),z.min(),z.max()))
for nm,v,sc in (('x',x,1000),('y',y,1000),('z',z,1000),('yaw',w,1)):
    A=np.column_stack([t,np.ones(len(v))]); sl,_,_,_=np.linalg.lstsq(A,v)
    res=v-A.dot(sl)
    print('  %-4s 漂移 %+7.2f /s (共 %+8.1f) | 去趋势抖动 %8.2f | 范围 %+8.1f ~ %+8.1f'%(
        nm, sl[0]*sc, sl[0]*sc*(t[-1]-t[0]), res.std()*sc,
        (v-v.mean()).min()*sc, (v-v.mean()).max()*sc))
print()
print('  水平总位移 %.0f mm   离窗口起点最远 %.0f mm   机头转 %.1f deg'%(
    1000*np.hypot(x[-1]-x[0],y[-1]-y[0]), 1000*np.hypot(x-x[0],y-y[0]).max(), w[-1]-w[0]))
