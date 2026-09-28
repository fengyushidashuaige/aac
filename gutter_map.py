#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""阶段2-B: 收 UDP + 订 /Odometry -> 投影到世界系 -> 发点云/Marker (python2 ROS)"""
import rospy, socket, json, math, sys, os, time, collections, numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

PORT      = 45678
CAM_DOWN  = 0.55      # 相机镜头到水槽的垂直距离 (m), 只影响绘制高度
CAM_YAW   = 0.0       # 相机相对机体的安装偏航 (rad); 0 = 假定对齐
MIN_CONF  = 0.6       # 与飞行代码一致的门限 (见 NEXT_STEPS.md 9.1)

pose = {'R': np.eye(3), 't': np.zeros(3), 'ok': False}
PBUF = collections.deque(maxlen=900)      # 位姿历史 (~90s @10Hz), 用于按帧时刻回查

def _stamp_of(m):
    try:
        v = m.header.stamp.to_sec()
        if v > 1e6: return v
    except Exception:
        pass
    return time.time()

def pose_at(ts):
    """取 ts 时刻的位姿 (线性插值位置; 姿态取最近). ts 早于缓冲则取最旧."""
    if not PBUF: return None, None
    if ts >= PBUF[-1][0]: return PBUF[-1][1], PBUF[-1][2]
    if ts <= PBUF[0][0]:  return PBUF[0][1],  PBUF[0][2]
    lo = 0
    for i in range(len(PBUF) - 1, 0, -1):
        if PBUF[i][0] <= ts: lo = i; break
    t0, R0, p0 = PBUF[lo]
    t1, R1, p1 = PBUF[min(lo + 1, len(PBUF) - 1)]
    dt = t1 - t0
    a = 0.0 if dt < 1e-9 else min(1.0, max(0.0, (ts - t0) / dt))
    return R0, p0 + a * (p1 - p0)
def odom_cb(m):
    p = m.pose.pose.position; q = m.pose.pose.orientation
    x,y,z,w = q.x,q.y,q.z,q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
    pose['R'] = R; pose['t'] = np.array([p.x,p.y,p.z]); pose['ok'] = True
    PBUF.append((_stamp_of(m), R, np.array([p.x,p.y,p.z])))
    if len(TRAJ) == 0 or (abs(TRAJ[-1][0]-p.x)+abs(TRAJ[-1][1]-p.y)) > 0.02:
        TRAJ.append((p.x, p.y))
        if len(TRAJ) > 20000: del TRAJ[:5000]

rospy.init_node('gutter_map', anonymous=True, disable_signals=True)
rospy.Subscriber('/Odometry', Odometry, odom_cb, queue_size=50)
pub_pc  = rospy.Publisher('/gutter_map/points', PointCloud2, queue_size=10)
pub_mk  = rospy.Publisher('/gutter_map/markers', MarkerArray, queue_size=10)

MAPFILE = '/home/amov/guttermap/run/map_view.jpg'
TRAJ = []          # 飞机轨迹 (世界系 x,y)

def render_map(pts, traj):
    """俯视图: 飞机轨迹(橙) + 水槽点(红), 1 米网格"""
    if len(pts) < 2:
        return
    A = np.array(pts)
    T = np.array(traj) if len(traj) > 1 else None
    xs = [A[:,0].min(), A[:,0].max()]; ys = [A[:,1].min(), A[:,1].max()]
    if T is not None:
        xs += [T[:,0].min(), T[:,0].max()]; ys += [T[:,1].min(), T[:,1].max()]
    x0, x1 = xs[0]-0.5, xs[1]+0.5; y0, y1 = ys[0]-0.5, ys[1]+0.5
    W, H = 700, 520
    sc = min(W/(x1-x0), H/(y1-y0))
    ox = int((W-(x1-x0)*sc)/2); oy = int((H-(y1-y0)*sc)/2)
    def tp(x, y): return (int(ox+(x-x0)*sc), int(H-oy-(y-y0)*sc))
    img = np.full((H, W, 3), 30, np.uint8)
    for g in np.arange(math.ceil(x0), x1, 1.0):
        a = tp(g, y0); b = tp(g, y1); cv2.line(img, a, b, (60,60,60), 1)
    for g in np.arange(math.ceil(y0), y1, 1.0):
        a = tp(x0, g); b = tp(x1, g); cv2.line(img, a, b, (60,60,60), 1)
    if T is not None:
        for i in range(1, len(T)):
            cv2.line(img, tp(T[i-1][0], T[i-1][1]), tp(T[i][0], T[i][1]), (200,150,70), 1)
    for w in A:
        cv2.circle(img, tp(w[0], w[1]), 2, (70,70,240), -1)
    # 线性度
    c = A.mean(axis=0)
    try:
        u, sv, vt = np.linalg.svd(A - c, full_matrices=False)
        perp = (A-c) - np.outer((A-c).dot(vt[0]), vt[0])
        res = 1000*np.median(np.linalg.norm(perp, axis=1))
        txt = 'points %d   scatter %.0f mm  (1m grid)' % (len(A), res)
        col = (90,220,90) if res < 40 else (90,180,240)
    except Exception:
        txt = 'points %d' % len(A); col = (170,170,170)
    cv2.putText(img, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.imwrite(MAPFILE, img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', PORT)); s.settimeout(1.0)

pts = []; skipped = [0]; last_render = [0.0]
print('[map] waiting: UDP %d + /Odometry' % PORT); sys.stdout.flush()
r = rospy.Rate(20)
last_pub = rospy.Time.now()
while not rospy.is_shutdown():
    try:
        data, _ = s.recvfrom(1024)
        d = json.loads(data.decode('utf-8'))
        if not d.get('detected') or d.get('held'):
            continue
        if d.get('conf', 0.0) < MIN_CONF:      # 剔除吹风筒误检
            skipped[0] += 1
            continue
        if not pose['ok']:
            continue
        off = d['offset_mm'] / 1000.0
        # ★ 按【帧时刻】回查位姿, 而不是用"最新"位姿
        _ts = d.get('t', None)
        R_use, t_use = (pose['R'], pose['t']) if _ts is None else pose_at(_ts)
        if R_use is None:
            R_use, t_use = pose['R'], pose['t']
        p_b = np.array([off*math.sin(CAM_YAW), -off*math.cos(CAM_YAW), -CAM_DOWN])
        p_w = R_use.dot(p_b) + t_use
        pts.append(p_w)
        if len(pts) > 4000:
            pts = pts[-4000:]
    except socket.timeout:
        pass

    if (rospy.Time.now() - last_pub).to_sec() > 0.5 and pts:
        last_pub = rospy.Time.now()
        hdr = Header(); hdr.stamp = rospy.Time.now(); hdr.frame_id = 'camera_init'
        pub_pc.publish(pc2.create_cloud_xyz32(hdr, [tuple(p) for p in pts]))
        A = np.array(pts)
        mk = Marker()
        mk.header = hdr; mk.ns='gutter'; mk.id=0
        mk.type = Marker.LINE_STRIP; mk.action = Marker.ADD
        mk.scale.x = 0.01
        mk.color.r = 1.0; mk.color.g = 0.2; mk.color.b = 0.0; mk.color.a = 1.0
        mk.pose.orientation.w = 1.0
        mk.points = [Point(*p) for p in pts[::max(1,len(pts)//200)]]
        ma = MarkerArray(); ma.markers.append(mk)
        mk2 = Marker()
        mk2.header = hdr; mk2.ns='gutter'; mk2.id=1
        mk2.type = Marker.SPHERE_LIST; mk2.action = Marker.ADD
        mk2.scale.x = mk2.scale.y = mk2.scale.z = 0.01
        mk2.color.g = 1.0; mk2.color.a = 1.0
        mk2.pose.orientation.w = 1.0
        mk2.points = [Point(*p) for p in pts[::max(1,len(pts)//300)]]
        ma.markers.append(mk2)
        pub_mk.publish(ma)
        if time.time() - last_render[0] > 1.0:
            last_render[0] = time.time()
            try: render_map(pts, TRAJ)
            except Exception: pass
        print('[map] %d 点 (剔除低conf %d)  x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
            skipped[0],
            len(pts), A[:,0].min(), A[:,0].max(), A[:,1].min(), A[:,1].max(),
            A[:,2].min(), A[:,2].max()))
        sys.stdout.flush()
