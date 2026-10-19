#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FAST-LIO -> PX4 vision_pose 桥接

订阅 /Odometry (FAST-LIO, frame_id=camera_init) -> 发 /mavros/vision_pose/pose

坐标系约定(已查 FAST_LIO 源码 IMU_Processing.hpp:198 确认):
  FAST-LIO 的 camera_init 世界系 = 开机瞬间的 IMU 机体坐标系。
  设置初始旋转的那行 `//state_inout.rot = Eye3d;` 是注释掉的, 只设了 grav,
  所以 init_state.rot 保持单位阵 => 世界系与机体是固定的单位关系。

  => 开机时飞机摆平, 输出姿态就是真实姿态, **直接透传即可**。
  => 与 VINS 不同(VINS 世界系是相机初始朝向, 恒有固定假 roll, 必须修正)。

模式 (参数 ~mode):
  passthrough  默认。原样转发。要求: 开机时飞机水平静止, 机头朝向一致。
  auto_cal     开机采样 CAL_SECS 秒静止姿态作基准, 减掉它。飞机开机姿态不固定时用。
  fixed        用固定四元数 ~qx ~qy ~qz ~qw 作修正(手工标定过的固定值)。

用法: python fastlio_to_mavros.py
      rosrun ... _mode:=auto_cal _cal_secs:=2.5
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point

def qmul(a, b):
    """Hamilton 四元数乘法 a*b, 各元素 (w,x,y,z)"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def qrot(q, v):
    """用单位四元数 q(w,x,y,z) 旋转向量 v(x,y,z)"""
    w, x, y, z = q
    ux, uy, uz = x, y, z
    vx, vy, vz = v
    cx = uy*vz - uz*vy
    cy = uz*vx - ux*vz
    cz = ux*vy - uy*vx
    u_x_c = (uy*cz - uz*cy, uz*cx - ux*cz, ux*cy - uy*cx)
    return (vx + 2*w*cx + 2*u_x_c[0],
            vy + 2*w*cy + 2*u_x_c[1],
            vz + 2*w*cz + 2*u_x_c[2])

pub = None
_n = [0]
_cal = {'start': None, 'last': None, 'done': False}
CORR = (1.0, 0.0, 0.0, 0.0)

def cb(msg):
    global pub, CORR
    o = msg.pose.pose.orientation
    qv = (o.w, o.x, o.y, o.z)
    p = msg.pose.pose.position

    if MODE == 'auto_cal':
        now = rospy.Time.now().to_sec()
        if _cal['start'] is None:
            _cal['start'] = now
            rospy.loginfo('auto_cal: sampling %.1fs, keep aircraft LEVEL and STILL', CAL_SECS)
        _cal['last'] = qv
        if not _cal['done']:
            if now - _cal['start'] < CAL_SECS:
                return                      # 采样中, 不转发
            w, x, y, z = _cal['last']
            CORR = (w, -x, -y, -z)
            _cal['done'] = True
            rospy.loginfo('auto_cal DONE: q_corr=(%.4f,%.4f,%.4f,%.4f)', *CORR)

    if MODE == 'passthrough':
        qw, qx, qy, qz = qv
        px, py, pz = p.x, p.y, p.z
    else:
        qc = qmul(CORR, qv)
        qw, qx, qy, qz = qc[0], qc[1], qc[2], qc[3]
        px, py, pz = qrot(CORR, (p.x, p.y, p.z))

    ps = PoseStamped()
    ps.header.stamp = msg.header.stamp
    ps.header.frame_id = "map"
    ps.pose.position = Point(px, py, pz)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    pub.publish(ps)

    _n[0] += 1
    if _n[0] % 100 == 1:
        rospy.loginfo('sent %d, pos (%.2f, %.2f, %.2f)', _n[0], px, py, pz)

rospy.init_node('fastlio_to_mavros', anonymous=True)
MODE = rospy.get_param('~mode', 'passthrough').lower()
CAL_SECS = float(rospy.get_param('~cal_secs', 2.5))
odom_topic = rospy.get_param('~odom_topic', '/Odometry')
if MODE == 'fixed':
    CORR = (float(rospy.get_param('~qw', 1.0)), float(rospy.get_param('~qx', 0.0)),
            float(rospy.get_param('~qy', 0.0)), float(rospy.get_param('~qz', 0.0)))
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
rospy.Subscriber(odom_topic, Odometry, cb, queue_size=20)
rospy.loginfo('fastlio_to_mavros up [MODE=%s]: %s -> /mavros/vision_pose/pose', MODE, odom_topic)
rospy.spin()
