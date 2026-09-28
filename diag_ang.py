#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
验证用户假设: 检测到的 angle_deg 是否 == 机头相对水槽的偏航 (heading error)?

几何: 相机俯角 al, 画面里槽中心线的倾角 tilt 满足
        tan(tilt) = tan(psi) * sin(al)
      (psi = 水槽方向相对机头的偏航). 小角时 tilt ≈ psi*sin(al).
所以: angle_meas(t) 应与 雷达 yaw(t) 强相关, 斜率 = -sin(al) (符号看约定).
回归的截距 ~ 水槽的真实世界朝向 + 相机安装偏航.
"""
import sys
import numpy as np
import rosbag

BAG = sys.argv[1]
CSVF = sys.argv[2]
MIN_CONF = 0.4


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


bag = rosbag.Bag(BAG)
T, P, Q = [], [], []
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
bag.close()
T = np.array(T); P = np.array(P)
Rall = np.array([quat_to_R(q) for q in Q])
yaw = np.degrees(np.arctan2(Rall[:, 1, 0], Rall[:, 0, 0]))

raw = open(CSVF, 'rb').read().replace(b'\x00', b'')
good = []
for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
    p = ln.split(',')
    if len(p) < 7:
        continue
    try:
        r = (float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]), float(p[6]))
    except Exception:
        continue
    if r[4] == 1 and r[3] >= MIN_CONF and T[0] <= r[0] <= T[-1]:
        good.append(r)

td = np.array([r[0] for r in good])
ang = np.array([r[2] for r in good])
off = np.array([r[1] for r in good]) / 1000.0
wpx = np.array([r[5] for r in good])

yi = np.interp(td, T, yaw)


def corr(a, b):
    a = a - a.mean(); b = b - b.mean()
    return float(a.dot(b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-12))


print('== angle_deg  vs  雷达 yaw ==')
print('   n = %d' % len(td))
print('   angle: 中位 %+.2f  std %.2f deg' % (np.median(ang), ang.std()))
print('   yaw  : 中位 %+.2f  std %.2f deg' % (np.median(yi), yi.std()))
print('   相关系数 corr(angle, yaw)        = %+.3f' % corr(ang, yi))
print('   相关系数 corr(angle, -yaw)       = %+.3f' % corr(ang, -yi))

# 回归 angle = s*yaw + b
A = np.column_stack([yi, np.ones(len(yi))])
sol, _, _, _ = np.linalg.lstsq(A, ang)
pred = A.dot(sol)
r2 = 1 - np.sum((ang - pred) ** 2) / max(np.sum((ang - ang.mean()) ** 2), 1e-12)
print('   回归 angle = %+.3f * yaw  %+.2f     R2 = %.3f' % (sol[0], sol[1], r2))
print('   -> |斜率| = |sin(相机俯角)| = %.3f  -> 俯角 = %.1f deg' % (
    abs(sol[0]), np.degrees(np.arcsin(min(1.0, abs(sol[0]))))))
print('   -> 截距 %+.2f deg = 水槽世界朝向 + 相机安装偏航 (yaw=0 时画面里槽的倾角)' % sol[1])

# 时间序列对照, 看是否同相
print()
print('== 时间序列对照 (t_rel, yaw, angle) ==')
print('   %8s %8s %8s %8s' % ('t', 'yaw', 'angle', 'diff'))
for k in range(0, len(td), max(1, len(td) // 20)):
    print('   %8.1f %8.1f %8.1f %8.1f' % (td[k] - T[0], yi[k], ang[k], ang[k] - yi[k]))

# 去掉 angle 后, 剩余是否还有结构 (槽本身不直 / 雷达漂)
res = ang - pred
print()
print('   回归残差 std = %.2f deg,  中位 %+.2f' % (res.std(), np.median(res)))
print('   残差 vs 时间 相关系数 = %+.3f (系统性漂移的话会显著)' % corr(res, td - td[0]))
print('   残差 vs x     相关系数 = %+.3f' % corr(res, np.interp(td, T, P[:, 0])))

# 高频分量: 相机角度是否只是噪声? (对比帧间跳变)
print('   angle 帧间跳变 中位 %.2f deg, yaw 帧间跳变 中位 %.3f deg' % (
    np.median(np.abs(np.diff(ang))), np.median(np.abs(np.diff(yi)))))
