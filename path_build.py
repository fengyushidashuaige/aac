#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
从飞行数据生成路径 path.csv

两种模式:
  --plane    只用【飞机轨迹】           (阶段1, 简单)
  默认       飞机轨迹 + 【相机偏移校正】  (阶段2, 自动: 飞歪也能算对)

相机校正原理:
    槽的真实位置 = 飞机位置 + R(航向)·(0, -offset)      (机体系横向)
    对轨迹上每个点, 用附近时间窗内的检出 offset 做平均 -> 得到连续的路径

用法:
  python2 path_build.py <bag> <detect.csv> <out.csv> [--plane]
"""
import sys, math, os
import numpy as np, rosbag
from nav_msgs.msg import Odometry

WIN_S     = 0.6      # 校正时的时间窗(秒)
MIN_CONF  = 0.4          # 2026-09-21: 0.6 -> 0.4 (适配微调后 conf 降低的 v2 模型)

def load_poses(bag_path):
    try:
        bag = rosbag.Bag(bag_path)
    except Exception:
        bag = rosbag.Bag(bag_path, allow_unindexed=True)
    T, P, Y = [], [], []
    for tp, m, t in bag.read_messages(topics=['/Odometry']):
        q = m.pose.pose.orientation; p = m.pose.pose.position
        x, y, z, w = q.x, q.y, q.z, q.w
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Y.append(yaw)
    bag.close()
    return np.array(T), np.array(P), np.array(Y)

def load_detect(csv_path):
    if not os.path.exists(csv_path):
        return []
    raw = open(csv_path, 'rb').read().replace(b'\x00', b'')
    out = []
    for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
        p = ln.split(',')
        if len(p) < 5:
            continue
        try:
            t, off, ang, cf, det = float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4])
        except ValueError:
            continue
        if cf >= MIN_CONF:
            out.append((t, off, ang, cf))
    return out

def main():
    if len(sys.argv) < 4:
        print(__doc__); sys.exit(1)
    bag_path, det_csv, out_csv = sys.argv[1], sys.argv[2], sys.argv[3]
    use_cam = '--plane' not in sys.argv

    T, P, Y = load_poses(bag_path)
    print('位姿 %d 条  %.1f~%.1f' % (len(T), T[0], T[-1]))
    det = load_detect(det_csv) if use_cam else []
    print('检出(conf>=%.1f) %d 条' % (MIN_CONF, len(det)))

    if not use_cam:
        print('>>> 模式: 只用飞机轨迹')
        path = [(P[i][0], P[i][1]) for i in range(len(P))]
    else:
        print('>>> 模式: 飞机轨迹 + 相机偏移校正')
        if len(det) == 0:
            print('!! 没有可用检出, 退化为纯飞机轨迹'); 
            path = [(P[i][0], P[i][1]) for i in range(len(P))]
        else:
            # ① 轨迹各点的累计弧长
            arc = [0.0]
            for i in range(1, len(T)):
                arc.append(arc[-1] + math.hypot(P[i][0]-P[i-1][0], P[i][1]-P[i-1][1]))
            arc = np.array(arc)

            # ② 每条检出 -> 对齐到最近的轨迹点 -> (弧长, offset)
            idx = np.clip(np.searchsorted(T, [d[0] for d in det]), 0, len(T)-1)
            s_meas = np.array([arc[j] for j in idx])
            o_meas = np.array([d[1] for d in det])
            order = np.argsort(s_meas); s_meas = s_meas[order]; o_meas = o_meas[order]
            # 同一弧长位置取平均
            su, ou = [], []
            for a, b in zip(s_meas, o_meas):
                if su and abs(a - su[-1]) < 0.05:
                    ou[-1] = 0.5*(ou[-1] + b)
                else:
                    su.append(a); ou.append(b)
            print('   有效测量点 %d 个 (弧长 %.1f ~ %.1f m / 总 %.1f m)' % (
                len(su), su[0], su[-1], arc[-1]))
            print('   覆盖率 %.0f%%  <- 有测量的弧长区间占比' % (
                100.0*(su[-1]-su[0])/max(arc[-1],1e-6)))

            # ③ 沿弧长插值, 得到每一点的偏移 (区间外取最近值)
            off_all = np.interp(arc, su, ou)
            print('   校正量: 中位 %+.0f mm  范围 [%+.0f, %+.0f]' % (
                np.median(off_all), off_all.min(), off_all.max()))

            # ④ 施加到轨迹上
            path = []
            for i in range(len(T)):
                off = float(off_all[i]); yw = Y[i]
                dx = -(off/1000.0) * (-math.sin(yw))
                dy = -(off/1000.0) * ( math.cos(yw))
                path.append((P[i][0] - dx, P[i][1] - dy))

    with open(out_csv, 'w') as f:
        f.write('x,y,z\n')
        for i, (x, y) in enumerate(path):
            f.write('%.4f,%.4f,%.4f\n' % (x, y, P[i][2] if i < len(P) else 0.0))
    print('已写 %s  (%d 点)' % (out_csv, len(path)))

    # 自检: 路径长度与平滑
    L = sum(math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]) for i in range(1, len(path)))
    print('路径长度 %.2f m' % L)

if __name__ == '__main__':
    main()
