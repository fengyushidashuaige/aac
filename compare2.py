#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""对比两趟飞行的数据质量, 找出第二趟残差变差的原因."""
import sys
import numpy as np
import rosbag

BAGS = ['/home/amov/logs/auto_map_01/flight.bag', '/home/amov/logs/auto_map_02/flight.bag']
CSVS = ['/home/amov/logs/guttermap_20260922_103821/detect.csv',
        '/home/amov/logs/guttermap_20260922_112200/detect.csv']


def quat_to_R(q):
    x, y, z, w = q
    n = max(np.linalg.norm(q), 1e-9)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


for tag, bagf, csvf in zip(('第一趟 10:38', '第二趟 11:22'), BAGS, CSVS):
    bag = rosbag.Bag(bagf)
    T, P, Q = [], [], []
    for tp, m, t in bag.read_messages(topics=['/Odometry']):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        T.append(t.to_sec()); P.append((p.x, p.y, p.z)); Q.append((q.x, q.y, q.z, q.w))
    bag.close()
    T = np.array(T); P = np.array(P); Rall = np.array([quat_to_R(q) for q in Q])
    dur = T[-1] - T[0]
    step = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
    yaw = np.degrees(np.arctan2(Rall[:, 1, 0], Rall[:, 0, 0]))
    dyaw = np.diff(np.unwrap(np.radians(yaw)))
    print('=' * 78)
    print('%s   时长 %.0fs  雷达 %.1fHz' % (tag, dur, len(T) / dur))
    print('  轨迹: 直线位移 %.2fm (x %.2f~%.2f, y %.2f~%.2f)  路径长 %.2fm  折返比 %.2f' % (
        np.linalg.norm(P[-1, :2] - P[0, :2]), P[:, 0].min(), P[:, 0].max(),
        P[:, 1].min(), P[:, 1].max(), step.sum(),
        step.sum() / max(np.linalg.norm(P[-1, :2] - P[0, :2]), 1e-6)))
    print('  高度 z %.2f~%.2f (中位 %.2f)   yaw 范围 %.1f~%.1f (std %.1f)' % (
        P[:, 2].min(), P[:, 2].max(), np.median(P[:, 2]), yaw.min(), yaw.max(), yaw.std()))
    print('  偏航角速度 |dpsi|/dt: 中位 %.1f deg/s  P90 %.1f' % (
        np.degrees(np.median(np.abs(dyaw) / np.maximum(np.diff(T), 1e-6))),
        np.degrees(np.percentile(np.abs(dyaw) / np.maximum(np.diff(T), 1e-6), 90))))

    raw = open(csvf, 'rb').read().replace(b'\x00', b'')
    rows = []
    for ln in raw.decode('utf-8', 'ignore').splitlines()[1:]:
        p = ln.split(',')
        if len(p) < 7:
            continue
        try:
            rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]), float(p[6])))
        except Exception:
            pass
    det = [r for r in rows if r[4] == 1 and T[0] <= r[0] <= T[-1]]
    off = np.array([r[1] for r in det]); wp = np.array([r[5] for r in det])
    cf = np.array([r[3] for r in det])
    print('  检测: 全场 %d 帧, 飞行时段检出 %d (%.0f%% 的帧率 %.1ffps)' % (
        len(rows), len(det), 100.0 * len(det) / max(len(rows), 1), len(rows) / dur))
    print('  off_mm: 中位 %.0f P10 %.0f P90 %.0f |off|>315 的占 %.0f%%  |off|>500 占 %.0f%%' % (
        np.median(off), np.percentile(off, 10), np.percentile(off, 90),
        100.0 * (np.abs(off) > 315).mean(), 100.0 * (np.abs(off) > 500).mean()))
    print('  wpx: 中位 %.1f P10 %.1f P90 %.1f min %.1f max %.1f' % (
        np.median(wp), np.percentile(wp, 10), np.percentile(wp, 90), wp.min(), wp.max()))
    print('  conf>=0.9 占 %.0f%%' % (100.0 * (cf >= 0.9).mean()))
    # 大偏差点是不是伴随小 wpx (截断特征)
    big = np.abs(off) > 400
    if big.sum() > 5:
        print('  |off|>400 的帧: %d, 其 wpx 中位 %.1f (整体 %.1f)' % (
            big.sum(), np.median(wp[big]), np.median(wp)))
    # 用户的摆动机动: 横向速度
    vlat = np.diff(P[:, 1]) / np.maximum(np.diff(T), 1e-6)
    print('  横向速度 |vy|: 中位 %.3f m/s  P90 %.3f  max %.3f' % (
        np.median(np.abs(vlat)), np.percentile(np.abs(vlat), 90), np.abs(vlat).max()))
