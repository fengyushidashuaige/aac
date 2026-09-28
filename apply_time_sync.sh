#!/bin/bash
# 时间戳对齐补丁 —— 在板子上跑一次即可
# 用法: bash apply_time_sync.sh
B=/home/amov/guttermap
echo "=========================================="
echo " 时间戳对齐补丁"
echo "=========================================="

python3 - <<'PY'
import io, shutil, os
B = '/home/amov/guttermap/'

# ---------- ① gutter_udp.py: 时间戳改打"帧到达时刻" ----------
p = B + 'gutter_udp.py'
s = io.open(p, encoding='utf-8').read()
shutil.copy2(p, p + '.bak_timesync')

old = """    frame_raw = frame.copy()
    frame = apply_corr(frame)
    r = det.detect(frame)"""
new = """    t_cap = time.time()          # ★ 帧到达时刻 (对齐用: 不是处理完的时刻)
    frame_raw = frame.copy()
    frame = apply_corr(frame)
    r = det.detect(frame)"""
assert old in s, 'udp anchor1'
s = s.replace(old, new, 1)

old2 = """    msg = {'t': time.time(),"""
new2 = """    msg = {'t': t_cap, 't_proc': time.time(),   # t=帧时刻, t_proc=处理完时刻"""
assert old2 in s, 'udp anchor2'
s = s.replace(old2, new2, 1)
io.open(p, 'w', encoding='utf-8').write(s)
print('  ✓ gutter_udp.py: 时间戳改为帧到达时刻')

# ---------- ② gutter_map.py: 位姿缓冲 + 按帧时刻插值 ----------
p2 = B + 'gutter_map.py'
s2 = io.open(p2, encoding='utf-8').read()
shutil.copy2(p2, p2 + '.bak_timesync')

s2 = s2.replace("import rospy, socket, json, math, sys, os, time, numpy as np",
                "import rospy, socket, json, math, sys, os, time, collections, numpy as np", 1)

old3 = "pose = {'R': np.eye(3), 't': np.zeros(3), 'ok': False}"
new3 = """pose = {'R': np.eye(3), 't': np.zeros(3), 'ok': False}
PBUF = collections.deque(maxlen=900)      # 位姿历史 (~90s @10Hz), 用于按帧时刻回查

def _stamp_of(m):
    try:
        v = m.header.stamp.to_sec()
        if v > 1e6: return v
    except Exception:
        pass
    return time.time()

def pose_at(ts):
    \"\"\"取 ts 时刻的位姿 (线性插值位置; 姿态取最近). ts 早于缓冲则取最旧.\"\"\"
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
    return R0, p0 + a * (p1 - p0)"""
assert old3 in s2, 'map anchor1'
s2 = s2.replace(old3, new3, 1)

old4 = """    pose['R'] = R; pose['t'] = np.array([p.x,p.y,p.z]); pose['ok'] = True"""
new4 = """    pose['R'] = R; pose['t'] = np.array([p.x,p.y,p.z]); pose['ok'] = True
    PBUF.append((_stamp_of(m), R, np.array([p.x,p.y,p.z])))"""
assert old4 in s2, 'map anchor2'
s2 = s2.replace(old4, new4, 1)

# 投影时用帧时刻回查位姿
old5 = """        off = d['offset_mm'] / 1000.0
        # 机体系 (LiDAR body: x前 y左 z上), 相机下视
        p_b = np.array([off*math.sin(CAM_YAW), -off*math.cos(CAM_YAW), -CAM_DOWN])
        p_w = pose['R'].dot(p_b) + pose['t']"""
new5 = """        off = d['offset_mm'] / 1000.0
        # ★ 按【帧时刻】回查位姿, 而不是用"最新"位姿
        _ts = d.get('t', None)
        R_use, t_use = (pose['R'], pose['t']) if _ts is None else pose_at(_ts)
        if R_use is None:
            R_use, t_use = pose['R'], pose['t']
        p_b = np.array([off*math.sin(CAM_YAW), -off*math.cos(CAM_YAW), -CAM_DOWN])
        p_w = R_use.dot(p_b) + t_use"""
if old5 in s2:
    s2 = s2.replace(old5, new5, 1)
    print('  ✓ gutter_map.py: 投影改用帧时刻回查位姿')
else:
    print('  !! gutter_map.py 投影锚点未找到, 打印实际内容:')
    i = s2.find("off = d['offset_mm']")
    print(s2[i-200:i+600])
io.open(p2, 'w', encoding='utf-8').write(s2)
PY

python3 -m py_compile $B/gutter_udp.py && echo "  ✓ gutter_udp.py 语法 OK"
python2 -m py_compile $B/gutter_map.py && echo "  ✓ gutter_map.py 语法 OK"
echo
echo "=========================================="
echo " 完成。重启生效:"
echo "   bash ~/guttermap/start_detect.sh"
echo " 回滚:"
echo "   cp $B/gutter_udp.py.bak_timesync $B/gutter_udp.py"
echo "   cp $B/gutter_map.py.bak_timesync  $B/gutter_map.py"
echo "=========================================="
