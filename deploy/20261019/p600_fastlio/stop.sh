#!/bin/bash
# P600 FAST-LIO2 视觉定位 一键停止 (按启动相反顺序)
# 用法: bash ~/p600_fastlio/stop.sh [--all] [--vins]
#   --all   连 roscore/rosmaster 一起停
#   --vins  连 VINS 链路(相机/vins_node/vins桥接)一起停
BASE=/home/amov/p600_fastlio
SELF=$$
STOP_MASTER=0; STOP_VINS=0
for a in "$@"; do case "$a" in
  --all) STOP_MASTER=1;; --vins) STOP_VINS=1;;
  *) echo "未知参数: $a"; exit 1;; esac; done

kill_by_cmdline() {
  local pat="$1" name="$2" pids
  pids=$(pgrep -f "$pat" 2>/dev/null | grep -v -e "^${SELF}$" -e "^${PPID}$")
  if [ -z "$pids" ]; then echo "  [ - ] $name 未运行"; return; fi
  echo "  [kill] $name  (pid: $(echo $pids | tr '\n' ' '))"
  echo "$pids" | xargs -r kill 2>/dev/null; sleep 1
  pids=$(pgrep -f "$pat" 2>/dev/null | grep -v -e "^${SELF}$" -e "^${PPID}$")
  [ -n "$pids" ] && { echo "  [KILL] $name 强杀"; echo "$pids" | xargs -r kill -9 2>/dev/null; }
}

echo "[stop] $(date) 停止 FAST-LIO 链路"
if [ -n "$(pgrep -f 'vins_to_mavros' 2>/dev/null | grep -v -e "^${SELF}$" -e "^${PPID}$")" ]; then
  echo "  ⚠️  检测到 VINS 桥接在跑 (两者都发 /mavros/vision_pose/pose, 会互相覆盖)"
  echo "     建议加 --vins 一起停"
fi
kill_by_cmdline 'fastlio_to_mavros\.py'  'fastlio_to_mavros 桥接'
kill_by_cmdline 'mavros_node'            'mavros_node'
kill_by_cmdline 'fastlio_mapping'        'fastlio_mapping (FAST-LIO2)'
kill_by_cmdline 'mapping_mid360\.launch' 'roslaunch mapping_mid360'
kill_by_cmdline 'livox_ros_driver2_node' 'livox_ros_driver2_node'
kill_by_cmdline 'msg_MID360'     'roslaunch msg_MID360'
if [ "$STOP_VINS" = "1" ]; then
  echo "[stop] --- 同时停止 VINS 链路 ---"
  kill_by_cmdline 'vins_to_mavros' 'vins_to_mavros 桥接'
  kill_by_cmdline 'vins_node'      'vins_node'
  kill_by_cmdline 'capture_image_imu' 'capture_image_imu (相机)'
fi
if [ "$STOP_MASTER" = "1" ]; then
  echo "[stop] --- 停止 ROS master ---"
  kill_by_cmdline 'rosmaster' 'rosmaster'
  kill_by_cmdline 'roscore'   'roscore'
  kill_by_cmdline 'rosout'    'rosout'
else
  echo "  [skip] roscore 保留 (要停用 --all)"
fi
sleep 1
echo "[stop] ===== 剩余相关进程 ====="
ps -eo pid,comm | grep -E 'livox|fastlio|mavros|rosmaster|roscore|vins' | grep -v grep || echo "  (已全部清理)"
echo "[stop] done"
