#!/bin/bash
# P600 FAST-LIO2 视觉定位 一键启动
# 用法: bash ~/p600_fastlio/start.sh
# 顺序: roscore -> 雷达驱动 -> FAST-LIO2 -> MAVROS -> 位置桥接   (幂等, 可反复跑)
BASE=/home/amov/p600_fastlio
LOG=$BASE/logs
FCU_URL="/dev/ttyTHS1:921600"

export ROS_MASTER_URI=http://192.168.144.60:11311
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
mkdir -p $LOG
SELF=$$
echo "[fastlio] $(date) start"

# 1. roscore
if ! (timeout 3 rostopic list >/dev/null 2>&1); then
  setsid nohup roscore > $LOG/roscore.log 2>&1 < /dev/null &
  sleep 5
fi
echo "[fastlio] master ok"

# 2. Livox MID360S 驱动  ← 注意是 S! 用错会静默不建数据通道
if ! pgrep -x livox_ros_drive >/dev/null; then
  setsid nohup roslaunch livox_ros_driver2 msg_MID360s.launch rviz_enable:=false > $LOG/livox.log 2>&1 < /dev/null &
  sleep 12
  if timeout 8 rostopic hz /livox/lidar 2>/dev/null | grep -q 'average rate'; then
    echo "[fastlio] 雷达 OK"
  else
    echo "[fastlio] !! 雷达无数据 — 试给雷达断电重启; 详见 $LOG/livox.log"
  fi
fi

# 3. FAST-LIO2
if ! pgrep -x fastlio_mapping >/dev/null; then
  setsid nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio.log 2>&1 < /dev/null &
  sleep 10
fi
echo "[fastlio] fast_lio started"

# 4. MAVROS (飞控串口)
if ! pgrep -f 'mavros_node' >/dev/null; then
  setsid nohup roslaunch mavros px4.launch fcu_url:="$FCU_URL" > $LOG/mavros.log 2>&1 < /dev/null &
  sleep 10
fi
echo "[fastlio] mavros started"

# 5. 位置桥接 (默认透传)
if ! pgrep -f 'fastlio_to_mavros.py' >/dev/null; then
  setsid nohup python $BASE/fastlio_to_mavros.py > $LOG/bridge.log 2>&1 < /dev/null &
fi
echo "[fastlio] bridge started"

# 冲突检查
[ -n "$(pgrep -f 'vins_to_mavros' 2>/dev/null | grep -v -e "^${SELF}$" -e "^${PPID}$")" ] && \
  echo "[fastlio] ⚠️  VINS 桥接也在跑, 两者都发 /mavros/vision_pose/pose, 会打架!"

echo "[fastlio] ===== 自检 (等 3s) ====="
sleep 3
echo "--- 进程 ---"
ps -ef | grep -E '[l]ivox_ros_drive|[f]astlio_mapping|[m]avros_node|[f]astlio_to_mavros' | awk '{print "   ", $2, $8, $9}'
echo "--- 雷达点云 ---";       timeout 6 rostopic hz /livox/lidar 2>/dev/null | tail -1
echo "--- FAST-LIO 里程计 ---"; timeout 6 rostopic hz /Odometry 2>/dev/null | tail -1
echo "--- 飞控连接 ---";        timeout 6 rostopic echo -n1 /mavros/state 2>/dev/null | grep -E 'connected|^mode'
echo "--- 发往飞控的定位 ---";  timeout 6 rostopic hz /mavros/vision_pose/pose 2>/dev/null | tail -1
echo "[fastlio] done"
