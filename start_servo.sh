#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  水槽视觉伺服 一键启动 (检测 + 伺服)
#
#  前置: ROS master + mavros 必须已就绪
#        先跑 ->  bash ~/p600_fastlio/start.sh    (雷达 + FAST-LIO + mavros)
#        纯伺服其实不需要 FAST-LIO, 但 mavros 是必须的(指令要经它下发)
#
#  停止: bash ~/guttermap/stop_servo.sh
# ═══════════════════════════════════════════════════════════
B=/home/amov/guttermap
source /opt/ros/melodic/setup.bash 2>/dev/null
export ROS_MASTER_URI=http://192.168.144.60:11311

echo "=== 前置检查 ==="
if ! (timeout 3 rostopic list >/dev/null 2>&1); then
  echo "  ROS master   X  不通"
  echo "   先跑: bash ~/p600_fastlio/start.sh"
  exit 1
fi
echo "  ROS master   ok"

if timeout 3 rostopic list 2>/dev/null | grep -q '^/mavros/state'; then
  echo "  mavros       ok"
else
  echo "  mavros       X  没在跑 —— 指令发不到飞控"
  echo "   先跑: bash ~/p600_fastlio/start.sh"
  exit 1
fi

echo
echo "=== 1/2  检测 gutter_udp.py ==="
if [ -f $B/run/gutter_udp.pid ] && kill -0 "$(cat $B/run/gutter_udp.pid)" 2>/dev/null; then
  echo "  已在跑 pid=$(cat $B/run/gutter_udp.pid)"
else
  cd /home/amov/gutter_uav || { echo "  X 进不去 /home/amov/gutter_uav"; exit 1; }
  setsid nohup python3 -u $B/gutter_udp.py > $B/run/udp.log 2>&1 < /dev/null &
  echo $! > $B/run/gutter_udp.pid
  echo "  已启动 pid=$(cat $B/run/gutter_udp.pid), 等 TRT 引擎加载..."
  sleep 12
  if kill -0 "$(cat $B/run/gutter_udp.pid)" 2>/dev/null; then
    grep -E '实际格式|相机 ok|!!' $B/run/udp.log | sed 's/^/    /'
  else
    echo "  X 检测进程退了, 看日志: $B/run/udp.log"
    tail -5 $B/run/udp.log | sed 's/^/    /'
  fi
fi

echo
echo "=== 2/2  伺服 servo_track.py ==="
if [ -f $B/run/servo.pid ] && kill -0 "$(cat $B/run/servo.pid)" 2>/dev/null; then
  echo "  已在跑 pid=$(cat $B/run/servo.pid)"
else
  setsid nohup python2 -u $B/servo_track.py > $B/run/servo.log 2>&1 < /dev/null &
  echo $! > $B/run/servo.pid
  sleep 4
  if kill -0 "$(cat $B/run/servo.pid)" 2>/dev/null; then
    echo "  已启动 pid=$(cat $B/run/servo.pid)"
  else
    echo "  X 伺服进程退了, 看日志: $B/run/servo.log"
    tail -5 $B/run/servo.log | sed 's/^/    /'
  fi
fi

cat << EOF

═══════════════ 就绪 ═══════════════
  预览      http://192.168.144.60:8080/
  使能      rosservice call /servo_track/set_enable true
  看伺服    tail -f $B/run/servo.log
  看检测    tail -f $B/run/udp.log
  停止      bash $B/stop_servo.sh

  注意  伺服默认【不使能】= 只发零速流。
        要它真的动, 必须执行上面的使能命令。
        接管/收工: rosservice call /servo_track/set_enable false
EOF
