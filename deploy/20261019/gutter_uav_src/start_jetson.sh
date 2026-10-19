#!/bin/bash
# Amov Jetson + PX4 启动
cd /home/amov/gutter_uav || exit 1
mkdir -p /home/amov/logs
export PYTHONUNBUFFERED=1
# 需要 dialout 组权限访问 /dev/ttyTHS0
pkill -f "python3 -u main.py" 2>/dev/null || true
sleep 1
nohup python3 -u main.py --platform jetson --no-preview \
  >/dev/null 2>&1 </dev/null &
echo $!
echo "web: http://192.168.144.60:8080"
echo "logs: /home/amov/logs/flight_latest"
echo "engine: ls -lh models/gutter_seg_320.engine"
