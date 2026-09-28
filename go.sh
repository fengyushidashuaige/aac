#!/bin/bash
# 一键起飞准备: 雷达定位 + 相机检测/投影 + 录制
# 用法: bash ~/guttermap/go.sh
B=/home/amov/guttermap
echo "=========================================="
echo " 第 1/4 步: 起雷达定位 (约 40 秒)"
echo "=========================================="
bash /home/amov/p600_fastlio/start.sh
source /opt/ros/melodic/setup.bash
source /home/amov/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.144.60:11311

echo
echo "--- 检查定位 ---"
if timeout 10 rostopic hz /Odometry 2>/dev/null | grep -q "average rate"; then
  echo "  ✓ 定位正常"
else
  echo "  ✗ /Odometry 无输出!"
  echo "    → 大概率是雷达卡住, 给雷达断电重启后再跑一次"
  exit 1
fi

echo
echo "=========================================="
echo " 第 2/4 步: 起相机检测 + 世界系投影"
echo "=========================================="
bash $B/start_detect.sh

echo
echo "=========================================="
echo " 第 3/4 步: 自动测光 (按当前光照调曝光/增益)"
echo "=========================================="
echo "  等检测进程就绪..."
for i in $(seq 1 40); do
  if curl -s --max-time 2 -o /dev/null http://127.0.0.1:8080/frame_raw.jpg 2>/dev/null; then
    echo "  ✓ 检测已就绪"; break
  fi
  sleep 2
done
# ★ 2026-09-24: 这里原来调 cam_tune.sh —— 【已删除, 别再调】
#   cam_tune.sh 是【换镜头之前】的老方案, 它写死 exposure_auto=1(手动曝光)
#   / WB=2800 / gamma=64 / brightness=-64, 跑完画面压糊压灰、光照一变就废。
#   现役镜头带 IR-CUT, 正确做法是【什么都别做】——
#   start_detect.sh 启动时 cam_init() 会自动设好:
#       exposure_auto=3(自动) / gamma=100 / brightness=0 / gain=60 / 白平衡自动
echo "  (跳过测光: cam_init() 已自动设好曝光/白平衡, 不要跑 cam_tune.sh)"

echo
echo "=========================================="
echo " 第 4/4 步: 起录制"
echo "=========================================="
bash $B/flight_record.sh

echo
echo "=========================================="
echo " 就绪 ✓ 可以飞了"
echo " 预览: http://192.168.144.60:8080"
echo "   (左边=原始  中间=喂模型  右边=地图)"
echo " 飞的时候看: conf>=0.7 且地图红点收成一条线\n  若画面全黑/全白/压糊 → 【不要】跑 cam_tune.sh!\n  改跑: bash ~/guttermap/start_detect.sh  (cam_init 会重置曝光/白平衡)"
echo "=========================================="
