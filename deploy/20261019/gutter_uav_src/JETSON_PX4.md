# Jetson + PX4 移植说明

## 当前状态（Amov `192.168.144.60`）

代码目录：`/home/amov/gutter_uav`

| 项 | 状态 |
|----|------|
| 源码部署 | 已上传（`--platform jetson`） |
| 飞控栈 | `fc_stack=px4`，串口 `/dev/ttyTHS1` @921600，模式 OFFBOARD |
| pymavlink | 已装（不升级 numpy，避免 SIGILL） |
| TensorRT engine | 板端 `trtexec` 编译中/耗时长（YOLO11-seg + TRT7） |
| 飞控心跳 | 上次探测 **无 HEARTBEAT**（确认飞控上电、接线 ttyTHS0） |
| 相机 | 当时无 `/dev/video*`，需接上 USB/CSI 相机 |

## 启动

```bash
cd /home/amov/gutter_uav
# 等 models/gutter_seg_320.engine 生成后再跑检测
bash start_jetson.sh
# 或
python3 -u main.py --platform jetson --no-preview --camera-id 0
```

## 与 RK3588 基线关系

- GitHub `master` 仍以 **RK3588 + CubeOrange/ArduPilot** 为可飞基线
- Jetson 用 `--platform jetson` 覆盖配置，不改默认 `config.py` 行为

## 注意

- **不要** `pip install 'numpy>=1.16'`：会装坏 aarch64 wheel 导致 `Illegal instruction`
- 系统 numpy 保持 **1.13.3**
- TRT 编译日志：`tail -f /tmp/trt_build.log`
