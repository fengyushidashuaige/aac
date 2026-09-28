# guttermap — 水槽 v2 工具链

建立 2026-09-14。水槽项目 **v2（雷达定位 + 相机建图）** 的建图/验证阶段工具。
**规划文档：`~/gutter_uav/NEXT_STEPS.md`**

## 干什么用

把「单目相机测出的水槽横向偏差」和「FAST-LIO 的雷达位姿」融合，
把水槽中心线**钉进世界系**变成一条折线。

有了折线，飞行就从「逐帧伺服瞬时测量」（v1 不丝滑的根源）
变成「**沿折线纯追踪 + 相机补低频残差**」。

## 前置依赖

- ROS Melodic + `~/fastlio_ws`（livox_ros_driver2 + FAST_LIO）已编译
- **雷达 Livox MID-360S**：`eth1 = 192.168.1.50/24`，雷达 `192.168.1.144`
  - ⚠️ 必须用 `msg_MID360s.launch`（用错会静默不建数据通道）
  - ⚠️ 卡住（命令通道通但数据端口不绑）→ **断电重启雷达**
- 相机 `/dev/video0`（DECXIN DCGX200 定焦）

## 完整流程（冷启动）

```bash
# 1. 起定位链路  ⚠️ 会占 /dev/ttyTHS1 并往飞控发 vision_pose
bash ~/p600_fastlio/start.sh

# 2. 起检测 + 投影
bash ~/guttermap/start_detect.sh
#    预览 http://192.168.144.60:8080/frame.jpg

# 3. 建图/验证：手推着飞机沿槽走，看 /gutter_map/points 拉不拉成线

# 4. 飞行记录
bash ~/guttermap/flight_record.sh          # 起录
#    ...你飞一圈...
bash ~/guttermap/flight_stop.sh            # 停录
bash ~/guttermap/analyze.sh                # 出轨迹分析

# 5. 收工
bash ~/guttermap/stop_detect.sh
```

## 文件

| 文件 | 作用 |
|------|------|
| `gutter_udp.py` | py3：相机+TRT检测 → UDP输出 + HTTP预览 + 录 `overlay.avi`/`detect.csv` |
| `gutter_map.py` | py2：收UDP + 订 `/Odometry` → 投影世界系 → 发 `/gutter_map/points`、`/gutter_map/markers` |
| `start_detect.sh` / `stop_detect.sh` | 上面两个的启停 |
| `flight_record.sh` / `flight_stop.sh` | rosbag + 飞控事件日志 |
| `flight_events.py` | py2：记录模式/解锁变化 |
| `analyze_flight.py` / `analyze.sh` | 轨迹分析：俯视图/闭合误差/漂移/高度剖面 |
| `run/` | 运行时 pid / log |

## 可调参数（`gutter_map.py` 顶部，**待标定**）

| 参数 | 当前 | 说明 |
|------|------|------|
| `CAM_DOWN` | 0.55 | 相机镜头到水槽的垂直距离(m)，**只影响绘制高度**，不影响水平投影 |
| `CAM_YAW` | 0.0 | 相机相对机体的安装偏航(rad)。**待标定** —— 注意真水槽在画面里稳定出现在 ~54°，见 NEXT_STEPS §4.4 |
| `MIN_CONF` | 0.6 | 与飞行代码一致的门限（实测：真水槽 ~0.87，吹风筒 0.35） |

## 已验证结论

- FAST-LIO `/Odometry` **10Hz**，台架静止 45s **零累积漂移**，位置抖动 σ=2~5mm，姿态 σ<0.15°
- 检测→UDP→位姿投影→点云 **全链路跑通**（z 投影值 −0.646 与理论完全一致）
- **0.6 门限实测生效**：吹风筒 conf 0.35 被拒（旧门限 0.3 会放行）
- 干跑 12.6s 静止：**闭合误差 6mm**，最大漂移 11mm

## 已知坑

1. **`rosbag record` 是两层进程**（python2 包装器 → `lib/rosbag/record` 子进程）。
   `$!` 抓到的是包装器，SIGINT 传不到子进程 → 必须给**整个进程组**发信号。
   `flight_stop.sh` 已处理（`kill -INT -PGID`）。
2. **`HTTPServer` 单线程**会被一个卡住的连接堵死 → 已用 `ThreadingMixIn`。
3. 检测进程固定占 **~4.5GB**（TRT/CUDA 一次性分配，**不是泄漏**，平台化不再涨）。
4. **智谱 GLM-4V-Flash 不能做空间位置判断**（会编造），只能问「有没有/什么颜色/读文字」。
5. 世界系锚定：FAST-LIO 世界系 = **开机瞬间机体系**。跨次使用需每次摆在同位置同朝向
   （yaw 无法自观测，是决定性的）。生产期换电池/跨天才需要 GPS。

## 下一步

见 `~/gutter_uav/NEXT_STEPS.md` §11


---

## 🔴 相机白平衡必须修正（2026-09-16 发现）

**相机自带的自动白平衡是坏的**，会造成严重品红偏色（R−B ≈ +95~128，R 通道接近饱和），
**导致分割模型完全失效（conf=0）**。2026-09-16 一次实飞 103 秒、4242 帧、零检出，就是它造成的。

实测：修 WB 前 `conf 0.000, detected 0/5` → 修 WB 后 `conf 0.500, detected 5/5`。

**修复（必须用 v4l2-ctl，cv2 设不了这个控制项）：**

```bash
v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 -c white_balance_temperature=2800
```

- 已内置到 `start_detect.sh`，每次启动自动执行
- ⚠️ **相机断电后会回到坏掉的 AWB**（UVC 无 NVRAM），所以必须每次开机设置
- ⚠️ **2026-09-16 之前采集的所有数据都在坏 WB 下拍摄的**，不可直接用于训练/调参

## 预览界面

浏览器打开 **`http://192.168.144.60:8080`** —— 实时 MJPEG 视频 + conf/offset/angle 大字显示。

| 路径 | 内容 |
|------|------|
| `/` | HTML 预览页（`preview.html`） |
| `/video_feed` | MJPEG 实时流 |
| `/frame.jpg` | 单帧 JPEG |
| `/api/telemetry` | JSON：off / ang / conf / det / fps |

判据：**conf ≥ 0.7 = 锁定水槽** ｜ **≈0.5 = 喷嘴** ｜ **0 = 无目标**
