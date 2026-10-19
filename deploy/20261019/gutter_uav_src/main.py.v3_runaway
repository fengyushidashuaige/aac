"""
PVC水槽中心线检测 — 主程序
在AIO 3588L上运行，实时检测水槽中心线并控制无人机

用法:
    python main.py                          # 使用默认摄像头
    python main.py --video test.mp4         # 使用视频文件(调试)
    python main.py --image test.jpg         # 单张图片(调试)
    python main.py --no-control             # 只检测不控制
    python main.py --no-preview             # 不显示预览窗口

键盘控制 (预览窗口):
    q/ESC  - 退出
    s      - 保存当前帧
    r      - 重置检测器
    +/-    - 调整Canny高阈值
    [/]    - 调整Hough投票阈值
    Space  - 切换PID激活
"""

import cv2
import numpy as np
import time
import argparse
import sys
import os
import subprocess
import queue
import threading
import faulthandler
import signal

faulthandler.register(signal.SIGUSR1, all_threads=True)

from config import Config, default_config
from detector import TroughDetector, CenterLineResult
from controller import FlightController, ControlCommand
from preview import SharedState, start_server, state as preview_state


class _Tee:
    """stdout 同时写终端和持久日志文件。"""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            try:
                s.write(data)
                s.flush()
            except Exception:
                pass

    def flush(self):
        for s in self._streams:
            try:
                s.flush()
            except Exception:
                pass


def _logs_root():
    """持久目录：Jetson=/home/amov/logs，RK3588=/home/firefly/logs。"""
    candidates = [
        "/home/amov/logs",
        "/home/firefly/logs",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"),
        os.path.join(os.getcwd(), "logs"),
    ]
    for d in candidates:
        try:
            os.makedirs(d, exist_ok=True)
            return d
        except Exception:
            continue
    return None


def _setup_persistent_log(session_dir=None):
    """日志落到会话目录或 logs/，不用 /tmp。"""
    log_dir = session_dir or _logs_root()
    if not log_dir:
        return None
    stamp = os.path.basename(session_dir) if session_dir else time.strftime("%Y%m%d_%H%M%S")
    if session_dir:
        path = os.path.join(log_dir, "flight.log")
    else:
        path = os.path.join(log_dir, f"gutter_flight_{stamp}.log")
    try:
        fp = open(path, "a", encoding="utf-8", buffering=1)
        sys.stdout = _Tee(sys.__stdout__, fp)
        sys.stderr = _Tee(sys.__stderr__, fp)
        root = _logs_root() or log_dir
        latest = os.path.join(root, "gutter_flight_latest.log")
        try:
            if os.path.islink(latest) or os.path.exists(latest):
                os.remove(latest)
            # 指向实际日志文件
            os.symlink(path, latest)
        except Exception:
            try:
                with open(latest + ".path", "w", encoding="utf-8") as pf:
                    pf.write(path + "\n")
            except Exception:
                pass
        return path
    except Exception as e:
        print(f"[Log] 无法创建持久日志: {e}")
        return None


def _setup_flight_session():
    """每次飞行一个目录：log + 录像，重启不丢。"""
    root = _logs_root()
    if not root:
        return None
    stamp = time.strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(root, f"flight_{stamp}")
    try:
        os.makedirs(session_dir, exist_ok=True)
    except Exception as e:
        print(f"[Session] 无法创建目录: {e}")
        return None
    latest_dir = os.path.join(root, "flight_latest")
    try:
        if os.path.islink(latest_dir):
            os.remove(latest_dir)
        if not os.path.exists(latest_dir):
            os.symlink(session_dir, latest_dir)
        else:
            with open(os.path.join(root, "flight_latest.path"), "w", encoding="utf-8") as pf:
                pf.write(session_dir + "\n")
    except Exception:
        try:
            with open(os.path.join(root, "flight_latest.path"), "w", encoding="utf-8") as pf:
                pf.write(session_dir + "\n")
        except Exception:
            pass
    return {
        "stamp": stamp,
        "dir": session_dir,
        "log": os.path.join(session_dir, "flight.log"),
        "video": os.path.join(session_dir, "overlay.avi"),
        "raw_video": os.path.join(session_dir, "raw.avi"),
    }


class _FlightVideoRecorder:
    """后台写叠加录像，避免堵检测/控制环。"""

    def __init__(self, path, width, height, fps=12.0, every_n=2):
        self.path = path
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.every_n = max(1, int(every_n))
        self._q = queue.Queue(maxsize=8)
        self._stop = threading.Event()
        self._writer = None
        self._frame_i = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _open(self):
        for fourcc_name in ("MJPG", "XVID"):
            fourcc = cv2.VideoWriter_fourcc(*fourcc_name)
            w = cv2.VideoWriter(self.path, fourcc, self.fps, (self.width, self.height))
            if w is not None and w.isOpened():
                self._writer = w
                print(f"[Record] 录像 {self.path} ({fourcc_name} {self.width}x{self.height} @{self.fps:.0f})")
                return True
            if w is not None:
                w.release()
        print(f"[Record] 无法创建录像: {self.path}")
        return False

    def _run(self):
        if not self._open():
            return
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            try:
                self._writer.write(item)
            except Exception:
                pass
        try:
            if self._writer is not None:
                self._writer.release()
        except Exception:
            pass

    def submit(self, frame_bgr, overlay_bgr=None):
        self._frame_i += 1
        if (self._frame_i % self.every_n) != 0:
            return
        src = overlay_bgr if overlay_bgr is not None else frame_bgr
        if src is None:
            return
        try:
            if src.shape[1] != self.width or src.shape[0] != self.height:
                src = cv2.resize(src, (self.width, self.height), interpolation=cv2.INTER_AREA)
            self._q.put_nowait(src.copy())
        except queue.Full:
            pass
        except Exception:
            pass

    def close(self):
        self._stop.set()
        try:
            self._q.put_nowait(None)
        except Exception:
            pass
        self._thread.join(timeout=3.0)


# ═══════════════════════════════════════════════════════════════
# 可视化
# ═══════════════════════════════════════════════════════════════

def draw_result(frame: np.ndarray, result: CenterLineResult,
                config: Config) -> np.ndarray:
    """
    在原始图像上绘制检测结果

    绘制内容:
    - 透视变换ROI区域
    - 检测到的所有线段 (白色)
    - 左边界 (绿色) / 右边界 (红色) / 中心线 (黄色)
    - 偏移量和角度文字
    - 水平指示条
    """
    dc = config.debug
    h, w = frame.shape[:2]

    # --- 绘制透视变换ROI ---
    tc = config.trough
    roi_pts = np.int32([
        [tc.perspective_src[0][0] * w, tc.perspective_src[0][1] * h],
        [tc.perspective_src[1][0] * w, tc.perspective_src[1][1] * h],
        [tc.perspective_src[2][0] * w, tc.perspective_src[2][1] * h],
        [tc.perspective_src[3][0] * w, tc.perspective_src[3][1] * h],
    ])
    cv2.polylines(frame, [roi_pts], True, (100, 100, 100), 2)

    # --- 绘制检测到的线段 (映射回原图) ---
    if result.raw_lines:
        for line in result.raw_lines:
            # 线段的鸟瞰图坐标需要映射回原图
            # 简化: 直接在鸟瞰图上画，这部分在另一个窗口显示
            pass

    # --- 绘制状态栏 ---
    _draw_status_bar(frame, result, config)

    return frame


def draw_warped_view(warped: np.ndarray, result: CenterLineResult,
                     config: Config, edges: np.ndarray = None) -> np.ndarray:
    """
    绘制鸟瞰图调试视图

    这是最重要的调试画面，显示:
    - 鸟瞰图
    - 边缘检测结果
    - 检测到的线段
    - 左右边界拟合结果
    - 中心线
    - 中心偏移指示
    """
    dc = config.debug
    h, w = warped.shape[:2]

    # --- 创建彩色输出 ---
    if edges is not None:
        # 将边缘叠加到鸟瞰图上
        vis = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        vis[edges > 0] = [50, 50, 100]  # 蓝色边缘
    else:
        vis = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)

    frame_center_x = w / 2.0

    # --- 绘制所有原始线段 ---
    if result.raw_lines:
        for line in result.raw_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(vis, (x1, y1), (x2, y2),
                     dc.color_all_lines, 1, cv2.LINE_AA)

    # --- 绘制左右边界点 ---
    for pt in result.left_points:
        cv2.circle(vis, (int(pt[0]), int(pt[1])),
                   2, dc.color_left_edge, -1)
    for pt in result.right_points:
        cv2.circle(vis, (int(pt[0]), int(pt[1])),
                   2, dc.color_right_edge, -1)

    # --- 绘制检测到的左右边界线 + 中心线 ---
    if result.detected:
        center_x = frame_center_x + result.offset_px

        # 左右边界X坐标
        left_x = center_x - result.trough_width_px / 2.0
        right_x = center_x + result.trough_width_px / 2.0

        # 左边界
        cv2.line(vis,
                 (int(left_x), 0), (int(left_x), h),
                 dc.color_left_edge, 2, cv2.LINE_AA)
        # 右边界
        cv2.line(vis,
                 (int(right_x), 0), (int(right_x), h),
                 dc.color_right_edge, 2, cv2.LINE_AA)
        # 中心线 (虚线效果)
        for y in range(0, h, 10):
            cv2.line(vis,
                     (int(center_x), y),
                     (int(center_x), min(y + 5, h)),
                     dc.color_center_line, 2, cv2.LINE_AA)

        # 图像中心线 (白色虚线，用于对比)
        for y in range(0, h, 10):
            cv2.line(vis,
                     (int(frame_center_x), y),
                     (int(frame_center_x), min(y + 5, h)),
                     (255, 255, 255), 1, cv2.LINE_AA)

        # 偏移量箭头
        arrow_start = (int(frame_center_x), h - 30)
        arrow_end = (int(center_x), h - 30)
        cv2.arrowedLine(vis, arrow_start, arrow_end,
                        (0, 255, 255), 2, tipLength=0.3)

    # --- 顶部信息栏 ---
    _draw_info_bar(vis, result, config)

    # --- 底部偏移指示条 ---
    _draw_offset_bar(vis, result, config)

    return vis


def _draw_info_bar(vis: np.ndarray, result: CenterLineResult,
                   config: Config):
    """绘制信息栏"""
    h, w = vis.shape[:2]

    # 半透明背景
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 0), (w, 40), (0, 0, 0), -1)
    vis[:] = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)

    # 文字信息
    if result.detected:
        offset_str = f"OFFSET: {result.offset_mm:+.1f}mm"
        angle_str = f"ANGLE: {result.angle_deg:+.1f}deg"
        conf_str = f"CONF: {result.confidence:.2f}"
        time_str = f"T: {result.processing_time_ms:.0f}ms"
    else:
        offset_str = "OFFSET: ---"
        angle_str = "ANGLE: ---"
        conf_str = "CONF: 0.00"
        time_str = f"T: {result.processing_time_ms:.0f}ms"

    cv2.putText(vis, offset_str, (5, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(vis, angle_str, (190, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(vis, conf_str, (370, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(vis, time_str, (500, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)


def _draw_offset_bar(vis: np.ndarray, result: CenterLineResult,
                     config: Config):
    """绘制底部偏移指示条 (像车道保持的仪表盘)"""
    h, w = vis.shape[:2]
    bar_y = h - 25
    bar_w = w - 40
    bar_x = 20

    # 背景条
    cv2.rectangle(vis, (bar_x, bar_y - 5), (bar_x + bar_w, bar_y + 15),
                  (40, 40, 40), -1)
    cv2.rectangle(vis, (bar_x, bar_y - 5), (bar_x + bar_w, bar_y + 15),
                  (80, 80, 80), 1)

    # 中心标记 (白色三角)
    center_x = bar_x + bar_w // 2
    pts = np.array([
        [center_x, bar_y - 8],
        [center_x - 6, bar_y + 17],
        [center_x + 6, bar_y + 17],
    ], dtype=np.int32)
    cv2.fillPoly(vis, [pts], (255, 255, 255))

    # 偏移指示 (黄色三角，位置反映偏移量)
    if result.detected:
        # 将偏移映射到指示条位置
        max_px = w / 4.0  # 最大显示偏移
        indicator_offset = np.clip(result.offset_px / max_px, -1.0, 1.0)
        indicator_x = center_x + int(indicator_offset * bar_w / 2)

        pts = np.array([
            [indicator_x, bar_y + 2],
            [indicator_x - 6, bar_y - 10],
            [indicator_x + 6, bar_y - 10],
        ], dtype=np.int32)
        cv2.fillPoly(vis, [pts], (0, 255, 255))

    # 刻度标记
    for i in range(-4, 5):
        tick_x = center_x + i * bar_w // 8
        cv2.line(vis, (tick_x, bar_y - 2), (tick_x, bar_y + 12),
                 (100, 100, 100), 1)


def _draw_status_bar(frame: np.ndarray, result: CenterLineResult,
                     config: Config):
    """在原图底部绘制简要状态栏"""
    h, w = frame.shape[:2]
    bar_h = 30

    # 半透明背景
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(frame, 0.6, overlay, 0.4, 0)

    status = "DETECTED" if result.detected else "SEARCHING"
    color = (0, 255, 0) if result.detected else (0, 0, 255)
    cv2.putText(frame, f"STATUS: {status}", (5, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


# ═══════════════════════════════════════════════════════════════
# 相机
# ═══════════════════════════════════════════════════════════════

def apply_camera_exposure(camera_id: int, cc) -> None:
    """用 v4l2 设置曝光。自动曝光让相机按环境亮度自适应。"""
    dev = f"/dev/video{camera_id}"
    if not os.path.exists(dev):
        print(f"[Camera] 跳过曝光设置: {dev} 不存在")
        return

    def _set(args):
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", dev] + args,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5,
            )
            return r.returncode == 0, (r.stderr or b"").decode("utf-8", "replace").strip()
        except Exception as e:
            return False, str(e)

    if cc.auto_exposure and not getattr(cc, "adaptive_exposure", False):
        # 3 = Aperture Priority Mode (连续自动曝光，随环境变化)
        ok, err = _set(["-c", "exposure_auto=3"])
        mode = "AUTO"
    else:
        ok, err = _set([
            "-c", "exposure_auto=1",
            "-c", f"exposure_absolute={int(cc.exposure_absolute)}",
            "-c", f"gain={int(cc.gain)}",
        ])
        mode = (
            f"ADAPTIVE exp={cc.exposure_absolute} gain={cc.gain}"
            if getattr(cc, "adaptive_exposure", False)
            else f"MANUAL exp={cc.exposure_absolute}"
        )

    if ok:
        print(f"[Camera] 曝光={mode}")
    else:
        print(f"[Camera] 曝光设置失败: {err or 'unknown'}")

    awb = "1" if cc.auto_white_balance else "0"
    ok, err = _set(["-c", f"white_balance_temperature_auto={awb}"])
    if ok:
        print(f"[Camera] AWB={'on' if cc.auto_white_balance else 'off'}")
    else:
        print(f"[Camera] AWB设置跳过: {err or 'unsupported'}")
# 2026-08-19 all-auto: removed hardcoded brightness/contrast/sharpness/backlight/power_line (keep firmware defaults like host preview)


    # 对焦控件因相机而异；户外跟槽距离相对固定，关掉 AF 防拉焦“呼吸”
    if cc.focus_auto:
        ok, err = _set(["-c", "focus_auto=1"])
        print(f"[Camera] AF={'on' if ok else 'unsupported'}"
              + (f" ({err})" if err and not ok else ""))
    else:
        ok1, err1 = _set(["-c", "focus_auto=0"])
        ok2, err2 = _set(["-c", f"focus_absolute={int(cc.focus_absolute)}"])
        print(
            f"[Camera] AF=off focus={int(cc.focus_absolute)} "
            f"ok={bool(ok1 and ok2)}"
            + (f" ({err1 or err2})" if (err1 or err2) else "")
        )


class _AdaptiveCameraOptimizer:
    """后台亮度闭环：限制曝光时间，避免硬件AE拉到长曝光导致运动模糊。"""

    def __init__(self, camera_id: int, cc):
        self.dev = f"/dev/video{camera_id}"
        self.cc = cc
        self.exposure = int(cc.exposure_absolute)
        self.gain = int(cc.gain)
        self.brightness = int(getattr(cc, "brightness", 0))
        self.interval = max(0.25, float(getattr(cc, "camera_adjust_interval_s", 0.8)))
        self._last_submit = 0.0
        self._luma_ema = None
        self._dark_streak = 0
        self._bright_streak = 0
        self._q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        now = time.monotonic()
        if frame is None or now - self._last_submit < self.interval:
            return
        self._last_submit = now
        try:
            self._q.put_nowait(frame.copy())
        except queue.Full:
            pass

    def _set(self):
        args = [
            "v4l2-ctl", "-d", self.dev,
            "-c", "exposure_auto=1",
            "-c", f"exposure_absolute={int(self.exposure)}",
            "-c", f"gain={int(self.gain)}",
            "-c", f"brightness={int(self.brightness)}",
            "-c", "backlight_compensation=0",
        ]
        try:
            subprocess.run(
                args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=1.5,
            )
        except Exception:
            pass

    def _run(self):
        target = float(getattr(self.cc, "target_luma", 112.0))
        exp_lo = int(getattr(self.cc, "exposure_min", 40))
        exp_hi = int(getattr(self.cc, "exposure_max", 260))
        gain_lo = int(getattr(self.cc, "gain_min", 20))
        gain_hi = int(getattr(self.cc, "gain_max", 180))
        bri_lo = -64
        bri_hi = 16
        while not self._stop.is_set():
            try:
                frame = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            h, w = frame.shape[:2]
            # 水槽主要在中央和下方；避开天空/桨叶对曝光计量的干扰
            roi = frame[int(h * 0.18):int(h * 0.95),
                        int(w * 0.12):int(w * 0.88)]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # p75：兼顾白色PVC与太阳高光，避免只追镜面闪点
            luma_raw = float(np.percentile(gray, 75))
            highlights = float(np.mean(gray >= 245))
            sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            if self._luma_ema is None:
                self._luma_ema = luma_raw
            else:
                self._luma_ema = 0.65 * self._luma_ema + 0.35 * luma_raw
            luma = float(self._luma_ema)

            error = target - luma
            changed = False
            # 大面积过曝/太阳：压暗快；变暗恢复慢，防来回抽
            if highlights > 0.08 or luma >= 230 or error < -12.0:
                self._bright_streak += 1
                self._dark_streak = 0
                if self._bright_streak >= 1:
                    if self.gain > gain_lo:
                        self.gain = max(gain_lo, self.gain - 6)
                    new_exp = max(exp_lo, int(self.exposure * 0.75))
                    if new_exp == self.exposure and self.exposure > exp_lo:
                        new_exp = max(exp_lo, self.exposure - max(1, self.exposure // 6))
                    self.exposure = new_exp
                    if self.exposure <= exp_lo and highlights > 0.12 and self._bright_streak >= 2:
                        self.brightness = max(bri_lo, self.brightness - 4)
                    changed = True
            elif error > 18.0:
                self._dark_streak += 1
                self._bright_streak = 0
                # 连续偏暗才回升，避免刚压完立刻拉爆
                if self._dark_streak >= 3:
                    base_bri = int(getattr(self.cc, "brightness", 0))
                    if self.brightness < base_bri:
                        self.brightness = min(bri_hi, self.brightness + 3, base_bri)
                        changed = True
                    elif self.exposure < exp_hi:
                        step = 1 if self.exposure < 8 else max(1, self.exposure // 10)
                        self.exposure = min(exp_hi, self.exposure + step)
                        changed = True
                    else:
                        self.gain = min(gain_hi, self.gain + 2)
                        changed = True
            else:
                self._bright_streak = 0
                self._dark_streak = 0

            if changed:
                self._set()
            print(
                f"[CameraOpt] luma={luma:.0f}/{target:.0f} "
                f"exp={self.exposure} gain={self.gain} bri={self.brightness} "
                f"hi={highlights:.2f} sharp={sharp:.0f}"
            )

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


def open_camera(config: Config):
    """打开相机 (优先GStreamer, 失败则fallback到OpenCV)"""
    cc = config.camera
    cap = None

    # 打开前先设曝光，避免被上次手动参数锁死
    apply_camera_exposure(cc.camera_id, cc)

    # 尝试GStreamer pipeline (RK3588硬件加速)
    if cc.gst_pipeline:
        try:
            cap = cv2.VideoCapture(cc.gst_pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                print("[Camera] 使用GStreamer pipeline")
                apply_camera_exposure(cc.camera_id, cc)
                return cap
        except Exception:
            print("[Camera] GStreamer失败，尝试V4L2/USB...")

    # Fallback: 普通摄像头（OpenCV3 不支持 CAP_V4L2 第二参数）
    # 必须钉死 MJPG：部分 UVC 在手动极低曝光后会落到 YUYV，OpenCV 解码成绿屏
    try:
        cap = cv2.VideoCapture(cc.camera_id, cv2.CAP_V4L2)
    except TypeError:
        cap = cv2.VideoCapture(cc.camera_id)
    mjpg = cv2.VideoWriter_fourcc(*"MJPG")
    try:
        cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减缓存延迟，画面更跟手
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cc.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cc.height)
    cap.set(cv2.CAP_PROP_FPS, min(cc.fps, 30))
    # 分辨率设置后部分驱动会改回 YUYV，再钉一次 MJPG
    try:
        cap.set(cv2.CAP_PROP_FOURCC, mjpg)
    except Exception:
        pass
    # 注意: 不能再在此处用外部 v4l2-ctl --set-fmt 改设备格式!
    # OpenCV 3.3.1 打开后已被外部进程改格式, buffer 与设备格式不一致,
    # cap.read() 会长时间 select timeout (实测 FPS 0.1)。格式交给 OpenCV 自管。

    if cap.isOpened():
        # OpenCV 打开后可能改写控件，再强制一次自动曝光/对焦
        apply_camera_exposure(cc.camera_id, cc)
        # v4l2-ctl 不可用时仍尝试 OpenCV UVC 属性；失败不会影响采集
        try:
            cap.set(cv2.CAP_PROP_AUTO_WB, 1 if cc.auto_white_balance else 0)
        except Exception:
            pass
        try:
            if hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if cc.focus_auto else 0)
            if (not cc.focus_auto) and hasattr(cv2, "CAP_PROP_FOCUS"):
                cap.set(cv2.CAP_PROP_FOCUS, float(cc.focus_absolute))
        except Exception:
            pass
        # OpenCV 设完后再用 v4l2 钉死一次（部分 UVC 会被 CAP_PROP 拉回 AF）
        if not cc.focus_auto:
            apply_camera_exposure(cc.camera_id, cc)
        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fourcc_v = int(cap.get(cv2.CAP_PROP_FOURCC) or 0)
        fourcc_s = "".join(chr((fourcc_v >> 8 * i) & 0xFF) for i in range(4))
        print(f"[Camera] USB/V4L2 摄像头 {actual_w}x{actual_h} fourcc={fourcc_s}")
        return cap

    raise RuntimeError("无法打开任何摄像头!")


# ═══════════════════════════════════════════════════════════════
# 主循环
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="PVC水槽中心线检测")
    parser.add_argument("--video", type=str, default=None,
                        help="使用视频文件 (调试用)")
    parser.add_argument("--image", type=str, default=None,
                        help="处理单张图片")
    parser.add_argument("--no-control", action="store_true",
                        help="不输出飞控指令")
    parser.add_argument("--no-preview", action="store_true",
                        help="不显示预览窗口")
    parser.add_argument("--camera-id", type=int, default=None,
                        help="摄像头ID (默认使用config中的值)")
    parser.add_argument("--platform", type=str, default=None,
                        choices=["rk3588", "jetson"],
                        help="jetson=Amov Jetson+PX4 配置覆盖")
    args = parser.parse_args()

    # --- 每次飞行会话目录（日志+录像，不用 /tmp）---
    session = _setup_flight_session()
    log_path = _setup_persistent_log(session["dir"] if session else None)

    # --- 配置 ---
    config = default_config
    if args.platform == "jetson":
        from config_jetson import apply_jetson_px4
        apply_jetson_px4(config)
        print("[Platform] Jetson + PX4 OFFBOARD")
    if args.camera_id is not None:
        config.camera.camera_id = args.camera_id

    # --- 初始化 ---
    detector = TroughDetector(config)
    controller = FlightController(config)  # 始终创建，用于MAVLink遥测
    enable_control = not args.no_control   # --no-control只禁止发送RC指令

    # --- 启动Web预览服务器 ---
    start_server(host="0.0.0.0", port=8080)

    print("=" * 60)
    print("PVC水槽中心线检测系统")
    print("=" * 60)
    if session:
        print(f"飞行会话: {session['dir']}")
    if log_path:
        print(f"日志文件: {log_path}")
    print(f"Canny: auto_threshold={config.canny.auto_threshold}")
    print(f"Hough: threshold={config.hough.threshold}, "
          f"minLine={config.hough.min_line_length}")
    print(f"飞行高度: {config.camera.mount_height_m}m")
    print(f"水槽宽度: {config.trough.trough_width_mm}mm")
    print(f"控制输出: {'禁用' if args.no_control else '启用'}")
    print("=" * 60)

    # --- 单张图片模式 ---
    if args.image:
        process_image(args.image, detector, config)
        return

    # --- 相机/视频 ---
    if args.video:
        cap = cv2.VideoCapture(args.video)
        print(f"[Input] 视频文件: {args.video}")
    else:
        cap = open_camera(config)

    if not cap.isOpened():
        print("错误: 无法打开视频源")
        sys.exit(1)

    camera_optimizer = None
    if (not args.video
            and bool(getattr(config.camera, "adaptive_exposure", False))):
        camera_optimizer = _AdaptiveCameraOptimizer(
            config.camera.camera_id, config.camera
        )
        print("[Camera] 软件自适应曝光已启动（运动模糊限幅）")

    # --- 飞行录像（叠加 + 可选原图，后台写入）---
    recorder = None
    raw_recorder = None
    writer = None  # 兼容旧本地预览路径
    if config.debug.save_video:
        vpath = config.debug.video_output.strip() if config.debug.video_output else ""
        if not vpath:
            vpath = session["video"] if session else os.path.join(
                _logs_root() or ".", f"overlay_{time.strftime('%Y%m%d_%H%M%S')}.avi"
            )
        recorder = _FlightVideoRecorder(
            vpath,
            width=config.camera.width,
            height=config.camera.height,
            # every_n=2 @~30fps → 约 15fps 内容；时间基与采样对齐，播放不忽快忽慢
            fps=15.0,
            every_n=int(getattr(config.debug, "record_every_n", 2) or 2),
        )
    if bool(getattr(config.debug, "save_raw_video", False)):
        rpath = session["raw_video"] if session else os.path.join(
            _logs_root() or ".", f"raw_{time.strftime('%Y%m%d_%H%M%S')}.avi"
        )
        raw_recorder = _FlightVideoRecorder(
            rpath,
            width=config.camera.width,
            height=config.camera.height,
            fps=15.0,
            every_n=int(getattr(config.debug, "raw_record_every_n", 2) or 2),
        )
        print(f"[Record] 原图训练录像 → {rpath}")

    # --- 交互参数 ---
    canny_high = config.canny.high_threshold
    hough_threshold = config.hough.threshold
    pid_active = enable_control

    # --- 预览后台线程 (编码不堵控制环) ---
    preview_q = queue.Queue(maxsize=1)
    preview_stop = threading.Event()

    def _preview_worker():
        while not preview_stop.is_set():
            try:
                item = preview_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            try:
                fr, res, c_pts, cfg = item
                web_disp = _build_web_preview_lite(fr, res, c_pts, cfg)
                # 叠加录像 + 无烧录原图（训练用）
                if recorder is not None:
                    recorder.submit(fr, web_disp)
                if raw_recorder is not None:
                    raw_recorder.submit(fr, None)
                wh, ww = web_disp.shape[:2]
                target_w = max(320, int(getattr(cfg.debug, "preview_width", 480)))
                if ww > target_w:
                    nh = max(1, int(round(wh * target_w / ww)))
                    web_disp = cv2.resize(
                        web_disp, (target_w, nh), interpolation=cv2.INTER_AREA
                    )
                preview_state.update_frame(
                    None, web_disp,
                    jpeg_quality=int(getattr(cfg.debug, "preview_jpeg_quality", 52)),
                )
            except Exception:
                pass

    preview_thread = threading.Thread(target=_preview_worker, daemon=True)
    preview_thread.start()

    # --- 主循环 ---
    frame_idx = 0
    fps_counter = 0
    fps_timer = time.perf_counter()
    current_fps = 0.0
    lost_streak = 0  # 连续读帧失败计数

    frame_start = time.perf_counter()
    actual_dt = 0.033  # 初始默认值

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # 相机偶发丢帧/读取超时: 跳过不退出;
                # 连续失败超过阈值才判定断流, 避免单次卡顿直接停机
                lost_streak += 1
                if lost_streak > 30:
                    print("视频断流")
                    break
                continue
            lost_streak = 0

            frame_idx += 1
            fps_counter += 1

            # --- 调整分辨率 ---
            if frame.shape[1] != config.camera.width:
                frame = cv2.resize(frame, (config.camera.width, config.camera.height))

            if camera_optimizer is not None:
                camera_optimizer.submit(frame)

            # --- 检测 ---
            result = detector.detect(frame)

            # --- 控制：仅真检测跟线；hold/丢检=不安全 → 衰减，并交给模式切 POSITION 悬停 ---
            cmd = None
            held = bool(getattr(result, "held", False))
            safe_track = bool(result.detected) and (not held)
            if controller and enable_control:
                if pid_active:
                    if safe_track:
                        cmd = controller.update(
                            result.offset_mm,
                            result.angle_deg,
                            result.confidence,
                            held=False,
                        )
                        if cmd is None or not cmd.active:
                            cmd = controller.idle_decay()
                    else:
                        cmd = controller.idle_decay()
                    controller.send_command(cmd)
                elif controller._fc_is_px4() and controller._fc_armed:
                    # PID 暂停时仍灌零速，避免 PX4 OFFBOARD 因断流被踢回
                    controller._send_body_velocity_raw(0.0, 0.0, 0.0)

            # --- MAVLink遥测；无新消息时也跑安全计时，避免丢线超时卡住 ---
            if controller and controller._mav_conn:
                telem = controller.read_telemetry()
                if telem:
                    preview_state.update_telemetry(**telem)
                controller.check_mode_transition(
                    telem.get('fc_mode', 'UNKNOWN') if telem else 'UNKNOWN',
                    telem.get('fc_armed', False) if telem else controller._fc_armed,
                    safe_track,
                    frame_dt=actual_dt,
                )

            # 终端/文件日志
            if config.debug.print_offset and frame_idx % 10 == 0:
                rng = float(getattr(preview_state, "rng_alt", 0.0) or 0.0)
                baro = float(getattr(preview_state, "altitude", 0.0) or 0.0)
                rng_q = int(getattr(preview_state, "rng_quality", 0) or 0)
                mode = str(getattr(preview_state, "fc_mode", "?"))
                alt_str = f"baro={baro:.2f}m rng={rng:.2f}m q={rng_q}"
                if result.detected:
                    roll_str = f"vy={cmd.roll_correction:+.2f}" if cmd else ""
                    yaw_str = f"yaw={cmd.yaw_correction:+.3f}" if cmd else ""
                    vx_str = f"vx={cmd.forward_speed:+.2f}" if cmd else ""
                    pid_str = "[CTRL]" if pid_active else "[MON]"
                    print(f"[{frame_idx:04d}] {pid_str} "
                          f"Offset={result.offset_mm:+.1f}mm "
                          f"Angle={result.angle_deg:+.1f}° "
                          f"Conf={result.confidence:.2f} "
                          f"{vx_str} {roll_str} {yaw_str} "
                          f"{alt_str} mode={mode} "
                          f"Time={result.processing_time_ms:.0f}ms")
                else:
                    print(f"[{frame_idx:04d}] [LOST] 未检测到水槽 "
                          f"{alt_str} mode={mode} "
                          f"({result.processing_time_ms:.0f}ms)")

            # --- Web预览: 丢给后台线程，不堵检测/控制 ---
            preview_n = max(1, int(getattr(config.debug, "preview_every_n", 2)))
            if frame_idx % preview_n == 0 and preview_q is not None:
                c_pts = getattr(detector, "_last_center_pts", None)
                try:
                    preview_q.put_nowait((
                        frame.copy(),
                        result,
                        None if c_pts is None else np.asarray(c_pts).copy(),
                        config,
                    ))
                except queue.Full:
                    pass

            # --- Web预览: 更新检测结果 ---
            preview_state.update_detection(
                offset_mm=result.offset_mm,
                angle_deg=result.angle_deg,
                confidence=result.confidence,
                detected=result.detected,
                processing_ms=result.processing_time_ms,
                detect_fps=current_fps,
                frame_count=detector.frame_count,
                detect_count=detector.detect_count,
                pid_active=pid_active,
                roll_correction=cmd.roll_correction if cmd else 0.0,
                yaw_correction=cmd.yaw_correction if cmd else 0.0,
                forward_speed=cmd.forward_speed if cmd else 0.0,
                forward_align=getattr(cmd, "forward_align", 0.0) if cmd else 0.0,
            )

            # --- 心跳年龄累加 ---
            if controller and controller._mav_conn:
                current_age = preview_state.heartbeat_age
                if current_age < 999:
                    preview_state.update_telemetry(heartbeat_age=current_age + actual_dt)

            # --- 真实帧间隔 (用于下一帧) ---
            now = time.perf_counter()
            actual_dt = now - frame_start
            frame_start = now

            # --- 可视化 ---
            if not args.no_preview:
                # 原始画面 + ROI
                frame_disp = frame.copy()
                draw_result(frame_disp, result, config)

                # 鸟瞰/Canny 调试仅传统视觉路径；YOLO 或缺方法时跳过，避免崩溃
                warped_disp = None
                if (not config.yolo.enabled
                        and hasattr(detector, "_canny")
                        and hasattr(detector, "_apply_roi")):
                    try:
                        warped = detector._warp_perspective(
                            detector._preprocess(frame)
                        )
                        edges = detector._canny(warped)
                        edges_roi = detector._apply_roi(edges)
                        warped_disp = draw_warped_view(
                            warped, edges_roi, result, config
                        )
                    except Exception:
                        warped_disp = None

                # 并排显示
                if warped_disp is not None:
                    combined = _stack_horizontal(frame_disp, warped_disp)
                else:
                    combined = frame_disp

                cv2.imshow("Trough CenterLine Detector", combined)

                # 保存视频（本地预览兼容路径；正常走 recorder）
                if writer:
                    writer.write(combined)

            # --- 键盘控制 (仅预览模式) ---
            if not args.no_preview:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q 或 ESC
                    break
                elif key == ord('s'):
                    # 保存截图
                    fname = f"screenshot_{frame_idx:04d}.jpg"
                    cv2.imwrite(fname, frame_disp)
                    print(f"截图已保存: {fname}")
                elif key == ord('r'):
                    detector.reset()
                    if controller:
                        controller.reset()
                    print("检测器已重置")
                elif key == ord('+') or key == ord('='):
                    canny_high = min(255, canny_high + 10)
                    config.canny.auto_threshold = False
                    config.canny.high_threshold = canny_high
                    config.canny.low_threshold = canny_high // 3
                    print(f"Canny high_threshold: {canny_high}")
                elif key == ord('-'):
                    canny_high = max(10, canny_high - 10)
                    config.canny.auto_threshold = False
                    config.canny.high_threshold = canny_high
                    config.canny.low_threshold = canny_high // 3
                    print(f"Canny high_threshold: {canny_high}")
                elif key == ord(']'):
                    hough_threshold += 5
                    config.hough.threshold = hough_threshold
                    print(f"Hough threshold: {hough_threshold}")
                elif key == ord('['):
                    hough_threshold = max(5, hough_threshold - 5)
                    config.hough.threshold = hough_threshold
                    print(f"Hough threshold: {hough_threshold}")
                elif key == ord(' '):
                    pid_active = not pid_active
                    status = "激活" if pid_active else "暂停"
                    print(f"PID控制: {status}")
                    if not pid_active and controller:
                        controller.reset()
            else:
                # 无头模式: 简单延时，用KeyboardInterrupt退出
                time.sleep(0.001)

            # --- FPS计算 ---
            if time.perf_counter() - fps_timer > 2.0:
                current_fps = fps_counter / (time.perf_counter() - fps_timer)
                fps_counter = 0
                fps_timer = time.perf_counter()
                if not args.no_preview:
                    cv2.setWindowTitle(
                        "Trough CenterLine Detector",
                        f"Trough Detector | FPS: {current_fps:.1f}"
                    )
                else:
                    print(f"[FPS: {current_fps:.1f}]")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        try:
            preview_stop.set()
            preview_q.put_nowait(None)
        except Exception:
            pass
        if recorder is not None:
            try:
                recorder.close()
            except Exception:
                pass
        if raw_recorder is not None:
            try:
                raw_recorder.close()
            except Exception:
                pass
        if camera_optimizer is not None:
            try:
                camera_optimizer.close()
            except Exception:
                pass
        cap.release()
        if writer:
            writer.release()
        if controller:
            controller.close()
        if not args.no_preview:
            cv2.destroyAllWindows()
        print(f"已退出. 总帧数: {frame_idx}, 检测成功: {detector.detect_count}")
        if session:
            print(f"本场数据: {session['dir']}")


def _build_web_preview_lite(frame, result, c_pts=None, cfg=None, mask=None):
    """后台预览用：只画线，不碰 detector 状态。"""
    h, w = frame.shape[:2]
    vis = frame.copy()
    # ★ 2026-09-20: 把模型的原始分割 mask 半透明叠上来(蓝色)。
    #   用途 —— 判断"检测歪了"到底是哪一类:
    #     mask 盖住了地板/背景   -> 模型分割错了   (要动模型/训练数据)
    #     mask 只盖住真槽, 线却偏 -> 模型是对的, 是后处理把线画偏了 (改代码)
    #   没这一步就只能看到后处理画出的线, 分不清是模型还是后处理的锅。
    if mask is not None:
        try:
            _m = mask
            if _m.ndim == 3:
                _m = _m[:, :, 0]
            if _m.shape[0] != h or _m.shape[1] != w:
                _m = cv2.resize(_m, (w, h), interpolation=cv2.INTER_NEAREST)
            _mb = _m > 0
            if _mb.any():
                vis[_mb] = (vis[_mb].astype(np.float32) * 0.55
                            + np.array([255.0, 0.0, 0.0], np.float32) * 0.45
                            ).astype(np.uint8)
        except Exception:
            pass
    cv2.line(vis, (w // 2, 0), (w // 2, h - 1), (0, 255, 0), 2)
    # 底部忽略区（喷管）分界线
    try:
        ignore = float(getattr(getattr(cfg, "yolo", None), "mask_bottom_ignore", 0.0) or 0.0)
    except Exception:
        ignore = 0.0
    if ignore > 0.05:
        y_cut = int(h * (1.0 - ignore))
        cv2.line(vis, (0, y_cut), (w - 1, y_cut), (255, 128, 0), 2)
        cv2.putText(
            vis, "ROI ignore below", (8, max(18, y_cut - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 128, 0), 1, cv2.LINE_AA,
        )
    if result.detected and result.left_points and result.right_points:
        l_pts = np.asarray(result.left_points, dtype=np.float64)
        r_pts = np.asarray(result.right_points, dtype=np.float64)
        if c_pts is None or len(c_pts) < 2:
            c_pts = 0.5 * (l_pts + r_pts)
        else:
            c_pts = np.asarray(c_pts, dtype=np.float64)
        if len(l_pts) >= 2:
            cv2.polylines(vis, [l_pts.astype(np.int32)], False, (0, 255, 255), 2)
        if len(r_pts) >= 2:
            cv2.polylines(vis, [r_pts.astype(np.int32)], False, (0, 255, 255), 2)
        if len(c_pts) >= 2:
            cv2.polylines(vis, [c_pts.astype(np.int32)], False, (0, 0, 255), 3)
            y_bot = float(c_pts[int(len(c_pts) * 0.75), 1])
            idx = int(np.argmin(np.abs(c_pts[:, 1] - y_bot)))
            cx_bot = float(c_pts[idx, 0])
            y_bot = float(c_pts[idx, 1])
            cv2.circle(vis, (int(round(cx_bot)), int(round(y_bot))), 7, (0, 0, 255), -1)
            cv2.circle(vis, (w // 2, int(round(y_bot))), 9, (0, 255, 0), 2)
        # 2026-09-24: hold 帧画的是卡尔曼预测的线, 标出来免得被当成新鲜测量
        if getattr(result, "held", False):
            cv2.putText(
                vis,
                f"HOLD(pred) off={result.offset_mm:+.1f}mm ang={result.angle_deg:+.1f}",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 2,
            )
        else:
            cv2.putText(
                vis,
                f"SEG off={result.offset_mm:+.1f}mm ang={result.angle_deg:+.1f} "
                f"conf={result.confidence:.2f}",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
            )
    elif result.detected:
        # ★ 2026-09-18 改: 原来这里画一条橙色竖线表示 hold, 但那根线的位置是拿
        #   【上一帧的 offset_px】算出来的 —— 没检测到时它指向旧位置, 看起来就是
        #   "乱指/脱槽"。按"没检测到就不画中心线"的原则, 去掉线, 只留文字提示。
        cv2.putText(
            vis,
            f"HOLD off={result.offset_mm:+.1f}mm ang={result.angle_deg:+.1f}",
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 2,
        )
    else:
        cv2.putText(
            vis, "SEG no mask", (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 2,
        )
    cv2.putText(
        vis, f"{result.processing_time_ms:.0f}ms",
        (5, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1,
    )
    return vis


def _build_web_preview(frame, result, detector, config):
    """
    Web预览:
      YOLO模式 — 与离线一致: 原图 + seg mask 着色 + 左右边/中心线
      HSV模式  — 原图ROI + 鸟瞰白区(旧逻辑)
    """
    h, w = frame.shape[:2]
    dc = config.debug

    # ── YOLO-seg: 轻量叠加(跳过整图 mask 着色，减卡顿) ──
    if config.yolo.enabled:
        vis = frame
        # 相机中心参考线(绿)
        vis = frame.copy()
        cv2.line(vis, (w // 2, 0), (w // 2, h - 1), (0, 255, 0), 2)

        if result.detected and result.left_points and result.right_points:
            l_pts = np.asarray(result.left_points, dtype=np.float64)
            r_pts = np.asarray(result.right_points, dtype=np.float64)
            c_pts = getattr(detector, "_last_center_pts", None)
            if c_pts is None or len(c_pts) < 2:
                c_pts = 0.5 * (l_pts + r_pts)
            else:
                c_pts = np.asarray(c_pts, dtype=np.float64)

            if len(l_pts) >= 2:
                cv2.polylines(vis, [l_pts.astype(np.int32)], False, (0, 255, 255), 2)
            if len(r_pts) >= 2:
                cv2.polylines(vis, [r_pts.astype(np.int32)], False, (0, 255, 255), 2)
            if len(c_pts) >= 2:
                cv2.polylines(vis, [c_pts.astype(np.int32)], False, (0, 0, 255), 3)

            y_bot = float(c_pts[int(len(c_pts) * 0.75), 1])
            idx = int(np.argmin(np.abs(c_pts[:, 1] - y_bot)))
            cx_bot = float(c_pts[idx, 0])
            y_bot = float(c_pts[idx, 1])
            cv2.circle(vis, (int(round(cx_bot)), int(round(y_bot))), 7, (0, 0, 255), -1)
            cv2.circle(vis, (w // 2, int(round(y_bot))), 9, (0, 255, 0), 2)

            cv2.putText(
                vis,
                f"SEG off={result.offset_mm:+.1f}mm ang={result.angle_deg:+.1f} "
                f"conf={result.confidence:.2f}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
        else:
            cv2.putText(
                vis,
                "SEG no mask",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 128, 255),
                2,
            )
        cv2.putText(
            vis,
            f"{result.processing_time_ms:.0f}ms",
            (5, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (200, 200, 200),
            1,
        )
        return vis

    # ── HSV传统: 原图ROI + 鸟瞰 ──
    tc = config.trough
    top = frame.copy()
    roi_pts = np.int32([
        [tc.perspective_src[0][0] * w, tc.perspective_src[0][1] * h],
        [tc.perspective_src[1][0] * w, tc.perspective_src[1][1] * h],
        [tc.perspective_src[2][0] * w, tc.perspective_src[2][1] * h],
        [tc.perspective_src[3][0] * w, tc.perspective_src[3][1] * h],
    ])
    cv2.polylines(top, [roi_pts], True, (100, 200, 100), 2)
    cv2.line(top, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    cv2.line(top, (0, h // 2), (w, h // 2), (255, 255, 255), 1)
    if result.detected:
        txt = f"DET: offset={result.offset_mm:+.1f}mm angle={result.angle_deg:+.1f}deg conf={result.confidence:.2f}"
        cv2.putText(top, txt, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    else:
        cv2.putText(top, "SEARCHING...", (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    gray = detector._preprocess(frame)
    warped_gray = detector._warp_perspective(gray)
    warped_color = detector._warp_perspective_color(frame)
    hsv = cv2.cvtColor(warped_color, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 100), (180, 40, 255))
    bot = warped_color.copy()
    bot[white_mask > 0] = (bot[white_mask > 0] * 0.5 + np.array([0, 200, 0]) * 0.5).astype(np.uint8)
    bh, bw = bot.shape[:2]
    frame_cx = bw / 2.0

    if result.raw_lines:
        for line in result.raw_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(bot, (x1, y1), (x2, y2), (255, 255, 255), 1, cv2.LINE_AA)
    for pt in result.left_points:
        cv2.circle(bot, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
    for pt in result.right_points:
        cv2.circle(bot, (int(pt[0]), int(pt[1])), 2, (0, 0, 255), -1)
    if result.detected and result.left_points and result.right_points:
        l_pts = np.int32(result.left_points)
        r_pts = np.int32(result.right_points)
        cv2.polylines(bot, [l_pts], False, (0, 255, 0), 3)
        cv2.polylines(bot, [r_pts], False, (0, 0, 255), 3)
        c_pts = np.int32((l_pts.astype(float) + r_pts.astype(float)) / 2.0)
        cv2.polylines(bot, [c_pts], False, (0, 255, 255), 2)
    else:
        cv2.putText(bot, "NO LINES FOUND", (int(frame_cx) - 70, bh // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.line(bot, (int(frame_cx), 0), (int(frame_cx), bh), (255, 255, 255), 1)
    top_scaled = cv2.resize(top, (bw, int(h * bw / w)))
    combined = np.vstack([top_scaled, bot])
    cv2.line(combined, (0, top_scaled.shape[0]), (bw, top_scaled.shape[0]), (80, 80, 80), 2)
    return combined


def process_image(path: str, detector: TroughDetector, config: Config):
    """处理单张图片"""
    frame = cv2.imread(path)
    if frame is None:
        print(f"无法读取图片: {path}")
        return

    result = detector.detect(frame)

    warped = detector._warp_perspective(detector._preprocess(frame))
    edges = detector._canny(warped)
    edges_roi = detector._apply_roi(edges)
    warped_disp = draw_warped_view(warped, edges_roi, result, config)

    cv2.imshow("Warped View", warped_disp)

    frame_disp = frame.copy()
    draw_result(frame_disp, result, config)
    cv2.imshow("Original", frame_disp)

    print(f"检测结果: {result}")
    print(f"偏移量: {result.offset_mm:+.1f}mm")
    print(f"角度: {result.angle_deg:+.1f}°")
    print(f"置信度: {result.confidence:.2f}")
    print(f"处理时间: {result.processing_time_ms:.0f}ms")
    print("按任意键退出...")

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def _stack_horizontal(img1, img2, target_width=1280):
    """水平拼接两个图像，缩放到统一宽度"""
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]

    # 统一高度
    target_h = min(h1, h2)

    # 缩放，保持宽高比
    scale1 = target_h / h1
    scale2 = target_h / h2
    new_w1 = int(w1 * scale1)
    new_w2 = int(w2 * scale2)

    img1_resized = cv2.resize(img1, (new_w1, target_h))
    img2_resized = cv2.resize(img2, (new_w2, target_h))

    combined = np.hstack([img1_resized, img2_resized])

    # 缩放到目标宽度
    if combined.shape[1] > target_width:
        scale = target_width / combined.shape[1]
        combined = cv2.resize(combined, (target_width, int(combined.shape[0] * scale)))

    return combined


if __name__ == "__main__":
    main()
