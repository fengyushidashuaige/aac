"""
Web实时预览系统 — MJPEG视频流 + 飞控遥测面板
基于Flask，无其他第三方依赖
"""

import threading
import time
import json
import cv2
import numpy as np
from flask import Flask, Response, request, render_template_string

# ──────────────────────────────────────────────
# 共享状态 (线程安全)
# ──────────────────────────────────────────────

class SharedState:
    """检测线程和Web线程之间的共享数据"""

    def __init__(self):
        self._lock = threading.Lock()

        # ── 视频帧 ──
        self._raw_jpeg: bytes = None          # 原始画面JPEG
        self._detection_jpeg: bytes = None    # 检测叠加画面JPEG
        self._frame_time: float = 0.0

        # ── 检测结果 ──
        self.offset_mm: float = 0.0
        self.angle_deg: float = 0.0
        self.confidence: float = 0.0
        self.detected: bool = False
        self.processing_ms: float = 0.0
        self.detect_fps: float = 0.0
        self.frame_count: int = 0
        self.detect_count: int = 0

        # ── 飞控遥测 ──
        self.fc_connected: bool = False
        self.fc_mode: str = "N/A"
        self.fc_armed: bool = False
        self.battery_voltage: float = 0.0
        self.battery_current: float = 0.0
        self.battery_remaining: int = 0
        self.heartbeat_age: float = 999.0
        self.altitude: float = 0.0
        self.rng_alt: float = 0.0          # 激光/测距仪高度 (m)
        self.rng_quality: int = 0          # 0-100，若无则 0
        self.groundspeed: float = 0.0
        self.heading: int = 0
        self.gps_fix: int = 0
        self.gps_lat: float = 0.0
        self.gps_lon: float = 0.0
        self.satellites: int = 0

        # ── 控制输出 ──
        self.roll_correction: float = 0.0
        self.yaw_correction: float = 0.0
        self.forward_speed: float = 0.0
        self.forward_align: float = 0.0
        self.pid_active: bool = False

    def update_frame(self, raw_frame: np.ndarray, detection_frame: np.ndarray,
                     jpeg_quality: int = 52):
        """更新视频帧 (从检测线程调用)。raw_frame 可为 None。"""
        t = time.perf_counter()
        quality = max(35, min(85, int(jpeg_quality)))
        raw_bytes = None
        det_bytes = None
        if raw_frame is not None:
            _, raw_jpg = cv2.imencode(
                '.jpg', raw_frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            raw_bytes = raw_jpg.tobytes()
        if detection_frame is not None:
            _, det_jpg = cv2.imencode(
                '.jpg', detection_frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            det_bytes = det_jpg.tobytes()
        with self._lock:
            if raw_bytes is not None:
                self._raw_jpeg = raw_bytes
            if det_bytes is not None:
                self._detection_jpeg = det_bytes
            self._frame_time = t

    def update_detection(self, **kwargs):
        """更新检测结果"""
        with self._lock:
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)

    def update_telemetry(self, **kwargs):
        """更新飞控遥测"""
        with self._lock:
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)

    def get_video_jpeg(self) -> bytes:
        """获取检测叠加画面的JPEG"""
        with self._lock:
            return self._detection_jpeg

    def get_telemetry(self) -> dict:
        """获取完整遥测快照"""
        with self._lock:
            return {
                "fc_connected": self.fc_connected,
                "fc_mode": self.fc_mode,
                "fc_armed": self.fc_armed,
                "battery_voltage": round(self.battery_voltage, 2),
                "battery_current": round(self.battery_current, 1),
                "battery_remaining": self.battery_remaining,
                "heartbeat_age": round(self.heartbeat_age, 2),
                "altitude": round(self.altitude, 1),
                "rng_alt": round(self.rng_alt, 2),
                "rng_quality": int(self.rng_quality),
                "groundspeed": round(self.groundspeed, 2),
                "heading": self.heading,
                "gps_fix": self.gps_fix,
                "gps_lat": round(self.gps_lat, 6),
                "gps_lon": round(self.gps_lon, 6),
                "satellites": self.satellites,
                "detected": self.detected,
                "offset_mm": round(self.offset_mm, 1),
                "angle_deg": round(self.angle_deg, 1),
                "confidence": round(self.confidence, 2),
                "processing_ms": round(self.processing_ms, 0),
                "detect_fps": round(self.detect_fps, 1),
                "frame_count": self.frame_count,
                "detect_count": self.detect_count,
                "roll_correction": round(self.roll_correction, 3),
                "yaw_correction": round(self.yaw_correction, 3),
                "forward_speed": round(self.forward_speed, 3),
                "forward_align": round(self.forward_align, 2),
                "pid_active": self.pid_active,
            }


# ──────────────────────────────────────────────
# Flask应用
# ──────────────────────────────────────────────

state = SharedState()
app = Flask(__name__)

# HTML模板 (内联，无需外部文件)
DASHBOARD_HTML = r"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>水槽无人机 · 地面站</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:#0d1117; color:#c9d1d9; font-family:'Segoe UI','PingFang SC',sans-serif;
         display:flex; justify-content:center; align-items:flex-start;
         min-height:100vh; padding:16px; gap:14px; overflow:auto; }
  /* === 左侧: 视频（中等窗口，不铺满屏） === */
  .video-panel { flex:0 0 auto; width:640px; max-width:70vw;
                 display:flex; flex-direction:column; background:#161b22;
                 border:1px solid #30363d; border-radius:8px; overflow:hidden;
                 position:relative; }
  .video-label { position:static; background:#21262d; color:#58a6ff;
                  padding:6px 12px; border-radius:0; font-size:12px;
                  border-bottom:1px solid #30363d; }
  .video-panel img { width:100%; height:auto; max-height:480px; object-fit:contain;
                     display:block; background:#000; }
  /* === 右侧: 仪表盘 === */
  .dash-panel { width:300px; flex:0 0 auto; background:#161b22; display:flex; flex-direction:column;
                overflow-y:auto; border:1px solid #30363d; border-radius:8px;
                padding:12px; gap:10px; max-height:calc(100vh - 32px); }
  .card { background:#21262d; border:1px solid #30363d; border-radius:6px; padding:10px 12px; }
  .card-title { font-size:12px; color:#8b949e; text-transform:uppercase; letter-spacing:1px;
                margin-bottom:8px; padding-bottom:5px; border-bottom:1px solid #30363d; }
  /* === 电池条 === */
  .battery-bar { height:16px; background:#30363d; border-radius:8px; overflow:hidden; margin:5px 0; }
  .battery-fill { height:100%; border-radius:8px; transition:width 0.5s; }
  .battery-low { background:#f85149; }
  .battery-mid { background:#d2991d; }
  .battery-ok { background:#3fb950; }
  /* === 数据行 === */
  .row { display:flex; justify-content:space-between; padding:3px 0; font-size:13px; }
  .row .label { color:#8b949e; }
  .row .value { color:#e6edf3; font-weight:600; font-variant-numeric:tabular-nums; }
  .row .value.green { color:#3fb950; }
  .row .value.red { color:#f85149; }
  .row .value.yellow { color:#d2991d; }
  .row .value.blue { color:#58a6ff; }
  /* === 检测状态 === */
  .detect-status { display:inline-block; padding:3px 10px; border-radius:4px; font-size:12px; font-weight:700; }
  .detect-ok { background:#3fb95020; color:#3fb950; }
  .detect-no { background:#f8514920; color:#f85149; }
  /* === 偏移指示条 === */
  .offset-bar-wrap { background:#30363d; height:11px; border-radius:6px; position:relative; margin:7px 0; }
  .offset-bar-center { position:absolute; left:50%; top:-2px; width:2px; height:15px; background:#fff; }
  .offset-bar-marker { position:absolute; top:-2px; width:9px; height:15px; border-radius:3px;
                       background:#58a6ff; transform:translateX(-50%); transition:left 0.3s; }
  @media (max-width:900px) {
    body { flex-direction:column; align-items:center; }
    .video-panel { width:min(640px, 96vw); }
    .dash-panel { width:min(640px, 96vw); max-height:none; }
  }
</style>
</head>
<body>

<!-- 左侧: 视频流 -->
<div class="video-panel">
  <div class="video-label">实时检测画面</div>
  <img id="video" src="/frame.jpg" alt="Video Stream" />
</div>

<!-- 右侧: 仪表盘 -->
<div class="dash-panel" id="dash">

  <!-- 飞控状态 -->
  <div class="card">
    <div class="card-title">🛩️ 飞控状态</div>
    <div class="row"><span class="label">模式</span><span class="value blue" id="fc_mode">--</span></div>
    <div class="row"><span class="label">解锁</span><span class="value" id="fc_armed">--</span></div>
    <div class="row"><span class="label">心跳</span><span class="value" id="heartbeat">--</span></div>
  </div>

  <!-- 电池 -->
  <div class="card">
    <div class="card-title">🔋 电池</div>
    <div class="row"><span class="label">电压</span><span class="value" id="bat_v">--</span></div>
    <div class="battery-bar"><div class="battery-fill" id="bat_bar" style="width:0%"></div></div>
    <div class="row"><span class="label">电流</span><span class="value" id="bat_a">--</span></div>
    <div class="row"><span class="label">剩余</span><span class="value" id="bat_pct">--</span></div>
  </div>

  <!-- 检测 -->
  <div class="card">
    <div class="card-title">🎯 中心线检测</div>
    <div style="text-align:center;margin-bottom:8px;">
      <span class="detect-status" id="detect_status">等待中</span>
    </div>
    <div class="row"><span class="label">横向偏移</span><span class="value" id="offset">--</span></div>
    <div class="row"><span class="label">航向偏差</span><span class="value" id="angle">--</span></div>
    <div class="row"><span class="label">置信度</span><span class="value" id="conf">--</span></div>
    <div class="row"><span class="label">处理耗时</span><span class="value" id="proc_ms">--</span></div>
    <div class="row"><span class="label">检测FPS</span><span class="value" id="det_fps">--</span></div>
    <!-- 偏移指示条 -->
    <div class="offset-bar-wrap">
      <div class="offset-bar-center"></div>
      <div class="offset-bar-marker" id="offset_marker" style="left:50%;"></div>
    </div>
    <div style="text-align:center;font-size:11px;color:#8b949e;margin-top:4px;">
      &larr; 水槽在左 &nbsp;&nbsp;|&nbsp;&nbsp; 水槽在右 &rarr;
    </div>
  </div>

  <!-- 控制输出 -->
  <div class="card">
    <div class="card-title">🎮 控制输出</div>
    <div class="row"><span class="label">PID状态</span><span class="value" id="pid_status">--</span></div>
    <div class="row"><span class="label">前进 vx</span><span class="value" id="vx">--</span></div>
    <div class="row"><span class="label">对齐系数</span><span class="value" id="align">--</span></div>
    <div class="row"><span class="label">横向 vy</span><span class="value" id="roll">--</span></div>
    <div class="row"><span class="label">偏航 yaw</span><span class="value" id="yaw">--</span></div>
  </div>

  <!-- GPS -->
  <div class="card">
    <div class="card-title">📍 GPS</div>
    <div class="row"><span class="label">定位</span><span class="value" id="gps_fix">--</span></div>
    <div class="row"><span class="label">卫星</span><span class="value" id="sats">--</span></div>
    <div class="row"><span class="label">气压高</span><span class="value" id="alt">--</span></div>
    <div class="row"><span class="label">激光高</span><span class="value" id="rng_alt">--</span></div>
    <div class="row"><span class="label">测距质量</span><span class="value" id="rng_q">--</span></div>
    <div class="row"><span class="label">速度</span><span class="value" id="speed">--</span></div>
  </div>

</div>

<script>
// 使用短连接拉取单帧，避免Jetson进程重启后浏览器的MJPEG长连接永久卡死。
const video = document.getElementById('video');
let videoBusy = false;
function refreshVideo() {
  if (!video || videoBusy) return;
  videoBusy = true;
  const done = () => { videoBusy = false; };
  video.onload = done;
  video.onerror = done;
  video.src = '/frame.jpg?t=' + Date.now();
}

// 每秒拉取遥测数据
const BAT_WARN = 14.4;   // 4S: 3.6V/cell
const BAT_MIN  = 14.0;   // 4S: 3.5V/cell

async function refresh() {
  try {
    const r = await fetch('/api/telemetry');
    const d = await r.json();

    // ── 飞控 ──
    set('fc_mode', d.fc_mode, d.fc_connected ? 'blue' : '');
    set('fc_armed', d.fc_armed ? '🔴 ARMED' : '⚫ DISARMED',
       d.fc_armed ? 'red' : '');
    set('heartbeat', d.fc_connected ? d.heartbeat_age.toFixed(1)+'s' : '断开',
       d.fc_connected ? (d.heartbeat_age<1?'green':'yellow') : 'red');
    set('pid_status', d.pid_active ? '✅ 激活' : '⏸️ 暂停',
       d.pid_active ? 'green' : '');

    // ── 电池 ──
    set('bat_v', d.battery_voltage.toFixed(2)+' V');
    set('bat_a', d.battery_current.toFixed(1)+' A');
    set('bat_pct', d.battery_remaining+'%');
    let bar = document.getElementById('bat_bar');
    if (bar) {
      let pct = d.battery_remaining;
      bar.style.width = Math.min(100, Math.max(0, pct)) + '%';
      bar.className = 'battery-fill ' + (
        pct < 20 ? 'battery-low' : pct < 50 ? 'battery-mid' : 'battery-ok');
    }

    // ── 检测 ──
    let statusEl = document.getElementById('detect_status');
    if (d.detected) {
      statusEl.textContent = '✅ 已检测';
      statusEl.className = 'detect-status detect-ok';
    } else {
      statusEl.textContent = '🔍 搜索中';
      statusEl.className = 'detect-status detect-no';
    }
    set('offset', (d.offset_mm>=0?'+':'')+d.offset_mm.toFixed(1)+' mm');
    set('angle', (d.angle_deg>=0?'+':'')+d.angle_deg.toFixed(1)+'°');
    set('conf', (d.confidence*100).toFixed(0)+'%');
    set('proc_ms', d.processing_ms.toFixed(0)+' ms');
    set('det_fps', d.detect_fps.toFixed(1)+' fps');

    // 偏移指示条 (映射 -200..+200mm → 5%..95%)
    let offsetPct = 50 + (d.offset_mm / 200) * 45;
    offsetPct = Math.max(5, Math.min(95, offsetPct));
    let marker = document.getElementById('offset_marker');
    if (marker) marker.style.left = offsetPct + '%';

    // ── 控制 ──
    set('vx', (d.forward_speed>=0?'+':'')+(d.forward_speed||0).toFixed(3)+' m/s',
        (d.forward_speed||0)>0.01?'green':((d.forward_speed||0)>0?'yellow':''));
    set('align', ((d.forward_align||0)*100).toFixed(0)+'%',
        (d.forward_align||0)>=0.8?'green':((d.forward_align||0)>=0.4?'yellow':'red'));
    set('roll', (d.roll_correction>=0?'+':'')+d.roll_correction.toFixed(3)+' m/s');
    set('yaw', (d.yaw_correction>=0?'+':'')+d.yaw_correction.toFixed(3)+' rad/s');

    // ── GPS (MAVLink fix_type: 0无 1无定位 2=2D 3=3D 4=DGPS 5=RTK浮点 6=RTK固定) ──
    const fixNames = ['无','无定位','2D','3D','DGPS','RTK浮点','RTK固定'];
    const fix = Number(d.gps_fix||0);
    const fixCls = fix>=6 ? 'green' : (fix>=5 ? 'green' : (fix>=3 ? 'yellow' : 'red'));
    set('gps_fix', fixNames[fix] || ('FIX'+fix), fixCls);
    set('sats', d.satellites);
    set('alt', (d.altitude||0).toFixed(2)+' m');
    const rng = (d.rng_alt!=null) ? Number(d.rng_alt) : 0;
    const rngQ = (d.rng_quality!=null) ? Number(d.rng_quality) : 0;
    // 激光高：无数据灰；与气压差大或质量低标黄/红，方便看跳动
    let rngCls = '';
    if (rng <= 0.01) rngCls = '';
    else if (rngQ > 0 && rngQ < 60) rngCls = 'yellow';
    else if (Math.abs(rng - (d.altitude||0)) > 0.35) rngCls = 'yellow';
    set('rng_alt', rng > 0.01 ? rng.toFixed(2)+' m' : '--', rngCls);
    set('rng_q', rngQ > 0 ? (rngQ+'%') : '--', rngQ>0 && rngQ<60 ? 'yellow' : '');
    set('speed', d.groundspeed.toFixed(2)+' m/s');

  } catch(e) { console.log('Telemetry fetch error:', e); }
}

function set(id, text, cls) {
  let el = document.getElementById(id);
  if (!el) return;
  if (text !== undefined) el.textContent = text;
  if (cls !== undefined) {
    el.className = 'value ' + cls;
  }
}

// 每秒刷新
setInterval(refresh, 500);
setInterval(refreshVideo, 67);  // 约15fps；失败后下一轮自动重连
refresh();
refreshVideo();
</script>

</body>
</html>
"""


@app.route('/')
def index():
    return render_template_string(DASHBOARD_HTML)


@app.route('/video_feed')
def video_feed():
    """MJPEG视频流"""
    def generate():
        while True:
            jpg = state.get_video_jpeg()
            if jpg:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
            else:
                # 还没帧时发送一个黑帧
                black = np.zeros((480, 640, 3), dtype=np.uint8)
                _, jpg = cv2.imencode('.jpg', black, [cv2.IMWRITE_JPEG_QUALITY, 50])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
            time.sleep(0.04)  # 最多约25fps，跟随逐帧Jetson预览

    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/frame.jpg')
def frame_jpeg():
    """返回最新一帧；浏览器短轮询比长期MJPEG连接更抗断线。"""
    jpg = state.get_video_jpeg()
    if not jpg:
        black = np.zeros((480, 640, 3), dtype=np.uint8)
        _, encoded = cv2.imencode(
            '.jpg', black, [cv2.IMWRITE_JPEG_QUALITY, 50]
        )
        jpg = encoded.tobytes()
    return Response(
        jpg,
        mimetype='image/jpeg',
        headers={
            'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
            'Pragma': 'no-cache',
        },
    )


@app.route('/api/telemetry')
def api_telemetry():
    return json.dumps(state.get_telemetry(), ensure_ascii=False), \
           200, {'Content-Type': 'application/json'}


# ──────────────────────────────────────────────
# 启动入口
# ──────────────────────────────────────────────

def start_server(host: str = "0.0.0.0", port: int = 8080):
    """在后台线程启动Flask服务器"""
    t = threading.Thread(target=lambda: app.run(
        host=host, port=port, threaded=True, debug=False
    ), daemon=True)
    t.start()
    print(f"[Preview] Web预览已启动 → http://{host}:{port}")
    return t


if __name__ == "__main__":
    print("预览服务器 (调试模式)")
    app.run(host="0.0.0.0", port=8080, debug=False)
