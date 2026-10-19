"""
PVC水槽中心线检测 - 全部可调参数
将所有参数集中在这里，方便现场调参
"""

from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class CameraConfig:
    """相机参数"""
    # 图像分辨率 (板子USB摄像头实际: 640x480)
    width: int = 640
    height: int = 480
    fps: int = 60

    # 相机安装角度
    tilt_angle_deg: float = 45.0   # 前倾向下角度
    mount_height_m: float = 1.0    # 飞行高度(米)

    # 相机视场角 (根据实际镜头填写)
    hfov_deg: float = 70.0         # 水平视场角
    vfov_deg: float = 43.0         # 垂直视场角 (根据16:9估算)

    # USB摄像头模式 (板子上是/dev/video23)
    gst_pipeline: str = ""
    camera_id: int = 23

    # 曝光: True=硬件自动曝光(随环境自适应); False=手动固定
    # UVC: exposure_auto=3 → Aperture Priority (连续自动)
    auto_exposure: bool = True
    # 软件亮度闭环：限制曝光时间避免飞行运动模糊，再用增益补亮度
    adaptive_exposure: bool = False
    target_luma: float = 112.0
    exposure_min: int = 40
    exposure_max: int = 260
    gain_min: int = 20
    gain_max: int = 180
    camera_adjust_interval_s: float = 0.8
    contrast: int = 20
    sharpness: int = 5
    backlight_compensation: int = 86
    power_line_frequency: int = 1       # 1=50Hz，减少室内灯闪烁
    auto_white_balance: bool = True
    # 仅 auto_exposure=False 时生效
    exposure_absolute: int = 156
    gain: int = 106
    # 对焦固定(跟槽距离相对固定，避免AF来回拉)
    focus_auto: bool = False
    focus_absolute: int = 350


@dataclass
class PreprocessConfig:
    """图像预处理参数"""
    # CLAHE (自适应直方图均衡化) — 应对光照不均
    clahe_clip_limit: float = 2.0
    clahe_tile_size: int = 8

    # 高斯模糊
    blur_kernel: int = 5
    blur_sigma: float = 1.0


@dataclass
class CannyConfig:
    """Canny边缘检测参数"""
    # auto_threshold=True 时，自动根据画面内容计算阈值(推荐)
    # auto_threshold=False 时，使用下面的固定阈值
    auto_threshold: bool = False   # 板子画面偏暗，先用手动阈值
    sigma: float = 0.33          # 自动阈值系数

    # 固定阈值 (auto_threshold=False时生效)
    low_threshold: int = 25
    high_threshold: int = 80

    # Sobel算子大小
    sobel_size: int = 3


@dataclass
class HoughConfig:
    """霍夫线检测参数"""
    rho: float = 1.0             # ρ分辨率(像素)
    theta: float = 1.0           # θ分辨率(度),会被转成弧度
    threshold: int = 30           # 最小投票数 (平衡噪声和检测)
    min_line_length: float = 30.0 # 最小线段长度(像素)
    max_line_gap: float = 40.0    # 允许的断线间隙(像素)


@dataclass
class TroughConfig:
    """水槽几何参数"""
    # PVC水槽实际宽度 (毫米)
    trough_width_mm: float = 110.0

    # 2026-09-22: seed for the rolling validity reference (px). Arms the gate
    # on frame 1, before the rolling window has any samples. 0 = disabled.
    ref_width_px: float = 0.0

    # 透视变换源点 — 定义地面ROI在原图中的位置
    # 相机45°前倾: 地面在画面下半部
    # 顺序: 左上(远处), 右上(远处), 右下(近处), 左下(近处)
    # 值: 相对于图像宽高的比例 (0-1)
    perspective_src: Tuple[Tuple[float, float], ...] = (
        (0.25, 0.45),   # 左上 — 远处地面左边界
        (0.75, 0.45),   # 右上 — 远处地面右边界
        (0.90, 0.95),   # 右下 — 近处地面右边界
        (0.10, 0.95),   # 左下 — 近处地面左边界
    )

    perspective_out_width: int = 400
    perspective_out_height: int = 600

    # 鸟瞰图下每像素对应实际毫米数 (标定后填入)
    # 计算方法: trough_width_mm / (鸟瞰图中水槽占的像素数)
    mm_per_pixel: float = 0.0    # 0 = 自动估算
    # 自动估算时用的比例 (水槽宽度占鸟瞰图宽度的比例)
    trough_width_ratio: float = 0.27  # 实测水槽占鸟瞰图宽度约27%


@dataclass
class ControlConfig:
    """飞控参数 — 慢速前进 + 偏航/横向软混控"""
    # PID参数 — 横向修正 (mm → m/s)
    lateral_kp: float = 0.12
    lateral_ki: float = 0.0
    lateral_kd: float = 0.02

    # PID参数 — 航向修正 (deg → rad/s)；关 D，避免噪声导致过冲
    yaw_kp: float = 0.045
    yaw_ki: float = 0.0
    yaw_kd: float = 0.0

    # 输出限幅：yaw 略收，避免单边打满后难回头
    max_lateral_speed: float = 0.10   # m/s
    max_yaw_rate: float = 0.09        # rad/s ≈5.2°/s
    yaw_slew_rate: float = 0.35       # 允许更快改向（配合角度重置）
    lateral_slew_rate: float = 0.35   # vy 指令最大变化率 (m/s²)
    lateral_slew_rate_down: float = 0.70  # 收油门/刹车更快，抑制过冲
    # |offset|<此值时横向按比例衰减，减轻冲过中线
    lateral_soft_zone_mm: float = 80.0

    # 软混控：大偏航时仍保留一点横移，对齐后仍保留一点偏航
    yaw_priority_deg: float = 10.0    # 超过此角，横移减弱
    vy_during_yaw_scale: float = 0.55 # 大偏航时 vy 保留比例
    yaw_during_lat_scale: float = 0.50  # 以横移为主时 yaw 保留比例

    # 控制死区（喷管对准用：横向死区略小于吹叶区）
    dead_zone_mm: float = 8.0
    dead_zone_deg: float = 3.0

    # 检测置信度接受门限:
    # 低于此值视为无效检测(垃圾帧/异物误检), 不参与控制
    # 实测依据: 真水槽 conf~0.87, 吹风筒误检 conf~0.35 (NEXT_STEPS.md 4.2)
    min_accept_conf: float = 0.6

    # 锁心后几乎不纠偏（锁方向），只有偏很大才微调
    # 注: 新 PD 控制下 cruise_corr_scale 替代这两个字段（巡航中保留小增益主动对中）
    cruise_vy_scale: float = 0.15
    cruise_yaw_scale: float = 0.0
    cruise_corr_scale: float = 0.35    # 巡航中横向/偏航保留增益 (0=开环)

    # GUIDED模式巡航 — 锁心后匀速前进
    forward_speed: float = 0.06       # 巡航前进 (m/s)
    max_forward_speed: float = 0.08   # 硬顶
    forward_slew_rate: float = 0.10   # vx 爬升/下降斜率
    # 未锁时：大致对中就给速度；锁上后恒定 vx
    forward_align_angle_deg: float = 14.0
    forward_align_offset_mm: float = 60.0
    forward_min_scale: float = 0.40
    # 巡航锁滞回
    cruise_lock_angle_deg: float = 8.0
    cruise_lock_offset_mm: float = 40.0
    cruise_unlock_angle_deg: float = 14.0
    cruise_unlock_offset_mm: float = 70.0
    cruise_lock_frames: int = 8       # ~0.5s @15FPS

    # ── 平滑 PD + 延迟预测（替代分段 bang-bang）──
    # vy_des = kp·(off + L·off_dot) + kd·off_dot, 单位: off=mm, off_dot=mm/s, vy=m/s
    offset_kp_s: float = 0.0008        # 100mm 偏 → 0.08 m/s (限幅前)
    offset_kd_s: float = 0.0003        # 接近速度阻尼, 防止冲过中心后回摆
    offset_pred_latency_s: float = 0.25  # 检测→执行链路延迟, 前馈补偿
    offset_deriv_tau_s: float = 0.35   # off_dot 滑窗时长 = 2.5×τ (≈0.9s 最小二乘斜率)
    max_ctrl_offset_mm: float = 500.0  # 控制用偏移限幅（超此值按此值纠，避免垃圾检测打满）
    # 纯偏航门限: |angle| 超过此值不横移（图像偏移在斜角下失真）
    yaw_only_gate_deg: float = 20.0
    vy_in_yaw_gate_scale: float = 0.25 # 纯偏航阶段 vy 保留比例
    yaw_kp_deg: float = 0.010          # 偏航速率 = -k·angle (rad/s 每度)
    cruise_lock_od_mm_s: float = 30.0  # 锁心要求: |off_dot| 低于此 (mm/s)
    cruise_unlock_od_mm_s: float = 55.0

    # RC override通道
    rc_pwm_center: int = 1500
    rc_pwm_range: int = 400

    # MAVLink — RK3588 默认 ttyS8；Jetson/PX4 用 config_jetson 覆盖
    use_mavlink: bool = True
    mavlink_connection: str = "/dev/ttyS8"
    mavlink_baud: int = 115200
    # "ardupilot" = GUIDED/LOITER；"px4" = OFFBOARD/AUTO.LOITER
    fc_stack: str = "ardupilot"

    # 飞行模式
    auto_guided: bool = False         # 首飞：手动切 GUIDED/OFFBOARD
    safety_loss_timeout: float = 1.0  # 丢线超时(秒) → LOITER/HOLD


@dataclass
class YoloConfig:
    """YOLO-seg检测参数"""
    enabled: bool = True                     # True=YOLO-seg, False=HSV传统
    # FP 能检出；INT8(gutter_seg_480.rknn) 现场分数≈0 不可用
    weights: str = "models/gutter_seg_320_fp.rknn"
    conf: float = 0.15
    imgsz: int = 320
    device: str = "npu"
    candidate_conf: float = 0.05
    fallback_conf: float = 0.02
    mask_threshold_low: float = 0.32
    mask_threshold_high: float = 0.40
    extend_mask: bool = False
    # 忽略画面底部比例（机头涵道喷管），只用水槽上半/远场
    mask_bottom_ignore: float = 0.0
    # 横向偏移取样行（相对图像高度）；底部忽略后宜取 0.35~0.45
    ctrl_y_frac: float = 0.72
    # 推理前高斯核(奇数)；>=3 减轻果冻高频扭曲对分割的影响
    infer_blur_ksize: int = 0


@dataclass
class DebugConfig:
    """调试参数"""
    show_preview: bool = True           # 显示调试画面
    save_video: bool = True             # 每次飞行保存叠加画面录像
    save_raw_video: bool = False        # 同时保存无叠加原图（训练用）
    video_output: str = ""              # 空=自动放到 logs/flight_*/overlay.avi
    record_every_n: int = 2             # 隔帧写录像，减轻 CPU（1=每帧）
    raw_record_every_n: int = 2         # 原图录像隔帧；训练采集可设 1
    preview_width: int = 480             # Web预览宽度；Jetson可覆盖为640
    preview_jpeg_quality: int = 52       # 预览JPEG质量
    preview_every_n: int = 2             # 每N帧更新Web预览
    print_offset: bool = True           # 现场看 CTRL/LOST，确认是否在往中线靠
    log_file: str = ""                  # 日志文件路径(空=自动)

    # 可视化颜色
    color_left_edge: Tuple[int, int, int] = (0, 255, 0)     # 绿色-左边界
    color_right_edge: Tuple[int, int, int] = (0, 0, 255)    # 红色-右边界
    color_center_line: Tuple[int, int, int] = (0, 255, 255) # 黄色-中心线
    color_all_lines: Tuple[int, int, int] = (255, 255, 255) # 白色-所有检测线


@dataclass
class Config:
    """总配置"""
    camera: CameraConfig = field(default_factory=CameraConfig)
    preprocess: PreprocessConfig = field(default_factory=PreprocessConfig)
    canny: CannyConfig = field(default_factory=CannyConfig)
    hough: HoughConfig = field(default_factory=HoughConfig)
    trough: TroughConfig = field(default_factory=TroughConfig)
    yolo: YoloConfig = field(default_factory=YoloConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)


# 创建一个全局默认配置实例，方便直接 import
default_config = Config()
