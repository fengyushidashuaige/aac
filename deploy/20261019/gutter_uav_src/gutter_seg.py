#!/usr/bin/env python3
"""YOLO-seg / detect gutter → mask + centerline (works for tilted troughs)."""

from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


def _oriented_long_short(pts: np.ndarray):
    """Return p0,p1 along long axis, unit short-axis su, short, long, angle_vs_vertical_deg."""
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
    short, long = sorted([float(rw), float(rh)])
    box = cv2.boxPoints(((cx, cy), (rw, rh), ang))
    d01 = box[1] - box[0]
    d12 = box[2] - box[1]
    if float(np.linalg.norm(d01)) >= float(np.linalg.norm(d12)):
        p0, p1 = box[0], box[1]
        sd = d12
    else:
        p0, p1 = box[1], box[2]
        sd = d01
    sn = float(np.linalg.norm(sd))
    if sn < 1e-3:
        return None
    su = sd / sn
    dx = float(p1[0] - p0[0])
    dy = float(p1[1] - p0[1])
    # angle of long axis vs image vertical (0 = upright)
    ang_v = float(np.degrees(np.arctan2(dx, max(abs(dy), 1e-3))))
    if dy < 0:
        ang_v = -ang_v
    return p0, p1, su, short, long, ang_v, (cx, cy)


def extend_mask_along_trough(
    frame_bgr: np.ndarray,
    mask: np.ndarray,
    max_extend_frac: float = 0.55,
) -> np.ndarray:
    """
    Grow incomplete tip along trough long-axis into white/low-sat pixels.
    Helps when labels/model only covered the near half of a diagonal trough.
    """
    m = (mask > 0).astype(np.uint8)
    ys, xs = np.where(m > 0)
    if xs.size < 80:
        return mask
    h, w = frame_bgr.shape[:2]
    pts = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
    orient = _oriented_long_short(pts)
    if orient is None:
        return mask
    p0, p1, su, short, long, _ang, (cx, cy) = orient
    if long < 40:
        return mask

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    white = (
        ((hsv[:, :, 1] <= 55) & (hsv[:, :, 2] >= 140))
        | ((gray >= 190) & (hsv[:, :, 1] <= 40))
    ).astype(np.uint8)

    # Direction from mask centroid toward each long end; extend the end that
    # still has white ahead but little mask.
    lu = (p1 - p0).astype(np.float64)
    ln = float(np.linalg.norm(lu))
    if ln < 1e-3:
        return mask
    lu = lu / ln
    half_w = max(8.0, min(0.55 * short, 55.0))
    out = m.copy()
    extend = int(long * max_extend_frac)
    for sign in (-1.0, 1.0):
        # start from the corresponding end of current mask projection
        # project mask points onto long axis
        rel = pts - np.array([cx, cy], dtype=np.float32)
        proj = rel[:, 0] * lu[0] + rel[:, 1] * lu[1]
        end_t = float(proj.max() if sign > 0 else proj.min())
        tip = np.array([cx, cy], dtype=np.float64) + lu * end_t
        added = 0
        miss = 0
        for step in range(1, extend + 1):
            c = tip + lu * (sign * step)
            # count white vs mask in a short-axis strip
            white_hits = 0
            mask_hits = 0
            samples = 0
            for s in np.linspace(-half_w, half_w, max(9, int(half_w))):
                x = int(round(c[0] + su[0] * s))
                y = int(round(c[1] + su[1] * s))
                if not (0 <= x < w and 0 <= y < h):
                    continue
                samples += 1
                if white[y, x]:
                    white_hits += 1
                if out[y, x]:
                    mask_hits += 1
            if samples < 5:
                break
            white_frac = white_hits / samples
            if white_frac < 0.22:
                miss += 1
                if miss >= 8:
                    break
                continue
            miss = 0
            # paint a short-axis disk of white pixels into mask
            for s in np.linspace(-half_w, half_w, max(11, int(half_w * 2))):
                x = int(round(c[0] + su[0] * s))
                y = int(round(c[1] + su[1] * s))
                if 0 <= x < w and 0 <= y < h and white[y, x]:
                    out[y, x] = 1
                    added += 1
        # only keep extension if meaningful
        _ = added
    out = cv2.morphologyEx(out * 255, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), 1)
    return out


def mask_to_centerline(
    mask: np.ndarray,
    min_area: float = 800.0,
    min_row_fill: int = 6,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]]:
    """
    Extract trough centerline from a segmentation mask.

    Fast path: work on modest downscale. Tilted troughs sample along the long
    axis; if that fails, fall back to row-wise mids (keeps lock at awkward yaw).
    """
    if mask is None or mask.size == 0:
        return None
    # 后处理已 CLOSE，这里不再重复 morph（省数 ms）
    m = mask if mask.dtype == np.uint8 else (mask > 0).astype(np.uint8)
    nz = int(cv2.countNonZero(m))
    if nz < min_area:
        return None

    # 先裁 bbox，避免全图 np.where
    x, y, bw, bh = cv2.boundingRect(m)
    if bw < 8 or bh < 20:
        return None
    pad = 2
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(m.shape[1], x + bw + pad)
    y1 = min(m.shape[0], y + bh + pad)
    mc_full = m[y0:y1, x0:x1]
    hf, wf = mc_full.shape[:2]

    # 过肥 mask（低阈值胀开）先轻微腐蚀，避免定向扫描宽全超限
    fill = float(nz) / float(max(bw * bh, 1))
    if fill > 0.72:
        mc_full = cv2.erode(mc_full, np.ones((3, 3), np.uint8), iterations=1)
        if int(cv2.countNonZero(mc_full)) < min_area * 0.5:
            mc_full = m[y0:y1, x0:x1]

    # 半分辨率加速；输入已是半图时不再二次压太狠
    scale = 1.0
    max_side = max(hf, wf)
    target = 160 if max_side > 200 else 128
    if max_side > target:
        scale = float(target) / float(max_side)
        nw = max(8, int(round(wf * scale)))
        nh = max(8, int(round(hf * scale)))
        mc = cv2.resize(mc_full, (nw, nh), interpolation=cv2.INTER_NEAREST)
        scale = nw / float(wf)
        min_area_s = min_area * scale * scale
        min_row_fill_s = max(3, int(round(min_row_fill * scale)))
    else:
        mc = mc_full
        min_area_s = min_area
        min_row_fill_s = min_row_fill
    hc, wc = mc.shape[:2]
    inv = 1.0 / scale

    # 用最大轮廓做方向，比全像素点集快很多
    cnts = cv2.findContours(mc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1] if len(cnts) == 3 else cnts[0]
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cnt) < min_area_s * 0.35:
        return None
    pts = cnt.reshape(-1, 2).astype(np.float32)
    if pts.shape[0] < 20:
        ys_all, xs_all = np.where(mc > 0)
        if xs_all.size < 40:
            return None
        pts = np.column_stack(
            [xs_all.astype(np.float32), ys_all.astype(np.float32)]
        )

    span_x = float(pts[:, 0].max() - pts[:, 0].min() + 1)
    span_y = float(pts[:, 1].max() - pts[:, 1].min() + 1)
    orient = _oriented_long_short(pts)
    if orient is None:
        return None
    p0, p1, su, short, long, ang_v, _cxy = orient
    if long < 12 or short < 2:
        return None

    # 倾角>5° 优先定向采样
    use_oriented = abs(ang_v) > 5.0 or span_x > span_y * 1.1 or short < span_x * 0.4

    def _sample_oriented():
        out_l, out_r, out_y, out_w = [], [], [], []
        pp0, pp1 = p0, p1
        if float(pp1[1]) < float(pp0[1]):
            pp0, pp1 = pp1, pp0
        half_short = 0.5 * short
        n = max(8, min(14, int(long / max(12.0, 1.0))))
        prev_mid = None
        # 倾角肥 mask 时 short 会被估大；宽上限放宽，否则全被滤掉 → 丢检
        true_w_cap = min(max(short * 1.8, 36.0), 110.0)
        span = min(half_short * 1.35 + 6.0, 70.0)
        step_s = max(1.2, span / 10.0)
        s_grid = np.arange(-span, span + 1e-6, step_s, dtype=np.float64)
        su0, su1 = float(su[0]), float(su[1])
        for i in range(n):
            t = i / max(n - 1, 1)
            mx = float(pp0[0] * (1 - t) + pp1[0] * t)
            my = float(pp0[1] * (1 - t) + pp1[1] * t)
            xs = np.rint(mx + su0 * s_grid).astype(np.int32)
            ys_i = np.rint(my + su1 * s_grid).astype(np.int32)
            valid = (xs >= 0) & (xs < wc) & (ys_i >= 0) & (ys_i < hc)
            if not np.any(valid):
                continue
            on = np.zeros(s_grid.shape[0], dtype=bool)
            on[valid] = mc[ys_i[valid], xs[valid]] > 0
            if int(on.sum()) < 2:
                continue
            hit_s = s_grid[on]
            s0, s1 = float(hit_s.min()), float(hit_s.max())
            ww = s1 - s0
            if ww < 2.0 or ww > true_w_cap:
                continue
            mid_x = mx + su0 * 0.5 * (s0 + s1)
            mid_y = my + su1 * 0.5 * (s0 + s1)
            if prev_mid is not None and abs(mid_x - prev_mid) > max(22.0, short * 1.35):
                continue
            hw = 0.5 * ww
            x_a = mid_x + su0 * (-hw)
            x_b = mid_x + su0 * hw
            out_l.append(float(min(x_a, x_b)) * inv + x0)
            out_r.append(float(max(x_a, x_b)) * inv + x0)
            out_y.append(float(mid_y) * inv + y0)
            out_w.append(float(ww) * inv)
            prev_mid = mid_x
        return out_l, out_r, out_y, out_w

    def _sample_rows():
        out_l, out_r, out_y, out_w = [], [], [], []
        prev_mid = None
        expect_w = float(np.clip(short, 6.0, 55.0))
        row_step = 2 if hc > 80 else 1
        for yy in range(0, hc, row_step):
            cols = np.flatnonzero(mc[yy])
            if cols.size < min_row_fill_s:
                continue
            breaks = np.where(np.diff(cols) > 1)[0] + 1
            starts = np.concatenate(([0], breaks))
            ends = np.concatenate((breaks, [cols.size]))
            best = None
            best_score = -1e18
            for s, e in zip(starts, ends):
                run = cols[s:e]
                if run.size < min_row_fill_s:
                    continue
                lx, rx = int(run[0]), int(run[-1])
                ww = rx - lx + 1
                if ww > max(expect_w * 3.0, 70.0):
                    continue
                mid = 0.5 * (lx + rx)
                score = float(run.size) - abs(ww - expect_w) * 0.6
                if prev_mid is not None:
                    score -= abs(mid - prev_mid) * 2.0
                if score > best_score:
                    best_score = score
                    best = (lx, rx, ww, mid)
            if best is None:
                continue
            lx, rx, ww, mid = best
            if prev_mid is not None and ww < expect_w * 0.25:
                continue
            out_l.append(float(lx) * inv + x0)
            out_r.append(float(rx) * inv + x0)
            out_y.append(float(yy) * inv + y0)
            out_w.append(float(ww) * inv)
            prev_mid = mid
            expect_w = 0.85 * expect_w + 0.15 * float(np.clip(ww, 5.0, 60.0))
        return out_l, out_r, out_y, out_w

    lefts, rights, ys, widths = [], [], [], []
    if use_oriented:
        lefts, rights, ys, widths = _sample_oriented()
    # 定向失败或点数不够：隔行回退，避免这个歪角直接 LOST
    if len(ys) < 6:
        lefts, rights, ys, widths = _sample_rows()
    if len(ys) < 6 and use_oriented:
        # 再试一次定向（有时行扫描也不够）
        lefts, rights, ys, widths = _sample_oriented()

    if len(ys) < 6:
        return None

    lefts = np.asarray(lefts, dtype=np.float64)
    rights = np.asarray(rights, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    widths = np.asarray(widths, dtype=np.float64)
    mids = 0.5 * (lefts + rights)
    order = np.argsort(ys)
    ys, mids, lefts, rights, widths = (
        ys[order],
        mids[order],
        lefts[order],
        rights[order],
        widths[order],
    )

    med_w = float(np.median(widths))
    keep = (widths >= med_w * 0.35) & (widths <= med_w * 2.2)
    if int(keep.sum()) >= 6:
        ys, mids, lefts, rights, widths = (
            ys[keep],
            mids[keep],
            lefts[keep],
            rights[keep],
            widths[keep],
        )

    if len(ys) < 6:
        return None

    # 仅平滑中心点；左右端按原宽度一半沿图像 X 重建（detector 会按角度重画）
    if len(mids) >= 5:
        k = 3
        padk = k // 2
        ext = np.pad(mids, (padk, padk), mode="edge")
        mids = np.convolve(ext, np.ones(k) / k, mode="valid")
        half = 0.5 * widths
        lefts = mids - half
        rights = mids + half

    if len(ys) >= 6:
        lo = int(len(ys) * 0.15)
        hi = int(len(ys) * 0.85)
        ys_f, mids_f = ys[lo:hi], mids[lo:hi]
        if len(ys_f) >= 5 and float(np.std(ys_f)) > 2.0:
            coef = np.polyfit(ys_f, mids_f, 1)
            pred = np.polyval(coef, ys_f)
            resid = np.abs(mids_f - pred)
            thr = max(6.0, float(np.median(resid)) * 3.0)
            inlier = resid <= thr
            if int(inlier.sum()) >= 5:
                coef = np.polyfit(ys_f[inlier], mids_f[inlier], 1)
            angle_deg = float(np.degrees(np.arctan(coef[0])))
        else:
            angle_deg = float(ang_v)
    else:
        angle_deg = float(ang_v)

    return lefts, rights, ys, widths, angle_deg


def tighten_box_to_trough(
    frame_bgr: np.ndarray,
    box,
    expect_w_lo: float = 30.0,
    expect_w_hi: float = 100.0,
    pad: int = 8,
):
    """Shrink a loose YOLO box to white/U trough (det fallback)."""
    h, w = frame_bgr.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in box[:4]]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w - 1, x2), min(h - 1, y2)
    if x2 - x1 < 12 or y2 - y1 < 12:
        return None

    roi = frame_bgr[y1 : y2 + 1, x1 : x2 + 1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    white = (
        ((hsv[:, :, 1] <= 60) & (hsv[:, :, 2] >= 130))
        | ((gray >= 185) & (hsv[:, :, 1] <= 45))
    ).astype(np.uint8) * 255
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
    dark = (gray < 55).astype(np.uint8) * 255
    bright_d = cv2.dilate((gray >= 180).astype(np.uint8) * 255, np.ones((5, 5), np.uint8), 1)
    trough = cv2.bitwise_or(white, cv2.bitwise_and(dark, bright_d))

    ys, xs = np.where(trough > 0)
    if xs.size < 80:
        return None
    pts = np.column_stack([xs, ys]).astype(np.float32)
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
    short, long = sorted([float(rw), float(rh)])
    if short < 6 or long < 24:
        return None
    target_short = float(np.clip(short, expect_w_lo * 0.45, expect_w_hi))
    if rw <= rh:
        rw2, rh2 = target_short, long
    else:
        rw2, rh2 = long, target_short
    box_pts = cv2.boxPoints(((cx, cy), (rw2, rh2), ang))
    lx = int(box_pts[:, 0].min())
    rx = int(box_pts[:, 0].max())
    ty = int(box_pts[:, 1].min())
    by = int(box_pts[:, 1].max())
    return (
        max(0, x1 + lx - pad),
        max(0, y1 + ty - pad),
        min(w - 1, x1 + rx + pad),
        min(h - 1, y1 + by + pad),
    )


class GutterSegInfer:
    """YOLO-seg: real instance mask → centerline (correct under camera tilt)."""

    def __init__(self, weights: str, conf: float = 0.25, imgsz: int = 640, device: str = ""):
        self.weights = str(weights)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.device = device
        self._model = None
        self._is_seg = None

    def _ensure(self):
        if self._model is not None:
            return
        if not Path(self.weights).exists():
            raise FileNotFoundError(f"weights not found: {self.weights}")
        from ultralytics import YOLO

        self._model = YOLO(self.weights)
        task = getattr(self._model, "task", None)
        self._is_seg = task == "segment" or "seg" in Path(self.weights).stem

    def predict_mask(self, frame_bgr: np.ndarray):
        """
        Returns (mask_uint8, conf, box_xyxy_or_None).
        Prefer real seg mask; fall back to filled det box.
        """
        self._ensure()
        kwargs = dict(conf=self.conf, imgsz=self.imgsz, verbose=False)
        if self.device:
            kwargs["device"] = self.device
        r0 = self._model.predict(frame_bgr, **kwargs)[0]
        ih, iw = frame_bgr.shape[:2]

        # --- Segmentation path ---
        if r0.masks is not None and len(r0.masks) > 0 and r0.boxes is not None and len(r0.boxes) > 0:
            masks = r0.masks.data.cpu().numpy()  # (N,H,W) model size
            confs = r0.boxes.conf.cpu().numpy()
            xyxy = r0.boxes.xyxy.cpu().numpy()
            best_i = None
            best_score = -1e9
            for i in range(len(masks)):
                m = masks[i]
                # resize to frame
                if m.shape[0] != ih or m.shape[1] != iw:
                    m_full = cv2.resize(m.astype(np.float32), (iw, ih), interpolation=cv2.INTER_LINEAR)
                else:
                    m_full = m.astype(np.float32)
                bin_m = (m_full > 0.35).astype(np.uint8)
                area = float(bin_m.sum())
                if area < 200:
                    continue
                ys, xs = np.where(bin_m > 0)
                if xs.size < 20:
                    continue
                span_x = float(xs.max() - xs.min() + 1)
                span_y = float(ys.max() - ys.min() + 1)
                elong = max(span_x, span_y) / max(min(span_x, span_y), 1.0)
                conf = float(confs[i])
                score = conf * (40.0 + min(elong, 12.0) * 12.0)
                if elong < 2.0:
                    score *= 0.5
                # prefer near center
                cx = 0.5 * (xs.min() + xs.max())
                cy = 0.5 * (ys.min() + ys.max())
                score -= 0.01 * abs(cx - iw * 0.5)
                score -= 0.005 * abs(cy - ih * 0.5)
                if score > best_score:
                    best_score = score
                    best_i = i
                    best_mask = (bin_m * 255).astype(np.uint8)
                    best_conf = conf
                    best_box = tuple(map(float, xyxy[i]))
            if best_i is not None:
                # light morph clean + slight dilate so tip isn't clipped
                best_mask = cv2.morphologyEx(
                    best_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1
                )
                best_mask = cv2.dilate(best_mask, np.ones((3, 3), np.uint8), iterations=1)
                # Tip extension only when mask is clearly truncated (short vs image)
                ys2, xs2 = np.where(best_mask > 0)
                if xs2.size >= 20:
                    span = max(float(xs2.max() - xs2.min()), float(ys2.max() - ys2.min()))
                    if span < 0.55 * max(ih, iw):
                        best_mask = extend_mask_along_trough(frame_bgr, best_mask)
                    ys2, xs2 = np.where(best_mask > 0)
                    best_box = (
                        float(xs2.min()),
                        float(ys2.min()),
                        float(xs2.max()),
                        float(ys2.max()),
                    )
                return best_mask, best_conf, best_box

        # --- Detection fallback: filled box (legacy) ---
        if r0.boxes is None or len(r0.boxes) == 0:
            return None, 0.0, None
        det = GutterDetInfer(self.weights, conf=self.conf, imgsz=self.imgsz, device=self.device)
        det._model = self._model
        box = det.predict_box(frame_bgr)
        if box is None:
            return None, 0.0, None
        x1, y1, x2, y2, conf = box
        mask = np.zeros((ih, iw), dtype=np.uint8)
        mask[max(0, int(y1)) : min(ih, int(y2)), max(0, int(x1)) : min(iw, int(x2))] = 255
        return mask, conf, (x1, y1, x2, y2)


class GutterDetInfer:
    """YOLO detection for gutter; returns tightened trough box."""

    def __init__(self, weights: str, conf: float = 0.25, imgsz: int = 640, device: str = ""):
        self.weights = str(weights)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.device = device
        self._model = None
        self.max_box_w_frac = 0.35
        self.expect_w_lo = 30.0
        self.expect_w_hi = 100.0

    def _ensure(self):
        if self._model is not None:
            return
        if not Path(self.weights).exists():
            raise FileNotFoundError(f"weights not found: {self.weights}")
        from ultralytics import YOLO

        self._model = YOLO(self.weights)

    def predict_box(self, frame_bgr: np.ndarray):
        self._ensure()
        kwargs = dict(conf=self.conf, imgsz=self.imgsz, verbose=False)
        if self.device:
            kwargs["device"] = self.device
        r0 = self._model.predict(frame_bgr, **kwargs)[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return None
        xyxy = r0.boxes.xyxy.cpu().numpy()
        confs = r0.boxes.conf.cpu().numpy()
        ih, iw = frame_bgr.shape[:2]
        max_w = iw * self.max_box_w_frac
        cands = []
        for i, (x1, y1, x2, y2) in enumerate(xyxy):
            bw = max(0.0, x2 - x1)
            bh = max(0.0, y2 - y1)
            if bw < 12 or bh < 12:
                continue
            conf = float(confs[i])
            elong = max(bw, bh) / max(min(bw, bh), 1.0)
            area_frac = (bw * bh) / max(float(iw * ih), 1.0)
            score = conf * (50.0 + min(elong, 10.0) * 10.0)
            if elong < 2.0:
                score *= 0.55
            if area_frac > 0.40 and elong < 3.0:
                score *= 0.7
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            score -= 0.015 * abs(cx - iw * 0.5)
            score -= 0.008 * abs(cy - ih * 0.5)
            cands.append((score, conf, float(x1), float(y1), float(x2), float(y2), bw, bh))
        if not cands:
            return None
        cands.sort(key=lambda t: t[0], reverse=True)
        for cand in cands[:5]:
            _s, conf, x1, y1, x2, y2, bw, bh = cand
            tight = tighten_box_to_trough(
                frame_bgr, (x1, y1, x2, y2),
                expect_w_lo=self.expect_w_lo, expect_w_hi=self.expect_w_hi, pad=6,
            )
            if tight is not None:
                tx1, ty1, tx2, ty2 = tight
                tw, th = tx2 - tx1, ty2 - ty1
                t_elong = max(tw, th) / max(min(tw, th), 1.0)
                if t_elong >= 1.8 and (tw * th) <= (bw * bh) * 1.05:
                    x1, y1, x2, y2 = float(tx1), float(ty1), float(tx2), float(ty2)
                    bw, bh = x2 - x1, y2 - y1
            elong = max(bw, bh) / max(min(bw, bh), 1.0)
            area_frac = (bw * bh) / max(float(iw * ih), 1.0)
            if area_frac > 0.55 and elong < 2.5 and conf < 0.5:
                continue
            if elong < 1.4 and conf < 0.35:
                continue
            return x1, y1, x2, y2, conf
        return None

    def predict_mask(self, frame_bgr: np.ndarray):
        box = self.predict_box(frame_bgr)
        if box is None:
            return None, 0.0, None
        x1, y1, x2, y2, conf = box
        h, w = frame_bgr.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[max(0, int(y1)) : min(h, int(y2)), max(0, int(x1)) : min(w, int(x2))] = 255
        return mask, conf, (x1, y1, x2, y2)
