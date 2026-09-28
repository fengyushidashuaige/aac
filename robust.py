import os
p = os.path.expanduser('~/guttermap/gutter_track.py')
s = open(p, encoding='utf-8').read()
n = 0
# 状态: 用列表缓冲替代 EMA
old = "        self.wpx_lp = None\n"
new = ("        self.wpx_buf = []          # 近 200 帧 wpx (用85百分位当参考, EMA会被坏帧拖垮)\n")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
# udp_loop 里的门控+尺度
old2 = """                if ok and _w > 1.0:
                    self.wpx_lp = _w if self.wpx_lp is None else 0.98 * self.wpx_lp + 0.02 * _w
                    if not (0.80 * self.wpx_lp <= _w <= 1.25 * self.wpx_lp):
                        ok = False
                if ok:
                    self.cam_off = _off
                    self.cam_ang = _ang
                    self.cam_t = time.time()"""
new2 = """                if ok and _w > 1.0:
                    self.wpx_buf.append(_w)
                    if len(self.wpx_buf) > 200:
                        self.wpx_buf.pop(0)
                    if len(self.wpx_buf) >= 20:
                        # ★ 参考用 85 百分位: 槽被截断只会让 wpx 变小, 所以高分位不会被拖下去
                        #   (旧的 EMA 参考会被持续 15s 的坏帧拖到 26, 门控就失效了 ✗)
                        _ref = float(np.percentile(self.wpx_buf, 85))
                        if _w < 0.75 * _ref or _w > 1.6 * _ref:
                            ok = False
                        else:
                            # ★ 尺度用参考宽度, 不用逐帧 wpx -> 截断时不再把 off 放大
                            _off = _off * _w / _ref
                if ok:
                    self.cam_off = _off
                    self.cam_ang = _ang
                    self.cam_t = time.time()"""
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
if n == 2:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
