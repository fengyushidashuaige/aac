import os
p = os.path.expanduser('~/guttermap/replay_trim.py')
s = open(p, encoding='utf-8').read()
old = "def sim(TAU, CAP, LIM, skip_bad=True):"
new = """# ★ 插值到控制频率 20Hz (修正环是按 20Hz 跑的, 不是按日志的 1Hz)
ti = np.arange(T[0], T[-1], 0.05)
cte = np.interp(ti, T, cte); off = np.interp(ti, T, off); T = ti
print('插值到 20Hz: %d 点' % len(T))


def sim(TAU, CAP, LIM, skip_bad=True):"""
if s.count(old) == 1:
    s = s.replace(old, new)
    open(p, 'w', encoding='utf-8').write(s); print('ok')
else: print('匹配 %d' % s.count(old))
