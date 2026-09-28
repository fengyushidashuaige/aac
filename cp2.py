import os
p = os.path.expanduser('~/guttermap/clean_path.py')
s = open(p, encoding='utf-8').read()
old = """X = W[:, 0].copy()"""
new = """# ③ 先处理"折返": 沿主方向投影, 一旦明显倒退就在那里截断
c0 = W[:, :2].mean(axis=0)
_u, _sv, _vt = np.linalg.svd(W[:, :2] - c0, full_matrices=False)
_d = _vt[0]
_t = (W[:, :2] - c0).dot(_d)
_rmax = np.maximum.accumulate(_t)
_bad = _t < _rmax - 0.05            # 倒退超过 50mm 就认为开始折返
if _bad.any():
    k = int(np.argmax(_bad))
    print('  ★ 检测到折返: 在第 %d/%d 点(弧长约 %.2fm)处截断, 后面 %.2fm 丢弃' % (
        k, n, np.sum(np.linalg.norm(np.diff(W[:k+1, :2], axis=0), axis=1)),
        np.sum(np.linalg.norm(np.diff(W[k:, :2], axis=0), axis=1))))
    W = W[:k + 1].copy()
    P0 = W[:, :2].copy()
    n = len(W)
else:
    print('  (没有检测到折返)')

X = W[:, 0].copy()"""
if s.count(old) == 1:
    s = s.replace(old, new)
    open(p, 'w', encoding='utf-8').write(s)
    print('ok')
else:
    print('匹配 %d 次' % s.count(old))
