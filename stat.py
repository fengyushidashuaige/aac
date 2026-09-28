import os
p = os.path.expanduser('~/guttermap/hover_check.py')
s = open(p, encoding='utf-8').read()
i = s.index("print()\nprint('=== 悬停段")
s = s[:i] + '''print()
print('=== ★ 地面静止段 (飞机根本没动, 任何读数变化都是雷达估计漂移) ===')
g = Z < 0.10
# 取最长的静止段
best2 = (0, 0, 0); i = 0
while i < len(g):
    if g[i]:
        j = i
        while j < len(g) and g[j]:
            j += 1
        if T[j-1]-T[i] > best2[0]:
            best2 = (T[j-1]-T[i], i, j)
        i = j
    else:
        i += 1
d2, a0, a1 = best2
if d2 < 5:
    print('  地面段太短 (%.1fs)' % d2)
else:
    tt = T[a0:a1] - T[a0]
    xx = X[a0:a1]; yy = Y[a0:a1]; zz = Z[a0:a1]; ww = YW[a0:a1]
    print('  静止 %.1fs' % d2)
    for nm, v in (('x', xx), ('y', yy), ('z', zz), ('yaw', ww)):
        A = np.column_stack([tt, np.ones(len(v))])
        sl, _, _, _ = np.linalg.lstsq(A, v)
        res = v - A.dot(sl)
        sc = 1.0 if nm == 'yaw' else 1000.0
        print('  %-4s 漂移 %+8.2f /s (共 %+8.1f) | 抖动 %7.2f | 总变化 %+8.1f' % (
            nm, sl[0]*sc, sl[0]*sc*d2, res.std()*sc, (v[-1]-v[0])*sc))
    print()
    print('  => 地面静止时若 x/y/yaw 还在明显变化, 那就是【雷达估计在漂】✗')
    print('     把 vision_pose 喂给飞控 => 飞控以为自己在动 => 真的去修正 => 飞机真的动 ✗')

''' + s[i:]
open(p, 'w', encoding='utf-8').write(s)
print('ok')
