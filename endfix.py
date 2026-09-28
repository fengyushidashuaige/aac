import os
p = os.path.expanduser('~/guttermap/pursuit.py')
s = open(p, encoding='utf-8').read()
n = 0
old = "        if remain < 0.15:"
new = ("        # 终点判定提前: 0.15m 太晚, 会在末端猛打舵/横摆(实测)\n"
       "        if remain < 0.40:")
if s.count(old) == 1:
    s = s.replace(old, new); n += 1
old2 = "        L = min(self.lookahead, max(0.25, remain * 0.8))"
new2 = ("        # 前视距离加下限: 末端缩到 0.25m 时, 同一个横向误差会算出 4 倍大的偏航率 -> 左右摆\n"
        "        L = min(self.lookahead, max(0.50, remain * 0.8))")
if s.count(old2) == 1:
    s = s.replace(old2, new2); n += 1
if n:
    open(p, 'w', encoding='utf-8').write(s)
print('改了 %d 处' % n)
