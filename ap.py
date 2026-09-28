import os
p = os.path.expanduser('~/guttermap/auto_path.sh')
s = open(p, encoding='utf-8').read()
old = """  cp "$SDIR/polyline.csv" "$B/path_${FLIGHT}.csv\""""
new = """  # ★ 路径清洗: 去掉"掉头折返"那段(会被纯追踪照飞 -> 把飞机带出槽)
  python3 $B/clean_path.py "$SDIR/polyline.csv" "$B/path_${FLIGHT}.csv" 15 9
  cp -f "$SDIR/polyline.csv" "$B/polyline_${FLIGHT}_raw.csv\""""
if s.count(old) == 1:
    s = s.replace(old, new)
    open(p, 'w', encoding='utf-8').write(s)
    print('已接入清洗步骤')
else:
    print('匹配 %d 次, 未改' % s.count(old))
