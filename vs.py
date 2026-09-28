import os
p = os.path.expanduser('~/guttermap/servo_track.py')
s = open(p, encoding='utf-8').read()
old = "'~vy_sign', +1.0"
new = "'~vy_sign', -1.0"
print('匹配 %d 次' % s.count(old))
if s.count(old) == 1:
    open(p, 'w', encoding='utf-8').write(s.replace(old, new))
    print('已改: vy_sign 默认 -1 (FLU 下 linear.y>0=左移, off>0=槽在右 -> 要往右 => 取负)')
