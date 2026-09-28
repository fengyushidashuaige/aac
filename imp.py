import os
p = os.path.expanduser('~/guttermap/gutter_track.py')
s = open(p, encoding='utf-8').read()
old = "import math, socket, json, csv, threading, time\nimport rospy\n"
new = "import math, socket, json, csv, threading, time\nimport numpy as np\nimport rospy\n"
if s.count(old) == 1 and 'import numpy' not in s:
    open(p, 'w', encoding='utf-8').write(s.replace(old, new))
    print('已加 import numpy as np')
else:
    print('匹配 %d 次 / 已有 numpy: %s' % (s.count(old), 'import numpy' in s))
