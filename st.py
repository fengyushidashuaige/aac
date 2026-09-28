import os
p = os.path.expanduser('~/guttermap/servo_tune.sh')
s = open(p, encoding='utf-8').read()
s = s.replace("KA=${5:-}\n", "KA=${5:-}\nKAARG=\"\"\n[ -n \"$KA\" ] && KAARG=\"_kp_ang:=$KA\"\n")
s = s.replace("setsid nohup python2 -u $B/servo_track.py $A _yaw_sign:=$YS _vy_sign:=$VS \\",
              "setsid nohup python2 -u $B/servo_track.py $A _yaw_sign:=$YS _vy_sign:=$VS $KAARG \\")
s = s.replace("# 用法: bash ~/guttermap/servo_tune.sh <yaw|center|forward> [use_psi 0/1] [yaw_sign] [vy_sign]",
              "# 用法: bash ~/guttermap/servo_tune.sh <yaw|center|forward> [use_psi 0/1] [yaw_sign] [vy_sign] [kp_ang]")
s = s.replace("#     bash servo_tune.sh yaw 0 1 -1   # 关掉 psi, 用原始 ang",
              "#     bash servo_tune.sh yaw 0 1 -1      # 关掉 psi, 用原始 ang\n"
              "#     bash servo_tune.sh center 0 0 -1   # 关掉航向环(kp_ang=0), 只控横向")
open(p, 'w', encoding='utf-8').write(s)
print('ok')
