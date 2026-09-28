#!/bin/bash
# ============================================================================
#  !! DEPRECATED / 别用 !!  (2026-09-22 加)
#  这是【换镜头之前】的手动测光方案: 它会把
#      exposure_auto=1(手动曝光) / brightness=-64 / gamma=64 / WB=2800
#  写死, 再往上爬 gain. 现役镜头【带 IR-CUT】, 这套值会把画面压糊压灰,
#  而且手动曝光远不如自动曝光稳(光照一变就废)。
#  正确做法: 什么都别做 —— gutter_udp.py 上电时的 cam_init() 会自动应用
#      exposure_auto=3 / gamma=100 / brightness=0 / gain=60 / WB自动
#  即: 想恢复就是 bash ~/guttermap/start_detect.sh
#  实测: 跑过本脚本后画面亮度/对比确实变差(2026-09-22)。
# ============================================================================
# 自动测光 (v2): 从"最暗"开始往上加, 直到亮度合适。任何光照下跑一次即可。
# 用法: bash ~/guttermap/cam_tune.sh
python3 - <<'PY'
import subprocess, time, urllib.request, sys
import numpy as np, cv2
D='/dev/video0'; TARGET=105.0
def setc(k,v): subprocess.call(['v4l2-ctl','-d',D,'-c','%s=%s'%(k,v)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
def stat():
    for _ in range(5):
        try:
            b=urllib.request.urlopen('http://127.0.0.1:8080/frame_raw.jpg',timeout=3).read()
            im=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_COLOR)
            if im is not None:
                g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY).astype(np.float32)
                return float(g.mean()), float(g.std())
        except Exception: pass
        time.sleep(0.4)
    return None, None

print('固定物理最优: exposure_auto=1(可设曝光)  WB=2800  gamma=64(高对比)  brightness=-64')
setc('exposure_auto',1); setc('white_balance_temperature_auto',0)
setc('white_balance_temperature',2800)
setc('brightness',-64); setc('gamma',64)
setc('exposure_absolute',1); setc('gain',0)
time.sleep(1.2)
m,s = stat()
if m is None: print('!! 抓帧失败, 检查检测进程是否在跑'); sys.exit(1)
print('  最暗档: 亮度 %.1f 对比 %.1f' % (m,s))

# 从最暗往上提: 先 gain, 不够再 exposure
if m < TARGET-15:
    print('  偏暗, 提增益...')
    best=(abs(m-TARGET), 0, m)
    for g in (3,8,20,40,80,150,300,600,1023):
        setc('gain',g); time.sleep(1.0)
        m,s = stat()
        if m is None: continue
        d=abs(m-TARGET)
        print('    gain=%-5d 亮度 %6.1f' % (g,m))
        if d<best[0]: best=(d,g,m)
        if m>TARGET+25: break
    setc('gain',best[1]); time.sleep(0.8)
    print('  选 gain=%d (亮度 %.1f)' % (best[1],best[2]))
    m,s = stat()
    if m is not None and m < TARGET-25:
        print('  还是暗, 提曝光...')
        b2=(abs(m-TARGET),1,m)
        for e in (3,10,30,100,300,1000,3000):
            setc('exposure_absolute',e); time.sleep(1.0)
            m,s=stat()
            if m is None: continue
            d=abs(m-TARGET)
            print('    exp=%-5d 亮度 %6.1f' % (e,m))
            if d<b2[0]: b2=(d,e,m)
            if m>TARGET+25: break
        setc('exposure_absolute',b2[1]); time.sleep(0.8)
        print('  选 exp=%d (亮度 %.1f)' % (b2[1],b2[2]))
else:
    print('  亮度已够 (或过亮, 已是最暗档)')

m,s = stat()
print()
if m is not None:
    verdict = 'OK' if 70<=m<=150 else ('偏暗' if m<70 else '偏亮')
    print('  ===> 最终: 亮度 %.1f  对比 %.1f   [%s]' % (m,s,verdict))
    if m>150:
        print('   (已经是最暗档了, 画面偏亮 — 可能是直射强光, 换个角度试试)')
PY
echo; v4l2-ctl -d /dev/video0 --list-ctrls 2>/dev/null | grep -E "brightness|gamma|gain |exposure_auto|exposure_absolute|white_balance_temperature " | sed 's/^/  /'
