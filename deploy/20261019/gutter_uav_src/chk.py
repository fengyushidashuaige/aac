import sys, os
sys.path.insert(0, os.path.expanduser('~/gutter_uav'))
os.chdir(os.path.expanduser('~/gutter_uav'))
import numpy as np, cv2
from config import default_config
from config_jetson import apply_jetson_px4
from detector import TroughDetector
cfg = default_config; apply_jetson_px4(cfg)
det = TroughDetector(cfg)
files = sorted(os.listdir('/home/amov/logs/guttermap_20260922_103821/raw'))[::60]
print('%14s %7s %7s %9s %9s %9s %9s' % ('file','scale_x','scale_y','mask_x0','mask_x1','mask中位x','report off_px'))
for fn in files:
    f = '/home/amov/logs/guttermap_20260922_103821/raw/' + fn
    img = cv2.imread(f)
    if img is None: continue
    res = det.detect(img)
    m = getattr(det, '_last_seg_mask', None)
    if m is None or m.sum() == 0 or not res.detected: continue
    xs = np.where(m.any(axis=0))[0]
    # 中间行
    rows = np.where(m.any(axis=1))[0]
    yc = rows[len(rows)//2]
    cm = np.where(m[yc] > 0)[0]
    print('%14s %7.3f %7.3f %9d %9d %9.1f %9.1f' % (
        fn, det._mask_scale_x, det._mask_scale_y, xs.min(), xs.max(),
        0.5*(cm.min()+cm.max()) if cm.size else -1, res.offset_px))
