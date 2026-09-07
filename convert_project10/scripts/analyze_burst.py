# -*- coding: utf-8 -*-
"""验证 14B 点记录假设 + 突发块边界 + 子包头定位"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END)
blk1 = f.read(BLK)
f.close()

# ---- A. 找突发块：非零游程（记录内允许<=4B 零间隙）----
nz = (np.frombuffer(blk1, dtype=np.uint8) != 0)
# 平滑：4B 窗口内有非零就算数据
kernel = np.ones(4, dtype=bool)
smooth = np.convolve(nz, kernel, 'same') > 0
# 突发边界
d = np.diff(smooth.astype(np.int8))
starts = np.where(d == 1)[0] + 1
ends = np.where(d == -1)[0] + 1
if smooth[0]: starts = np.r_[0, starts]
if smooth[-1]: ends = np.r_[ends, len(smooth)]
sizes = ends - starts
gaps = starts[1:] - ends[:-1]
print(f"bursts: {len(starts)}, size med={np.median(sizes):.0f} max={sizes.max()} min={sizes.min()}")
print(f"gaps: med={np.median(gaps):.0f} max={gaps.max()}")
print(f"first 20 bursts (start_off, size): {list(zip(starts[:20].tolist(), sizes[:20].tolist()))}")
print(f"burst size histogram: {np.percentile(sizes, [10,25,50,75,90])}")
# 突发块起点对 1380 的余数分布
r1380 = (starts + HDR_END) % 1380
print(f"burst start % 1380 top values: {np.bincount(r1380.astype(int) if r1380.max()<1380 else r1380.astype(int)).argsort()[-8:][::-1]}")
print(f"  (need modulo care) uniq remainders sample: {r1380[:20]}")

# ---- B. 14B 记录全块统计：扫描对齐候选 ----
# 对每个偶数起始偏移 0..13，统计连续 14B 记录中 |i32|<35000 的占比
raw = blk1
best = []
for align in range(14):
    n = (len(raw) - align) // 14
    rec = np.frombuffer(raw[align:align + n*14], dtype='<i4').reshape(-1, 3)  # 先只看 i32 部分（错位时含 u16）
    # 严格：每 14B 取前 12B 为 3 个 i32
    ok = np.all(np.abs(rec.astype(np.int64)) < 35000, axis=1)
    best.append((align, ok.mean()))
for a, r in best:
    print(f"align {a:2d}: record-valid ratio = {r:.4f}")

# ---- C. 用最优对齐解出所有记录，距离分布 ----
align = max(best, key=lambda x: x[1])[0]
n = (len(raw) - align) // 14
v = np.frombuffer(raw[align:align+n*14], dtype='<i4').reshape(-1, 3)
u = np.frombuffer(raw[align+12:align+12+(n-1)*14+2:14] if False else np.zeros(1))  # 占位
u16s = np.frombuffer(raw, dtype='<u2')
# 直接按记录取 u16：偏移 align+12 + k*14
u = np.array([int.from_bytes(raw[align+12+k*14:align+14+k*14], 'little') for k in range(n)])
ok = np.all(np.abs(v.astype(np.int64)) < 35000, axis=1) & (u > 0) & (u < 4096)
vv = v[ok].astype(np.float64)
dist = np.sqrt((vv**2).sum(axis=1))
print(f"\nvalid records: {ok.sum()} / {n}")
print(f"dist percentiles [1,10,50,90,99]: {np.percentile(dist, [1,10,50,90,99]).round(1)}")
print(f"u16 attr distribution: {np.bincount(u[ok].astype(int), minlength=16)[:16]}")
# xyz 范围
print(f"x range: {vv[:,0].min():.0f}..{vv[:,0].max():.0f}  y: {vv[:,1].min():.0f}..{vv[:,1].max():.0f}  z: {vv[:,2].min():.0f}..{vv[:,2].max():.0f}")
