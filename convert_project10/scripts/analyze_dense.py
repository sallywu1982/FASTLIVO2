# -*- coding: utf-8 -*-
"""稠密区定位 + 记录对齐 + 子包边界探测"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END + BLK)
raw = f.read(BLK)
f.close()

# ---- A. 16B 粒度密度图（整块压缩视图）----
nz = (np.frombuffer(raw, dtype=np.uint8) != 0).astype(np.int32)
d16 = nz[:BLK//16*16].reshape(-1, 16).sum(axis=1)
dense = d16 >= 8
# 找最长稠密段
best = (0, 0)
s = None
for i, v in enumerate(dense):
    if v and s is None: s = i
    if (not v or i == len(dense)-1) and s is not None:
        e = i if not v else i+1
        if e - s > best[1]-best[0]: best = (s, e)
        s = None
ds, de = best[0]*16, best[1]*16
print(f"longest dense span: [{ds:#x},{de:#x}) len={de-ds}")
# 稠密段列表（>256B）
spans = []
s = None
for i, v in enumerate(dense):
    if v and s is None: s = i
    if (not v or i == len(dense)-1) and s is not None:
        e = i if not v else i+1
        if (e-s)*16 >= 256: spans.append((s*16, e*16))
        s = None
print(f"dense spans >=256B: {len(spans)}, total {sum(e-s for s,e in spans)}B")
print("first 10 spans:", [(hex(s), hex(e)) for s, e in spans[:10]])

# ---- B. 稠密段内 14 种对齐测试 ----
span = spans[0] if spans else (ds, de)
print(f"\nusing span {span:#x}-{span:#x}")
seg = raw[span[0]:span[1]]
n = len(seg)//14
for align in range(14):
    m = (len(seg)-align)//14
    v = np.frombuffer(seg[align:align+m*14], dtype='<i4').reshape(-1, 4)  # 每14B含4个i32槽?不——14B=3.5个i32
    # 改为逐字段验证：记录=i32,i32,i32,u16
    ok = 0
    vv = np.frombuffer(seg[align:align+m*14], dtype='<i4')
    for k in range(m):
        r = vv[k*3:k*3+3] if False else None
    # 简单法：reshape (m,3) 只覆盖 12B/记录，u16 单独
    v3 = np.frombuffer(seg[align:align+m*12], dtype='<i4').reshape(-1, 3) if False else None
    # 直接向量化：14B 记录的 i32 部分 = 以 14B 步长取 3 个 i32 —— 用字节重组
    idx = np.arange(m)
    base = align + idx*14
    b0 = np.frombuffer(seg[align:], dtype=np.uint8)
    off = idx*14
    x = (b0[off].astype(np.int64)) | (b0[off+1].astype(np.int64)<<8) | (b0[off+2].astype(np.int64)<<16) | (b0[off+3].astype(np.int64)<<24)
    y = (b0[off+4].astype(np.int64)) | (b0[off+5].astype(np.int64)<<8) | (b0[off+6].astype(np.int64)<<16) | (b0[off+7].astype(np.int64)<<24)
    z = (b0[off+8].astype(np.int64)) | (b0[off+9].astype(np.int64)<<8) | (b0[off+10].astype(np.int64)<<16) | (b0[off+11].astype(np.int64)<<24)
    # 符号扩展
    for arr in (x, y, z): arr[arr >= 2**31] -= 2**32
    u = b0[off+12].astype(np.int64) | (b0[off+13].astype(np.int64)<<8)
    okm = (np.abs(x)<35000)&(np.abs(y)<35000)&(np.abs(z)<35000)&(u>0)&(u<4096)
    print(f"  align {align:2d}: valid={okm.mean():.4f} (n={m})")

# ---- C. 用最优对齐走整个稠密段，找断裂 ----
best_align = None; best_rate = 0
for align in range(14):
    m = (len(seg)-align)//14
    b0 = np.frombuffer(seg[align:], dtype=np.uint8)
    off = np.arange(m)*14
    x = b0[off].astype(np.int64) | (b0[off+1].astype(np.int64)<<8) | (b0[off+2].astype(np.int64)<<16) | (b0[off+3].astype(np.int64)<<24)
    x[x>=2**31]-=2**32
    y = b0[off+4].astype(np.int64) | (b0[off+5].astype(np.int64)<<8) | (b0[off+6].astype(np.int64)<<16) | (b0[off+7].astype(np.int64)<<24)
    y[y>=2**31]-=2**32
    z = b0[off+8].astype(np.int64) | (b0[off+9].astype(np.int64)<<8) | (b0[off+10].astype(np.int64)<<16) | (b0[off+11].astype(np.int64)<<24)
    z[z>=2**31]-=2**32
    u = b0[off+12] | (b0[off+13]<<8)
    okm = (np.abs(x)<35000)&(np.abs(y)<35000)&(np.abs(z)<35000)&(u>0)&(u<4096)
    if okm.mean() > best_rate: best_rate, best_align = okm.mean(), align

print(f"\nbest align={best_align} rate={best_rate:.4f}")
align = best_align
m = (len(seg)-align)//14
b0 = np.frombuffer(seg[align:], dtype=np.uint8)
off = np.arange(m)*14
def i32at(o):
    v = int.from_bytes(seg[align+o:align+o+4], 'little', signed=True)
    return v
ok = []
for k in range(m):
    x = i32at(k*14); y = i32at(k*14+4); z = i32at(k*14+8)
    u = int.from_bytes(seg[align+k*14+12:align+k*14+14], 'little')
    ok.append(abs(x)<35000 and abs(y)<35000 and abs(z)<35000 and 0<u<4096)
ok = np.array(ok)
bad = np.where(~ok)[0]
print(f"bad records: {len(bad)} / {m}")
# 连续坏块 = 边界候选
if len(bad):
    splits = np.split(bad, np.where(np.diff(bad)>1)[0]+1)
    print(f"bad clusters: {len(splits)}")
    for c in splits[:6]:
        k0, k1 = c[0], c[-1]
        print(f"  records {k0}..{k1} at span+{align+k0*14:#x}..{align+k1*14:#x}")
        lo = max(0, align+k0*14-10)
        chunk = raw[span[0]+lo:span[0]+lo+ (c.len if False else min(80, len(raw)-span[0]-lo))]
        h = ' '.join(f'{b:02x}' for b in chunk)
        print(f"    ctx: {h}")

# ---- D. 记录值语义统计（最优对齐）----
xs = np.array([i32at(k*14) for k in range(0, m, max(1, m//5000))])
ys = np.array([i32at(k*14+4) for k in range(0, m, max(1, m//5000))])
zs = np.array([i32at(k*14+8) for k in range(0, m, max(1, m//5000))])
us = np.array([int.from_bytes(seg[align+k*14+12:align+k*14+14],'little') for k in range(0, m, max(1, m//5000))])
d = np.sqrt(xs.astype(float)**2+ys**2+zs**2)
print(f"\nsampled records: n={len(xs)}")
print(f"dist pct[1,10,50,90,99]: {np.percentile(d,[1,10,50,90,99]).round(2)}")
print(f"u16 attr uniq(top): {np.unique(us, return_counts=True)}")
