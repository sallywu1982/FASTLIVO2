# -*- coding: utf-8 -*-
"""找子包周期：1380B 步长测试 + 非零掩码自相关 + 时间戳上下文"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END)
blk1 = f.read(BLK); blk2 = f.read(BLK)
f.close()

def hx(buf, base, off, n=32):
    c = buf[off:off+n]
    h = ' '.join(f'{b:02x}' for b in c)
    print(f'{base+off:08x}  {h}')

# ---- A. 1380B 步长包头 ----
print("=== 1380B stride headers in blk1 ===")
for k in range(6):
    hx(blk1, HDR_END+BLK, k*1380, 32)
print("=== 1380B stride headers in blk2 ===")
for k in range(3):
    hx(blk2, HDR_END+2*BLK, k*1380, 32)

# ---- B. 非零掩码自相关（byte 粒度太大，用 4B 粒度）----
mask = (np.frombuffer(blk1, dtype=np.uint8) != 0).astype(np.float32)
m4 = mask[:len(mask)//4*4].reshape(-1, 4).mean(axis=1)  # 每 4B 密度
m4 = m4 - m4.mean()
ac = np.correlate(m4[:69000], m4[:69000], 'full')[69000-1:]
# 找前 5 个峰
peaks = []
for lag in range(4, min(len(ac), 69000)):
    if ac[lag] > ac[lag-1] and ac[lag] > ac[lag+1] and ac[lag] > 0.3*ac.max():
        peaks.append((lag, ac[lag]))
peaks.sort(key=lambda x: -x[1])
print("=== autocorr peaks (4B granularity) ===")
for lag, v in peaks[:12]:
    print(f"  lag={lag}x4={lag*4}B  score={v:.1f}")

# ---- C. blk1 的 u32 ms 时间戳上下文 ----
print("=== u32 ms stamp contexts in blk1 ===")
u32 = np.frombuffer(blk1, dtype='<u4')
idx = np.where((u32 >= 176540000) & (u32 <= 176900000))[0]
for i in idx:
    off = i*4
    v = u32[i]
    print(f"  off-in-blk=0x{off:x} val={v} ({v/1000:.3f}s)  prev u32={u32[i-1] if i>0 else '?'} next u32={u32[i+1] if i+1<len(u32) else '?'}")
    hx(blk1, HDR_END+BLK, max(0, off-16), 48)
