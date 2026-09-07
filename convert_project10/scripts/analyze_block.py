# -*- coding: utf-8 -*-
"""深入解析 276000B 帧块内部结构"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END)
blk0 = f.read(BLK)          # 第一块（首帧，但可能是未同步的预滚数据）
blk1 = f.read(BLK)          # 第二块
f.close()

# ---- A. 非零密度图（32B 粒度），看子包结构 ----
def density(buf, step=32, per=64):
    n = len(buf)//step
    d = np.zeros(n, dtype=int)
    for i in range(n):
        d[i] = np.count_nonzero(buf[i*step:(i+1)*step])
    # 压缩打印：每行 per 个块
    for r in range(0, min(n, per*20), per):
        row = d[r:r+per]
        print(' '.join(f'{v:2x}' for v in row))

print("=== blk0 nonzero/32B (first rows) ===")
density(blk0)

# ---- B. 找时间戳：u32 毫秒 / 0.1ms / u64 微秒 ----
def find_stamps(buf, name):
    print(f"--- {name} ---")
    u32 = np.frombuffer(buf, dtype='<u4')
    for scale, lo, hi, tag in [(1, 176540000, 176900000, 'ms'),
                               (10, 1765400000, 1769000000, '0.1ms'),
                               (1000, 176540000, 176900000, 'us-thousand?')]:
        idx = np.where((u32 >= lo) & (u32 <= hi))[0]
        if len(idx):
            print(f"  u32 {tag}: {len(idx)} hits, first at off 0x{idx[0]*4:x} val={u32[idx[0]]}")
    u64 = np.frombuffer(buf[:len(buf)//8*8], dtype='<u8')
    idx = np.where((u64 > 176540000000) & (u64 < 176900000000))[0]
    if len(idx):
        print(f"  u64 us: {len(idx)} hits, first 0x{idx[0]*8:x} val={u64[idx[0]]}")
    # double GPS 秒
    f64 = np.frombuffer(buf[:len(buf)//8*8], dtype='<f8')
    idx = np.where((f64 > 176500) & (f64 < 176900))[0]
    if len(idx):
        print(f"  f64 s: {len(idx)} hits, first 0x{idx[0]*8:x} val={f64[idx[0]]:.6f}")

find_stamps(blk0, "blk0")
find_stamps(blk1, "blk1")

# ---- C. u16=11928 上下文 ----
print("=== context around 0x98F (u16=11928) ===")
off = 0x98F - HDR_END - 64
for i in range(off, off+160, 16):
    c = blk0[i:i+16]
    hx = ' '.join(f'{b:02x}' for b in c)
    asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in c)
    print(f'{HDR_END+i:08x}  {hx:<48}  {asc}')

# ---- D. 16B 记录假设的值域统计（整块） ----
print("=== i32 x4 (16B stride) value ranges in blk0 ===")
n = len(blk0)//16*16
q = np.frombuffer(blk0[:n], dtype='<i4').reshape(-1, 4)
for col in range(4):
    c = q[:, col].astype(np.int64)
    print(f"  col{col}: min={c.min()} max={c.max()} |v|>1000: {(abs(c)>1000).sum()} |v|<100: {(abs(c)<100).sum()} zeros={(c==0).sum()}")

# ---- E. 14B 记录假设 ----
print("=== 14B stride check on first 5000B ===")
raw = np.frombuffer(blk0[:5000], dtype='<i4')
# 手工扫描：i32 三元组 + u16，|a|<2000 |b|<2000 |c|<200
cnt = 0
for off in range(0, 4000, 2):
    a, b, c = struct_i32 = np.frombuffer(blk0[off:off+12], dtype='<i4')
    d = np.frombuffer(blk0[off+12:off+14], dtype='<u2')[0]
    if abs(a) < 2000 and abs(b) < 2000 and abs(c) < 100 and 0 < d < 20:
        cnt += 1
print(f"  matches(off even, first 4000B): {cnt}")
