# -*- coding: utf-8 -*-
"""分析 .lid 数据体结构：帧头同步字、点数、时间戳、帧步长"""
import struct, sys
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5  # CRC u2 @0x6B3 之后

f = open(LID, "rb")
f.seek(0, 2); fsize = f.tell()
f.seek(HDR_END)
data = f.read(4 * 1024 * 1024)  # 前 4MB 数据体
f.close()
print(f"file size={fsize}, data region start=0x{HDR_END:x}, read={len(data)}")

# ---- 1. 数据区开头 512B hexdump ----
def dump(buf, base, n=512):
    for i in range(0, n, 16):
        c = buf[i:i+16]
        hx = ' '.join(f'{b:02x}' for b in c)
        asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in c)
        print(f'{base+i:08x}  {hx:<48}  {asc}')
dump(data, HDR_END, 384)

# ---- 2. 在数据区前 64KB 找可能的 u16/u32 点计数字段 ----
print("\n--- u32 in [11000,13000] (前 4096B) ---")
n32 = np.frombuffer(data[:4096], dtype='<u4')
for i, v in enumerate(n32):
    if 11000 <= v <= 13000:
        print(f"  off 0x{HDR_END + i*4:x}: u32={v}")
print("--- u16 in [11500,13000] (前 2048B) ---")
n16 = np.frombuffer(data[:2048], dtype='<u2')
for i, v in enumerate(n16):
    if 11500 <= v <= 13000:
        print(f"  off 0x{HDR_END + i*2:x}: u16={v}")

# ---- 3. 找 float64 GPS 周内秒时间戳 176551~176845 ----
print("\n--- float64 GPS SOW stamps (前 1MB) ---")
d8 = np.frombuffer(data[:1024*1024], dtype='<f8')
idx = np.where((d8 > 176500) & (d8 < 176900))[0]
for i in idx[:20]:
    print(f"  off 0x{HDR_END + i*8:x}: f64={d8[i]:.6f}")
print(f"  total {len(idx)} hits")

# ---- 4. 同步字假设：取数据区开头 8 字节为模式，在前 3MB 中搜索出现位置 ----
pat = bytes(data[:8])
print(f"\n--- pattern {pat.hex()} occurrences (前 3MB) ---")
occ = []
start = 0
while True:
    j = data.find(pat, start)
    if j < 0 or j > 3*1024*1024: break
    occ.append(j); start = j + 1
print(f"  count={len(occ)}")
if len(occ) >= 5:
    d = np.diff(occ)
    print(f"  first offsets: {[hex(HDR_END+o) for o in occ[:8]]}")
    print(f"  deltas: {d[:10]} ... uniq ratios: {np.unique(d[:200])}")
