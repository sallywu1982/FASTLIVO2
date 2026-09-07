# -*- coding: utf-8 -*-
"""扫描所有 XX 00 01 02 魔数变体 + 全文件密度分布"""
import numpy as np
from collections import Counter

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5

f = open(LID, 'rb'); f.seek(HDR_END)
raw = f.read()
f.close()

u32 = np.frombuffer(raw[:len(raw)//4*4], dtype='<u4')
# 模式 XX0001 02 → u32 值 = 0x020100XX
cand = np.where((u32 & 0xFFFFFF00) == 0x02010000)[0]
print(f"pattern XX 00 01 02 count: {len(cand)}")
# 按首字节分类
first_bytes = (u32[cand] & 0xFF)
cnt = Counter(first_bytes.tolist())
print("by first byte:", dict(sorted(cnt.items())))

# 各类型的分布区间
for fb in sorted(cnt):
    idx = cand[first_bytes == fb]
    print(f"  type {fb:02x}: n={len(idx)}, data-range [{idx.min()*4:#x}, {idx.max()*4:#x}]")

# 密度图：4KB 粒度，非零占比
nz = (np.frombuffer(raw, dtype=np.uint8) != 0)
K = 4096
d = nz[:len(nz)//K*K].reshape(-1, K).mean(axis=1)
print(f"\ndensity map (4KB blocks, {len(d)} blocks) — 每行 64 块:")
sym = lambda v: '.' if v < 0.02 else ('-' if v < 0.2 else ('+' if v < 0.6 else '#'))
for r in range(0, len(d), 64):
    print(f"{r*K//1024:6d}K  " + ''.join(sym(v) for v in d[r:r+64]))
