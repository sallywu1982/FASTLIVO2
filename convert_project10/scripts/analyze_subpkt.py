# -*- coding: utf-8 -*-
"""子包头（16B 间隙）内容解析 + 记录对齐"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END + BLK)
raw = f.read(BLK)
f.close()

# ---- A. 16B 粒度密度 → 稠密段 + 间隙包头 ----
nz = (np.frombuffer(raw, dtype=np.uint8) != 0).astype(np.int32)
d16 = nz[:BLK//16*16].reshape(-1, 16).sum(axis=1)
dense = d16 >= 8
spans = []
s = None
for i, v in enumerate(dense):
    if v and s is None: s = i
    if (not v or i == len(dense)-1) and s is not None:
        e = i if not v else i+1
        spans.append((s*16, e*16))
        s = None
spans = [(a, b) for a, b in spans if b-a >= 64]
print(f"spans>=64B: {len(spans)}")

def dump(off, n, label):
    print(f"{label} @blk+{off:#x}:")
    for i in range(off, off+n, 16):
        c = raw[i:i+16]
        h = ' '.join(f'{b:02x}' for b in c)
        a = ''.join(chr(b) if 32 <= b < 127 else '.' for b in c)
        print(f"  {i:06x}  {h:<48}  {a}")

# 看第 3~8 个稠密段的开头及其前面的 16B（应为子包头）
for k in range(2, 8):
    a, b = spans[k]
    print(f"\n--- span{k}: [{a:#x},{b:#x}) size={b-a} ---")
    dump(max(0, a-16), 48, "span start with 16B pre-header")

# ---- B. 相邻段起点差 ----
starts = np.array([a for a, b in spans])
print("\nspan start deltas (first 30):", np.diff(starts)[:30].tolist())

# ---- C. 段内 14B 对齐（在第一个 >1KB 的段）----
big = max(spans, key=lambda t: t[1]-t[0])
a, b = big
seg = raw[a:b]
m = len(seg)//14
rates = []
for align in range(14):
    mm = (len(seg)-align)//14
    v = np.frombuffer(seg[align:], dtype=np.uint8)
    off = np.arange(mm)*14
    def s32(o):
        q = v[o].astype(np.int64)|(v[o+1].astype(np.int64)<<8)|(v[o+2].astype(np.int64)<<16)|(v[o+3].astype(np.int64)<<24)
        q[q>=2**31]-=2**32
        return q
    x, y, z = s32(off), s32(off+4), s32(off+8)
    u = v[off+12].astype(np.int64)|(v[off+13].astype(np.int64)<<8)
    okm = (np.abs(x)<35000)&(np.abs(y)<35000)&(np.abs(z)<35000)&(u>0)&(u<4096)
    rates.append(okm.mean())
    print(f"align {align:2d}: {okm.mean():.4f}")
