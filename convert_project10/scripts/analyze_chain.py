# -*- coding: utf-8 -*-
"""链式解析测试：[u16 n][n×14B 记录] 微包假设 + 突发块边界字节检查"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000

f = open(LID, 'rb'); f.seek(HDR_END + BLK)   # blk1
raw = f.read(BLK)
f.close()

def hx(off, n=48, pre=8):
    lo = max(0, off-pre)
    for i in range(lo, min(len(raw), off+n), 16):
        c = raw[i:i+16]
        h = ' '.join(f'{b:02x}' for b in c)
        a = ''.join(chr(b) if 32 <= b < 127 else '.' for b in c)
        print(f'  {i:06x}  {h:<48}  {a}')

# ---- A. 精确突发边界（不平滑，非零游程合并 <=2B 间隙）----
nz = (np.frombuffer(raw, dtype=np.uint8) != 0)
runs = []
i = 0
N = len(nz)
while i < N:
    if nz[i]:
        j = i
        while j < N and (nz[j] or (j+2 < N and nz[j+2]) or (j+1 < N and nz[j+1] and j+2 < N and nz[j+2])):
            j += 1
        runs.append((i, j))
        i = j
    else:
        i += 1
sizes = np.array([e-s for s, e in runs])
gaps = np.array([runs[k+1][0]-runs[k][1] for k in range(len(runs)-1)])
print(f"exact runs: {len(runs)}, sizes: med={np.median(sizes):.0f}, pct[10,50,90]={np.percentile(sizes,[10,50,90])}")
print(f"gaps: med={np.median(gaps):.0f}, pct[10,50,90]={np.percentile(gaps,[10,50,90])}, zero-gap count={(gaps==0).sum()}")
print(f"size %% 14: {np.bincount(sizes % 14)[:14]}")
print(f"(size-2) %% 14: {np.bincount((sizes-2) % 14)[:14]}")

# ---- B. 前几个突发块的字节细节（含前 8B）----
print("\n=== burst byte details ===")
for k in range(6):
    s, e = runs[k]
    print(f"run{k}: [{s:#x},{e:#x}) size={e-s}")
    hx(s, 40, pre=10)

# ---- C. 链式解析：[u16 n][n×14B] ----
def chain(start, max_pkt=100000):
    pos = start
    pkts, pts, bad = 0, 0, 0
    while pos + 2 <= len(raw) and pkts < max_pkt:
        n = int.from_bytes(raw[pos:pos+2], 'little')
        end = pos + 2 + n*14
        if n == 0 or end > len(raw):
            break
        recs = np.frombuffer(raw[pos+2:end], dtype='<i4').reshape(-1, 3) if n else np.zeros((0,3),'i4')
        if recs.size and np.abs(recs.astype(np.int64)).max() > 100000:
            break
        pkts += 1; pts += n; pos = end
    return pkts, pts, pos

# 尝试从多个种子启动链式解析
print("\n=== chain parse from seeds ===")
for seed in [0x307, 0x419, 0xAC8, 0x700, 0x1000]:
    if seed < len(raw):
        pkts, pts, pos = chain(seed)
        print(f"seed {seed:#x}: pkts={pkts} pts={pts} ended at {pos:#x} ({pos}/{len(raw)} = {pos/len(raw):.1%})")

# ---- D. u16 头的值分布（若链成立）----
s0 = 0x307
pos, vals = s0, []
for _ in range(500):
    if pos+2 > len(raw): break
    n = int.from_bytes(raw[pos:pos+2], 'little')
    if n == 0 or pos+2+n*14 > len(raw): break
    vals.append(n); pos += 2+n*14
print(f"\nu16 header values (first 40): {vals[:40]}")
print(f"u16 stats: min={min(vals) if vals else '-'} max={max(vals) if vals else '-'}")
