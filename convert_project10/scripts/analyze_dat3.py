# -*- coding: utf-8 -*-
"""解析 60B/120B 载荷：找 IMU 字段（时间戳 + 加计 + 陀螺）"""
import numpy as np

DAT = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Pos\20260818010209.dat"
raw = open(DAT, 'rb').read()

occ = []
p = 0
while True:
    j = raw.find(b'KQI\x00', p)
    if j < 0: break
    occ.append(j); p = j + 1

# 解析记录流（跳过首条大头部）
recs = []
for k in range(1, len(occ)):
    o = occ[k]
    delta = occ[k] - occ[k-1]
    rtype = int.from_bytes(raw[o+4:o+6], 'little')
    plen = int.from_bytes(raw[o+6:o+8], 'little')
    u8 = raw[o+8]
    recs.append((o, rtype, plen, u8, delta))

print("u8@8 by plen:")
from collections import Counter
c = Counter((plen, u8) for o, t, plen, u8, d in recs)
for (pl, u), n in sorted(c.items()):
    print(f"  plen={pl} u8@8={u}: {n}")

# ---- 60B 载荷 hexdump 前 3 条 + 多种解释 ----
r60 = [r for r in recs if r[2] == 60][:3]
r120 = [r for r in recs if r[2] == 120][:3]

def show(o, plen, tag):
    pl = raw[o+9:o+9+plen]
    print(f"\n=== {tag} @{o:#x} ===")
    for i in range(0, plen, 16):
        ch = pl[i:i+16]
        print(f"  +{i:3d}  " + ' '.join(f'{b:02x}' for b in ch))
    # 解释1: u64 ts + i16×26
    ts = int.from_bytes(pl[:8], 'little')
    print(f"  u64@0 = {ts}  (as epoch-ns: {ts/1e9:.6f})" )
    i16 = np.frombuffer(pl, dtype='<i2')
    print(f"  i16: {i16.tolist()}")
    f32 = np.frombuffer(pl[:len(pl)//4*4], dtype='<f4')
    print(f"  f32: {np.round(f32, 4).tolist()}")

for o, t, plen, u8, d in r60[:2]:
    show(o, plen, f"60B rec type={t}")
for o, t, plen, u8, d in r120[:2]:
    show(o, plen, f"120B rec type={t}")

# ---- 连续 60B 记录的 u64@0 差分（找采样间隔） ----
seq = [r for r in recs if r[2] == 60][:100]
ts = [int.from_bytes(raw[o+9:o+17], 'little') for o, t, plen, u8, d in seq]
dts = np.diff(ts)
print(f"\n60B rec u64@0 diffs: uniq={np.unique(dts)[:10]}")
print(f"ts[0]={ts[0]} ({ts[0]/1e9:.6f}s as epoch-ns)")

# 若 u64 不对，试 u32@4 / u32@8
for pos in (0, 4, 8):
    vv = [int.from_bytes(raw[o+9+pos:o+13+pos], 'little') for o, t, plen, u8, d in seq]
    dv = np.diff(vv)
    print(f"u32@{pos}: first={vv[0]}, diffs uniq(first20)={np.unique(dv)[:6]}")
