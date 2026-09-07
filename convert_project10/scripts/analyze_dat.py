# -*- coding: utf-8 -*-
"""分析 .dat KQI 容器：记录类型统计 + IMU 载荷试解"""
import numpy as np
from collections import Counter

DAT = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Pos\20260818010209.dat"
raw = open(DAT, 'rb').read()
print(f"file size = {len(raw)}")

# 首个大头部
assert raw[:4] == b'KQI\x00', raw[:8]
print("first record: magic KQI, ", end="")
# 记录结构假设: 'KQI\0' + u16 type + u16 len + u32 ? + payload
off = 0
records = []
while off + 12 <= len(raw):
    if raw[off:off+4] != b'KQI\x00':
        print(f"\n!! bad magic at {off:#x}: {raw[off:off+8].hex()}")
        break
    rtype = int.from_bytes(raw[off+4:off+6], 'little')
    plen = int.from_bytes(raw[off+6:off+8], 'little')
    u = int.from_bytes(raw[off+8:off+12], 'little')
    records.append((rtype, plen, u, off))
    off += 12 + plen
print(f"parsed {len(records)} records to offset {off:#x} ({off/len(raw):.1%})")

cnt = Counter((t, p) for t, p, u, o in records)
print("\n(type, payload_len) -> count:")
for (t, p), c in sorted(cnt.items(), key=lambda x: -x[1]):
    print(f"  type={t:5d} plen={p:5d}  count={c}")

# 各类型的 u 字段范围
for (t, p), c in sorted(cnt.items(), key=lambda x: -x[1])[:6]:
    us = [u for tt, pp, u, o in records if tt == t and pp == p]
    print(f"type={t} plen={p}: u field min={min(us)} max={max(us)}")

# ---- hexdump 每种主要类型的前 2 条 ----
seen = set()
for t, p, u, o in records:
    if (t, p) in seen: continue
    if cnt[(t, p)] < 50: continue
    seen.add((t, p))
    print(f"\n=== type={t} plen={p} payload samples ===")
    for k in range(2):
        oo = o
        # 找该类型第 k+1 条
        idx = [r for r in records if r[0] == t and r[1] == p][k]
        oo = idx[3]
        pl = raw[oo+12:oo+12+p]
        for i in range(0, min(p, 96), 16):
            c = pl[i:i+16]
            print(f"  +{i:3d}  " + ' '.join(f'{b:02x}' for b in c))
    if len(seen) > 5: break
