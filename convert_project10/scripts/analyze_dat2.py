# -*- coding: utf-8 -*-
"""通过 KQI 魔数位置间距推断记录结构"""
import numpy as np
from collections import Counter

DAT = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Pos\20260818010209.dat"
raw = open(DAT, 'rb').read()

occ = []
p = 0
while True:
    j = raw.find(b'KQI\x00', p)
    if j < 0: break
    occ.append(j); p = j + 1
occ = np.array(occ)
print(f"KQI magic count: {len(occ)}")
d = np.diff(occ)
c = Counter(d.tolist())
print("spacing -> count (top 15):")
for v, n in c.most_common(15):
    print(f"  {v:6d} B × {n}")

# 首记录（大头部）之后的记录头字节模式
print("\n=== hexdump: file head 128B ===")
for i in range(0, 128, 16):
    ch = raw[i:i+16]
    print(f"  {i:08x}  " + ' '.join(f'{b:02x}' for b in ch) + '  ' +
          ''.join(chr(b) if 32 <= b < 127 else '.' for b in ch))

print("\n=== 记录头字段尝试：魔数后 12 字节（第 2~6 条记录）===")
for k in range(1, 7):
    o = occ[k]
    hdr = raw[o:o+12]
    t16 = int.from_bytes(raw[o+4:o+6], 'little')
    l16 = int.from_bytes(raw[o+6:o+8], 'little')
    t8 = raw[o+4]
    print(f"  occ[{k}]@{o:#x} delta={occ[k]-occ[k-1]}: {hdr.hex()}  u16@4={t16} u16@6={l16} u8@4={t8}")

# 检查 delta=69 的记录：u16@6 是否 == 69-头长
print("\n=== 若头=4+2+2+1=9B: plen 应=60; 验证 delta 与 len 字段关系 ===")
ok = 0
for k in range(1, min(len(occ), 2000)):
    delta = occ[k] - occ[k-1]
    for hlen in (9, 10, 12, 8):
        pass
# 试 u8 长度@5 或 u16@7 等
for hlen, lpos, lwidth in [(9, 6, 1), (9, 6, 2), (10, 6, 2), (8, 5, 1), (9, 8, 1), (10, 8, 2)]:
    ok = 0; tot = 0
    for k in range(1, min(len(occ), 3000)):
        delta = occ[k] - occ[k-1]
        ln = int.from_bytes(raw[occ[k-1]+lpos:occ[k-1]+lpos+lwidth], 'little')
        tot += 1
        if ln + hlen == delta: ok += 1
    print(f"  hlen={hlen} len@{lpos} w{lwidth}: match {ok}/{tot}")
