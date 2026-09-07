# -*- coding: utf-8 -*-
"""全文件魔数扫描：帧分组 + 块间杂项区(708B)解析"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
MAGIC = bytes([0xc6, 0x00, 0x01, 0x02])

f = open(LID, 'rb')
f.seek(0, 2); fsize = f.tell()
f.seek(HDR_END)
raw = f.read()          # 整个数据区（287MB，可全读）
f.close()
print(f"data region: {len(raw)}B")

occ = []
p = 0
while True:
    j = raw.find(MAGIC, p)
    if j < 0: break
    # 验证 +20 处 u64 是否为合理 epoch-ns（2026 年 ≈ 1.75e18~1.79e18）
    ts = int.from_bytes(raw[j+20:j+28], 'little')
    if 1750000000000000000 < ts < 1800000000000000000:
        occ.append(j)
    p = j + 1
occ = np.array(occ)
print(f"magic w/ valid ts: {len(occ)} → frames = {len(occ)/200:.2f}")
d = np.diff(occ)
# 非常规间距（≠1380）
irreg = np.where(d != 1380)[0]
print(f"irregular spacings: {len(irreg)}")
if len(irreg):
    print(f"first 20 irregular gaps: {d[irreg[:20]].tolist()}")
    print(f"at occ idx: {irreg[:20].tolist()}")
    print(f"positions: {[hex(HDR_END+occ[i]) for i in irreg[:10]]}")

# 帧边界 = irregular gap 后的第一个 magic；统计每帧行为
if len(irreg):
    print("\nframe boundary structure (first 5 frames):")
    for i in irreg[:5]:
        print(f"  frame@data+{occ[i+1]:#x}: prev-pkt-end={occ[i]:#x}, gap={d[i]}B")

# ---- 杂项区内容：第一个 magic 前的 708B + 每帧 irregular gap 区 ----
def dump(buf, base, off, n):
    for i in range(off, off+n, 16):
        c = buf[i:i+16]
        h = ' '.join(f'{b:02x}' for b in c)
        a = ''.join(chr(b) if 32 <= b < 127 else '.' for b in c)
        print(f'  {base+i:08x}  {h:<48}  {a}')

print("\n=== misc region before FIRST magic (data start, 200B around first records) ===")
dump(raw, HDR_END, 0x60, 256)

# 帧间 gap 的杂项区（取第二个 irregular gap 处）
if len(irreg) > 1:
    i = irreg[1]
    gap_start = occ[i] + 1380
    print(f"\n=== inter-frame misc region @data+{gap_start:#x} (gap={d[i]}B) ===")
    dump(raw, HDR_END, gap_start, min(d[i], 384))
