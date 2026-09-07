# -*- coding: utf-8 -*-
"""扫描 c6 00 01 02 魔数 → 完整子包布局验证"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5
BLK = 276000
MAGIC = bytes([0xc6, 0x00, 0x01, 0x02])

f = open(LID, 'rb')
f.seek(HDR_END + BLK)
raw = f.read(BLK)          # blk1
f.close()

occ = []
p = 0
while True:
    j = raw.find(MAGIC, p)
    if j < 0: break
    occ.append(j); p = j + 1
occ = np.array(occ)
print(f"magic count in blk1: {len(occ)}")
print(f"first 12 offsets: {occ[:12].tolist()}")
d = np.diff(occ)
print(f"spacings uniq: {np.unique(d)[:20]} (min={d.min()} max={d.max()})")
print(f"offset % 1380 histogram: {np.bincount((occ % 1380).tolist())[:8]} ... nonzero idx: {np.where(np.bincount((occ % 1380).tolist())>0)[0][:10]}")

# 时间戳 u64 at +20
ts = np.array([int.from_bytes(raw[o+20:o+28], 'little') for o in occ[:50]])
dts = np.diff(ts)
print(f"\nts[0]={ts[0]}  dts uniq(first50): {np.unique(dts)}")
print(f"ts[0] as float seconds = {ts[0]/1e9:.6f}")
print(f"ts[-1] = {ts[-1]} ({ts[-1]/1e9:.6f}s), span={(ts[-1]-ts[0])/1e9:.4f}s over {len(ts)} pkts")
import datetime
print("ts[0] as epoch-ns UTC:", datetime.datetime.utcfromtimestamp(ts[0]/1e9) if 1e9 < ts[0] < 1e19 else 'n/a')
print("ts[0] as epoch-us UTC:", datetime.datetime.utcfromtimestamp(ts[0]/1e6) if 1e12 < ts[0] < 1e16 else 'n/a')

# 第一个子包的完整 hexdump（前 96B）
o0 = occ[0]
print(f"\nfirst pkt @blk1+{o0:#x}:")
for i in range(o0, o0+112, 16):
    c = raw[i:i+16]
    print(f"  {i:06x}  " + ' '.join(f'{b:02x}' for b in c))

# 检查 magic+4..+20 的 16B 保留区是否全零（统计前 50 包）
z = np.array([all(b == 0 for b in raw[o+4:o+20]) for o in occ[:50]])
print(f"\nreserved16 all-zero: {z.sum()}/{len(z)}")
# magic-4..magic 前一字节（上一包的尾巴）
for o in occ[:6]:
    print(f"  pkt@{o:#x} trailer(prev4B): {raw[o-4:o].hex()}  magic+28 first rec: {raw[o+28:o+42].hex()}")
