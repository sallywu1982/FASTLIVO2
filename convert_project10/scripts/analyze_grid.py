# -*- coding: utf-8 -*-
"""包网格走查：验证全文件 1380B 网格 + ts 连续性"""
import numpy as np

LID = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar\20260818010215_No000.lid"
HDR_END = 0x6B5

f = open(LID, 'rb'); f.seek(HDR_END)
raw = f.read()
f.close()
N = len(raw)

# 从 blk1 首包(708)反推网格相位，向前向后走查
grid = 708 % 1380
print(f"grid phase = {grid}")

def check(start_pkt, count, label):
    ok_magic = ok_ts = ok_res = 0
    ts_prev = None
    bad = []
    for k in range(count):
        off = start_pkt + k*1380
        if off+1380 > N: break
        m = raw[off:off+4]
        ts = int.from_bytes(raw[off+20:off+28], 'little')
        res = raw[off+4:off+20]
        if m[1:4] == b'\x00\x01\x02': ok_magic += 1
        else: bad.append((off, m.hex()))
        if 1750000000000000000 < ts < 1800000000000000000:
            ok_ts += 1
            if ts_prev is not None and ts != ts_prev + 480000 and abs(ts-ts_prev) > 5000000:
                pass
        if all(b == 0 for b in res): ok_res += 1
    print(f"{label}: magic_ok={ok_magic}/{count} ts_ok={ok_ts}/{count} reserved_zero={ok_res}/{count}")
    if bad[:3]: print(f"  bad magic examples: {bad[:3]}")

# 数据区开头到 blk1：从网格上第一个位置
first = grid + ((HDR_END-0) if False else 0)
# raw 索引 0 对应文件 HDR_END；708 是 raw 内偏移
check(708, 200, "blk1 区域")
check(708 - 200*1380, 200, "blk0 区域(708 之前)")
check(5_000_000 // 1380 * 1380 + grid, 200, "5MB 处")
check(50_000_000 // 1380 * 1380 + grid, 200, "50MB 处")
check(150_000_000 // 1380 * 1380 + grid, 200, "150MB 处")
check(287_000_000 // 1380 * 1380 + grid, 200, "287MB 处")

# ts 全文件抽样：每 1000 包取一个
print("\nts sampling every 100 pkts:")
prev = None
mono_breaks = 0
for i, off in enumerate(range(708, N-1380, 1380)):
    if i % 100: continue
    ts = int.from_bytes(raw[off+20:off+28], 'little')
    if not (1700000000000000000 < ts < 1850000000000000000):
        print(f"  !! invalid ts at raw+{off:#x}: {ts}")
        continue
    if prev is not None and ts < prev:
        mono_breaks += 1
        if mono_breaks < 5:
            print(f"  ts backward at raw+{off:#x}: {prev} -> {ts} (delta {(ts-prev)/1e6:.3f}ms)")
    prev = ts
print(f"  monotonic breaks: {mono_breaks}, last ts = {prev}")

# 总包数与帧数
total_pkts = (N - 708) // 1380 + 1
print(f"\ntotal packets on grid: {total_pkts}, /200 = {total_pkts/200:.2f} frames")

# 每帧点数快速统计（每 200 包 = 1 帧，帧内非零记录计数）
def frame_points(fi):
    pts = 0
    for k in range(200):
        off = 708 + (fi*200 + k)*1380
        if off+1380 > N: break
        body = raw[off+28:off+1376]
        # 14B 记录，非全零即算
        for r in range(0, len(body)-13, 14):
            if body[r:r+14].count(0) < 14: pts += 1
    return pts

for fi in [0, 1, 2, 500]:
    if fi*200*1380+708 < N:
        print(f"frame {fi}: ~{frame_points(fi)} nonzero records")
