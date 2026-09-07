# -*- coding: utf-8 -*-
"""
decode_dat.py — Pos .dat (KQI 容器) IMU 解码器
格式（逆向确认）:
  记录 = ['KQI\0'][u16 type][u16 payload_len][u8 sub_len] + payload
  首条大记录内嵌参数头（与 .lid 相同）
  type=1 载荷 = n × 60B IMU 样本块:
     样本 58B + 2B CRC:
       +0  u32 常量 0x01000000
       +4  u16 样本计数器（全局递增）
       +26 u64 时间戳 epoch-ns
       +34 6×f32 (g1,g2,g3, a1,a2,a3)  ← 前3疑似陀螺、后3加计(g 单位)
输出: out/imu.npz (t_ns, gyro, acc, counter)
"""
import os
import numpy as np

DAT = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Pos\20260818010209.dat"
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out", "imu.npz")

raw = open(DAT, 'rb').read()

# 扫描魔数位置（避免依赖不可靠的首条长度字段）
occ = []
p = 0
while True:
    j = raw.find(b'KQI\x00', p)
    if j < 0: break
    occ.append(j); p = j + 1
occ = np.array(occ)
print(f"KQI records: {len(occ)}")

# 解析 type=1 记录中的 IMU 样本
subs = []
for k in range(1, len(occ)):
    o = occ[k]
    end = occ[k+1] if k+1 < len(occ) else len(raw)
    rtype = int.from_bytes(raw[o+4:o+6], 'little')
    if rtype != 1:
        continue
    plen = end - o - 9
    payload = raw[o+9:end]
    # 60B 块切分
    for s in range(0, plen - 59, 60):
        blk = payload[s:s+58]
        if blk[:4] != b'\x00\x00\x00\x01':
            continue
        counter = int.from_bytes(blk[4:6], 'little')
        ts = int.from_bytes(blk[26:34], 'little')
        vals = np.frombuffer(blk[34:58], dtype='<f4')
        if not (1700000000000000000 < ts < 1850000000000000000):
            continue
        subs.append((ts, counter, *vals.tolist()))

print(f"IMU samples: {len(subs)}")
arr = np.array(subs, dtype=np.float64)
order = np.argsort(arr[:, 0])
arr = arr[order]

t = arr[:, 0]
counter = arr[:, 1]
gyro = arr[:, 2:5]
acc = arr[:, 5:8]

# ---- 验证统计 ----
dt = np.diff(t) / 1e6  # ms
print(f"\ntime: {t[0]/1e9:.6f} -> {t[-1]/1e9:.6f}  span={(t[-1]-t[0])/1e9:.2f}s")
print(f"dt(ms): median={np.median(dt):.3f}, mean={dt.mean():.3f}, std={dt.std():.3f}, "
      f"p1={np.percentile(dt,1):.2f}, p99={np.percentile(dt,99):.2f}")
print(f"implied rate: {1e9/np.median(np.diff(t)):.1f} Hz")
dc = np.diff(counter).astype(int)
print(f"counter: {counter[0]:.0f}..{counter[-1]:.0f}, steps uniq={np.unique(dc)[:8]}, gaps(>1)={(dc>1).sum()}")
am = np.linalg.norm(acc, axis=1)
print(f"\n|acc| (g): mean={am.mean():.4f}, std={am.std():.4f}, p1={np.percentile(am,1):.3f}, p99={np.percentile(am,99):.3f}")
# 静止段（前 5 秒）
m0 = t < t[0] + 5e9
print(f"static(first 5s): |acc| mean={am[m0].mean():.4f} std={am[m0].std():.5f}; "
      f"gyro mean={np.round(gyro[m0].mean(axis=0),5)} std={np.round(gyro[m0].std(axis=0),5)}")
print(f"acc mean static: {np.round(acc[m0].mean(axis=0),4)}")
print(f"gyro overall range: min={np.round(gyro.min(axis=0),3)} max={np.round(gyro.max(axis=0),3)}")
print(f"acc  overall range: min={np.round(acc.min(axis=0),3)} max={np.round(acc.max(axis=0),3)}")

np.savez(OUT, t_ns=t.astype(np.int64), counter=counter.astype(np.int64),
         gyro=gyro, acc=acc)
print(f"\nsaved -> {OUT}")
