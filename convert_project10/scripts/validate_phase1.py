# -*- coding: utf-8 -*-
"""阶段1验证：点云可视化 + IMU 曲线 + logLidarParse 交叉核对"""
import os, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")
LOG = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\shuttle_debug\LidarDecode\logLidarParse.txt"

# ---- 1. 点云可视化（3 个不同时段的帧）----
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
for col, fi in enumerate([100, 1500, 2800]):
    d = np.load(os.path.join(OUT, "lidar_frames", f"frame_{fi:05d}.npz"))
    xyz, attr = d['xyz'], d['attr']
    r = np.linalg.norm(xyz, axis=1)
    m = (r > 0.5) & (r < 60)
    x, y, z = xyz[m].T
    sc = axes[0, col].scatter(x, y, c=z, s=0.5, cmap='turbo')
    axes[0, col].set_title(f'frame {fi} XY (n={m.sum()})')
    axes[0, col].set_aspect('equal')
    plt.colorbar(sc, ax=axes[0, col], label='z(m)')
    axes[1, col].scatter(x, z, c=attr[m], s=0.5, cmap='viridis')
    axes[1, col].set_title(f'frame {fi} XZ (attr colored)')
    axes[1, col].set_aspect('equal')
plt.tight_layout()
plt.savefig(os.path.join(OUT, 'lidar_frames_check.png'), dpi=110)
print("saved lidar_frames_check.png")

# ---- 2. IMU 曲线 ----
imu = np.load(os.path.join(OUT, 'imu.npz'))
t = (imu['t_ns'] - imu['t_ns'][0]) / 1e9
gyro, acc = imu['gyro'], imu['acc']
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
axes[0].plot(t, acc, lw=0.4)
axes[0].set_ylabel('acc (g)'); axes[0].legend(['a1','a2','a3'])
axes[0].set_title('IMU acc (g units)')
axes[1].plot(t, gyro, lw=0.4)
axes[1].set_ylabel('gyro (rad/s)'); axes[1].legend(['g1','g2','g3'])
axes[1].set_title('IMU gyro (rad/s)')
dt = np.diff(imu['t_ns']) / 1e6
axes[2].plot(t[1:], dt, lw=0.4)
axes[2].set_ylabel('dt (ms)'); axes[2].set_xlabel('t (s)')
axes[2].set_title(f'IMU sample interval (median={np.median(dt):.2f}ms)')
for ax in axes: ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT, 'imu_check.png'), dpi=110)
print("saved imu_check.png")

# ---- 3. logLidarParse 交叉核对 ----
pat = re.compile(r'LidarFrameNum\s*=\s*(\d+)\s+LidarPointsSize\s*=\s*(\d+)\s+LidarTime from ([\d.]+) to ([\d.]+)')
logs = []
for line in open(LOG, encoding='utf-8', errors='ignore'):
    m = pat.search(line)
    if m:
        logs.append((int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))))
print(f"\nlogLidarParse frames: {len(logs)}")

dec_n, dec_t0 = [], []
for fi in range(3004):
    d = np.load(os.path.join(OUT, "lidar_frames", f"frame_{fi:05d}.npz"))
    dec_n.append(len(d['xyz']))
    dec_t0.append(d['ts0_ns'] / 1e9)
dec_n = np.array(dec_n); dec_t0 = np.array(dec_t0)

# 时间对齐：log 的 GPS SOW → epoch（加 18s 闰秒；周起点 2026-08-16 00:00 UTC = 1787049600? 用首帧锚定）
# 用 log frame k=1 与 decoded frame0 对齐验证
import datetime
week_start = int(datetime.datetime(2026, 8, 16, 0, 0, 0, tzinfo=datetime.timezone.utc).timestamp())
n = min(len(logs), len(dec_n))
log_t_epoch = np.array([week_start + logs[i][2] - 18 for i in range(n)])
log_n = np.array([logs[i][1] for i in range(n)])
dt_err = dec_t0[:n] - log_t_epoch
print(f"time align (decoded frame0 ↔ log frame1):")
print(f"  decoded t0[0] = {dec_t0[0]:.3f}")
print(f"  log frame1 epoch = {log_t_epoch[0]:.3f}  (week_start={week_start})")
print(f"  dt error: median={np.median(dt_err)*1000:.1f}ms, p95={np.percentile(dt_err,95)*1000:.1f}ms")
r = log_n[:n] - dec_n[:n]
print(f"point count diff (log - decoded): mean={r.mean():.0f}, median={np.median(r):.0f}, p10={np.percentile(r,10):.0f}, p90={np.percentile(r,90):.0f}")
