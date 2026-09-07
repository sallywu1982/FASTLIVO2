# -*- coding: utf-8 -*-
"""viz_results.py — 把 run_viz.sh 的成果渲染成效果图: 轨迹图 + 点云地图图"""
import os, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")

# ---- 1. 轨迹 ----
txt = open(os.path.join(OUT, 'odom_rec.log'), errors='ignore').read()
stamps = re.findall(r'stamp:\s*\n\s*secs: (\d+)\s*\n\s*nsecs: (\d+)', txt)
pos = re.findall(r'position:\s*\n\s*x: ([-\d.eE+]+)\s*\n\s*y: ([-\d.eE+]+)\s*\n\s*z: ([-\d.eE+]+)', txt)
n = min(len(stamps), len(pos))
t = np.array([int(s)+int(ns)*1e-9 for s, ns in stamps[:n]])
p = np.array([[float(a), float(b), float(c)] for a, b, c in pos[:n]])
t -= t.min()
print(f"轨迹: {n} 点, 时长 {t[-1]:.0f}s, 范围 x[{p[:,0].min():.0f},{p[:,0].max():.0f}] y[{p[:,1].min():.0f},{p[:,1].max():.0f}] z[{p[:,2].min():.1f},{p[:,2].max():.1f}]")

fig, axes = plt.subplots(1, 3, figsize=(20, 6.5))
sc = axes[0].scatter(p[:,0], p[:,1], c=t, s=2, cmap='viridis')
axes[0].plot(p[:,0], p[:,1], color='k', lw=0.3, alpha=0.4)
axes[0].set_aspect('equal'); axes[0].set_title(f'FAST-LIVO2 trajectory XY (top view, {n} poses, colored by time)')
axes[0].set_xlabel('x [m]'); axes[0].set_ylabel('y [m]')
plt.colorbar(sc, ax=axes[0], label='time [s]', shrink=0.85)
axes[1].plot(t, p[:,2], lw=0.8)
axes[1].set_title('height z vs time'); axes[1].set_xlabel('t [s]'); axes[1].set_ylabel('z [m]'); axes[1].grid(alpha=0.3)
d = np.sqrt(((np.diff(p[:,:2], axis=0))**2).sum(axis=1))
v = d / np.diff(t)
axes[2].plot(t[1:], np.clip(v, 0, np.percentile(v, 99.5)), lw=0.6)
axes[2].set_title(f'speed (mean {v.mean():.2f} m/s, max {v.max():.2f})')
axes[2].set_xlabel('t [s]'); axes[2].set_ylabel('v [m/s]'); axes[2].grid(alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUT, 'viz_trajectory.png'), dpi=110)
print("saved viz_trajectory.png")

# ---- 2. 点云地图（增量保存的分片 PCD，XYZRGB）----
pcd_dir = os.path.join(OUT, 'pcd')
files = sorted(f for f in os.listdir(pcd_dir) if f.endswith('.pcd')) if os.path.exists(pcd_dir) else []
rec = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
clouds = []
for fn in files:
    raw = open(os.path.join(pcd_dir, fn), 'rb').read()
    k = raw.find(b'DATA binary')
    if k < 0:
        continue
    header = raw[:k].decode('ascii', errors='ignore')
    npts = int(re.search(r'POINTS (\d+)', header).group(1))
    body = raw[k+raw[k:].find(b'\n')+1:]
    if npts and len(body) >= npts*16:
        clouds.append(np.frombuffer(body[:npts*16], dtype=rec, count=npts))
if clouds:
    arr = np.concatenate(clouds)
    npts = len(arr)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    rgb = arr['rgb'].astype(np.uint32)
    r = ((rgb >> 16) & 255).astype(np.float32)/255
    g = ((rgb >> 8) & 255).astype(np.float32)/255
    b = (rgb & 255).astype(np.float32)/255
    print(f"地图: {len(files)} 个分片共 {npts} 点, x[{xyz[:,0].min():.0f},{xyz[:,0].max():.0f}] y[{xyz[:,1].min():.0f},{xyz[:,1].max():.0f}] z[{xyz[:,2].min():.1f},{xyz[:,2].max():.1f}], 有着色点比例 {np.mean((r+g+b)>0):.2f}")
    m = np.random.rand(npts) < min(1.0, 2_500_000/npts)
    q = xyz[m]; col = np.stack([r[m], g[m], b[m]], axis=1)
    has = (col.sum(axis=1) > 0.05)
    fig, axes = plt.subplots(1, 2, figsize=(20, 9))
    axes[0].scatter(q[:,0], q[:,1], c=np.where(has[:,None], col, [0.55,0.55,0.55]), s=0.05, marker='.')
    axes[0].plot(p[:,0], p[:,1], color='red', lw=1.2, label='trajectory')
    axes[0].legend(); axes[0].set_aspect('equal')
    axes[0].set_title(f'registered map XY, {npts} pts (camera-colored; red = trajectory)')
    axes[0].set_xlabel('x [m]'); axes[0].set_ylabel('y [m]')
    sc = axes[1].scatter(q[:,0], q[:,2], c=q[:,1], s=0.05, cmap='turbo', marker='.')
    axes[1].set_aspect('equal')
    axes[1].set_title('map XZ (colored by y)')
    axes[1].set_xlabel('x [m]'); axes[1].set_ylabel('z [m]')
    plt.colorbar(sc, ax=axes[1], label='y [m]', shrink=0.8)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, 'viz_map.png'), dpi=110)
    print("saved viz_map.png")
else:
    print("未找到 PCD 分片")
