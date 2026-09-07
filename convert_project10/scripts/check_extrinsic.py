# -*- coding: utf-8 -*-
"""用静止段重力方向验证 LiDAR→IMU 旋转（±90° Z 轴假设）"""
import os, glob
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")

# ---- 1. IMU 静止段重力方向（IMU 系）----
imu = np.load(os.path.join(OUT, 'imu.npz'))
t = imu['t_ns']
acc = imu['acc']
m_static = t < t[0] + 5e9
g_imu = acc[m_static].mean(axis=0)
g_imu /= np.linalg.norm(g_imu)
print(f"IMU gravity dir (static 5s): {np.round(g_imu, 4)}")

# ---- 2. 雷达静止段：合并前 ~50 帧（~5s），拟合地面平面法向 ----
pts_all = []
for fn in sorted(glob.glob(os.path.join(OUT, 'lidar_frames', 'frame_*.npz')))[:50]:
    d = np.load(fn)
    pts_all.append(d['xyz'])
P = np.concatenate(pts_all)
r = np.linalg.norm(P, axis=1)
P = P[(r > 1.0) & (r < 30)]
print(f"static lidar points: {len(P)}")

# 地面点 = z 最低的一批（先粗略按 z 直方图取低处密集层）
zb = P[:, 2]
hist, edges = np.histogram(zb, bins=100)
k = np.argmax(hist)
z_ground = edges[k]
print(f"ground layer z ≈ {z_ground:.2f} m (hist peak), z range {zb.min():.2f}..{zb.max():.2f}")
gp = P[np.abs(zb - z_ground) < 0.15]
print(f"ground points: {len(gp)}")

# PCA 拟合平面
c = gp.mean(axis=0)
U, S, Vt = np.linalg.svd(gp - c, full_matrices=False)
n = Vt[2]
if n[2] > 0: n = -n   # 重力方向（指向地心，地面点在雷达下方 → 法向 z 分量与雷达 z 反号待验证）
g_lidar_up = n if n[2] > 0 else -n
print(f"plane normal (up): {np.round(g_lidar_up, 4)}, singular ratio {S[2]/S[0]:.2e}")

# ---- 3. 检验候选旋转 ----
def Rz(deg):
    a = np.radians(deg)
    c_, s_ = np.cos(a), np.sin(a)
    return np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]])

cands = {'Rz(-90)': Rz(-90), 'Rz(+90)': Rz(90), 'I': np.eye(3)}
print("\ncandidate R_lidar->imu  vs  gravity match (|cos| 越接近1越好):")
for name, R in cands.items():
    v = R @ g_lidar_up
    cos_up = np.dot(v, g_imu)          # g_imu 是比力(≈-重力/上方向反号?) 静止时比力= +up
    print(f"  {name}: R@up_lidar={np.round(v,4)}, cos(R@up, g_imu)={cos_up:+.4f}")
