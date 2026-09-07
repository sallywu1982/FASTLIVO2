# -*- coding: utf-8 -*-
"""
solve_beta.py — 从俯仰过渡段(t=45~64s)的旋转轴向测定外参偏航 β
原理: 载体俯仰时旋转轴 a 在雷达系与 IMU 系的像差 = 常量外参。
  IMU 侧: 平滑重力方向 g(t) 序列 → 相邻对叉积 = 旋转轴(chip 系)
  雷达侧: 逐帧地面法向 n(t)（投影众数 + PCA 精化）→ 叉积 = 旋转轴(雷达系)
  模型: axis_cloud ≈ W·Rz(β)·axis_chip
"""
import os, glob
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")
W = np.load(os.path.join(OUT, 'wedge_W.npy'))
imu = np.load(os.path.join(OUT, 'imu.npz'))
t_ns, acc = imu['t_ns'], imu['acc']
t0 = int(t_ns[0])
ts_list = np.array([int(np.load(fn)['ts0_ns']) for fn in sorted(glob.glob(os.path.join(OUT, 'lidar_frames', 'frame_*.npz')))])

T1, T2 = 43.0, 64.0

# ---- IMU 侧: 0.5s 窗口平滑重力 ----
gs, gts = [], []
for tv in np.arange(T1, T2, 0.5):
    m = ((t_ns - t0)/1e9 >= tv) & ((t_ns - t0)/1e9 < tv+0.5)
    if m.sum() < 50: continue
    g = acc[m].mean(axis=0); g /= np.linalg.norm(g)
    gs.append(g); gts.append(tv)
gs = np.array(gs)

# ---- 雷达侧: 逐帧地面法向 ----
def ground_normal(fi, n_hint):
    P = np.load(os.path.join(OUT, 'lidar_frames', f'frame_{fi:05d}.npz'))['xyz']
    r = np.linalg.norm(P, axis=1)
    P = P[(r > 1.5) & (r < 25)]
    d = P @ n_hint
    # 沿提示法向投影取众数薄层
    h, e = np.histogram(d, bins=np.arange(d.min(), d.max()+0.1, 0.1))
    k = np.argmax(h)
    sel = (d >= e[k]-0.10) & (d <= e[k]+0.10)
    if sel.sum() < 300: return None
    gp = P[sel]
    c = gp.mean(axis=0)
    U, S, Vt = np.linalg.svd(gp - c, full_matrices=False)
    n = Vt[2]
    if n @ n_hint < 0: n = -n
    return n

# 初始提示: W·g(t) （已验证的倾斜跟踪）
ns_, nts = [], []
n_hint = np.array([0, 0, 1.0])
for tv in np.arange(T1, T2, 0.2):
    fi = int(np.searchsorted(ts_list, t0 + tv*1e9)); fi = min(fi, len(ts_list)-1)
    i = np.searchsorted(t_ns, t0 + tv*1e9)
    g = acc[max(0,i-100):i+100].mean(axis=0); g /= np.linalg.norm(g)
    n = ground_normal(fi, W @ g)
    if n is not None:
        ns_.append(n); nts.append(tv)
ns_ = np.array(ns_)
print(f"重力样本 {len(gs)} 帧, 地面法向 {len(ns_)} 帧")

# ---- 构造旋转轴对（角度差>2°）----
pairs = []
for i in range(len(gs)):
    for j in range(i+1, len(gs)):
        ang = np.degrees(np.arccos(np.clip(gs[i] @ gs[j], -1, 1)))
        if 3.0 < ang < 40.0:
            # 找时间上对应的法向对
            ti, tj = gts[i], gts[j]
            ki = np.argmin(np.abs(np.array(nts)-ti)); kj = np.argmin(np.abs(np.array(nts)-tj))
            angc = np.degrees(np.arccos(np.clip(ns_[ki] @ ns_[kj], -1, 1)))
            if abs(angc - ang) < 2.0:
                a_chip = np.cross(gs[i], gs[j]); a_chip /= np.linalg.norm(a_chip)
                a_cloud = np.cross(ns_[ki], ns_[kj]); a_cloud /= np.linalg.norm(a_cloud)
                pairs.append((a_chip, a_cloud, ang))
print(f"有效轴对: {len(pairs)}, 平均角距 {np.mean([p[2] for p in pairs]):.1f}°")

def Rz(d): a = np.radians(d); return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])

# ---- 扫描 β ----
best = None
for beta in range(0, 360, 2):
    Rb = W @ Rz(beta)
    err = 0.0
    for a_chip, a_cloud, _ in pairs:
        pred = Rb @ a_chip
        err += np.degrees(np.arccos(np.clip(pred @ a_cloud, -1, 1)))
    err /= len(pairs)
    if best is None or err < best[1]:
        best = (beta, err)
print(f"\n最优 β = {best[0]}°  (平均轴向误差 {best[1]:.2f}°)")
# 4 个离散候选的误差
for b in (0, 90, 180, 270, best[0]):
    Rb = W @ Rz(b)
    err = np.mean([np.degrees(np.arccos(np.clip((Rb @ ac) @ acl, -1, 1))) for ac, acl, _ in pairs])
    print(f"  β={b:+4d}°: 轴向误差 {err:.2f}°")
