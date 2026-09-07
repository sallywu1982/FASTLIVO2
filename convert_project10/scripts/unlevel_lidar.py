# -*- coding: utf-8 -*-
"""
unlevel_lidar.py — 还原 .lid 点云的原始传感器坐标
背景: 设备 INS 对输出点云做了 roll/pitch 调平(偏航保留)。FAST-LIVO2 需要原始雷达坐标。
方法: 对 .dat 解出的 200Hz IMU 跑 Mahony 互补滤波得姿态 q(t)(chip->水平参考系)，
      逐包用 M(t) = Rz(yaw(q))·R(q) 的转置反旋转点云:
          p_raw = M(t)^T · p_leveled
      M 只含 roll/pitch(与偏航对易)，结果 = 原始点云差一个常量偏航(并入外参候选)。
验证: 反旋转后 t>60s 的地面倾角应≈34.5°(与 IMU 重力方向变化一致)。
"""
import os, glob, sys
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")

def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])

def mahony(t_ns, gyro, acc, kp=0.2, ki=0.01):
    n = len(t_ns)
    q = np.array([1.0, 0, 0, 0])
    # 初始对齐: 静止段重力 → 初始姿态(chip->level, 偏航置0)
    g0 = acc[:1000].mean(axis=0)
    g0 /= np.linalg.norm(g0)
    # 求 q0 使 R(q0)^T·(0,0,1) = g0  (即 R(q0)·g0 = (0,0,1)), 偏航任意取0
    axis = np.cross(g0, np.array([0, 0, 1.0]))
    s = np.linalg.norm(axis)
    c = float(np.dot(g0, [0, 0, 1.0]))
    if s < 1e-9:
        q = np.array([1.0, 0, 0, 0]) if c > 0 else np.array([0.0, 1, 0, 0])
    else:
        axis /= s
        ang = np.arctan2(s, c)
        q = np.array([np.cos(ang/2), *(np.sin(ang/2)*axis)])
    qs = np.empty((n, 4))
    eint = np.zeros(3)
    for i in range(n):
        if i > 0:
            dt = (t_ns[i] - t_ns[i-1]) / 1e9
            w = gyro[i].copy()
            # 重力反馈误差 (chip 系)
            R = quat_to_R(q)
            g_pred = R.T @ np.array([0, 0, 1.0])
            am = acc[i] / max(np.linalg.norm(acc[i]), 1e-9)
            e = np.cross(am, g_pred)
            eint += e * dt * ki
            w = w + kp * e + eint
            dq = 0.5 * quat_mul(q, np.array([0.0, *w]))
            q = quat_normalize(q + dq * dt)
        qs[i] = q
    return qs

def rp_only_R(q, W=None):
    """正确反旋转矩阵 M(t) = W·R(t)^T·Rz(ψ(t))
    R = Rz(ψ)·R_rp·W（chip→level，含初始楔角 W=R(0)）
    p_new = M·p_leveled = p_raw（差一个常量偏航，并入外参）"""
    R = quat_to_R(q)
    psi = np.arctan2(R[1, 0], R[0, 0])
    c, s = np.cos(psi), np.sin(psi)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return W @ R.T @ Rz

def main(verify_only=False):
    imu = np.load(os.path.join(OUT, 'imu.npz'))
    t_ns, gyro, acc = imu['t_ns'], imu['gyro'], imu['acc']
    print("running Mahony filter ...")
    qs = mahony(t_ns, gyro, acc)
    np.save(os.path.join(OUT, 'attitude_q.npy'), qs)
    # 验证: 滤波后 R(q)^T·(0,0,1) 应跟踪重力方向
    for tv in (0, 30, 50, 100, 200, 290):
        i = np.searchsorted(t_ns, t_ns[0] + tv*1e9)
        i = min(i, len(t_ns)-1)
        R = quat_to_R(qs[i])
        g_pred = R.T @ np.array([0, 0, 1.0])
        g_meas = acc[max(0,i-100):i+100].mean(axis=0)
        g_meas /= np.linalg.norm(g_meas)
        err = np.degrees(np.arccos(np.clip(g_pred @ g_meas, -1, 1)))
        print(f"  t={tv:3d}s: 姿态重力=({g_pred[0]:+.3f},{g_pred[1]:+.3f},{g_pred[2]:+.3f}) "
              f"实测=({g_meas[0]:+.3f},{g_meas[1]:+.3f},{g_meas[2]:+.3f}) 夹角={err:.2f}°")

    if verify_only:
        return
    # 预计算每个 IMU 时刻的反旋转矩阵 M(t) = W·R^T·Rz(ψ)，W = R(0)
    W = quat_to_R(qs[0])
    MT = np.array([rp_only_R(q, W) for q in qs])
    print(f"W (初始楔角/常量旋转) =\n{np.round(W, 4)}")
    print(f"M(0) 应为单位阵:\n{np.round(MT[0], 4)}")
    np.save(os.path.join(OUT, 'unlevel_MT.npy'), MT)
    np.save(os.path.join(OUT, 'wedge_W.npy'), W)

    # 逐帧反旋转（按包时间查最近 IMU 样本）
    files = sorted(glob.glob(os.path.join(OUT, 'lidar_frames', 'frame_*.npz')))
    print(f"un-leveling {len(files)} frames ...")
    for k, fn in enumerate(files):
        d = np.load(fn)
        xyz, t_off, ts0 = d['xyz'], d['t_off_ns'], int(d['ts0_ns'])
        t_pt = ts0 + t_off
        idx = np.searchsorted(t_ns, t_pt).clip(1, len(t_ns)-1)
        # 就近取样本
        left = t_ns[idx-1]; right = t_ns[idx]
        idx = np.where(np.abs(t_pt - left) < np.abs(t_pt - right), idx-1, idx)
        xyz_new = np.einsum('kij,kj->ki', MT[idx], xyz)
        np.savez_compressed(fn, xyz=xyz_new, attr=d['attr'], t_off_ns=t_off,
                            ts0_ns=np.int64(ts0), ts1_ns=d['ts1_ns'])
        if k % 300 == 0:
            print(f"  {k}/{len(files)}")
    print("done.")

if __name__ == '__main__':
    main(verify_only='--verify' in sys.argv)
