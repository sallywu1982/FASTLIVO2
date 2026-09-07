"""Geometry helpers: euler conventions, quaternions, slerp, Umeyama similarity.

Everything here is written from scratch (numpy only). Rotation conventions are
treated as *hypotheses* that the pipeline verifies empirically.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def euler_xyz_deg_to_R(rx, ry, rz, order="ZXY") -> np.ndarray:
    """Euler angles in degrees -> rotation matrix.

    `order='ZXY'` means R = Rz(a0) @ Rx(a1) @ Ry(a2) with (a0,a1,a2) = the
    three input angles in the given order. The Para/pos files store (e0,e1,e2)
    listed as e.g. [82.2, -21.4, 21.8]; callers pass them in file order and we
    compose left-to-right according to `order`.
    """
    a = np.deg2rad([rx, ry, rz])
    Rz = Rotation.from_euler("z", a[0]).as_matrix()
    Rx = Rotation.from_euler("x", a[1]).as_matrix()
    Ry = Rotation.from_euler("y", a[2]).as_matrix()
    if order == "ZXY":
        return Rz @ Rx @ Ry
    if order == "ZYX":
        return Rz @ Ry @ Rx
    if order == "XYZ":
        return Rx @ Ry @ Rz
    if order == "YZX":
        return Ry @ Rz @ Rx
    raise ValueError(order)


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:  # shortest arc
        q1 = -q1
        d = -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = np.arccos(np.clip(d, -1.0, 1.0))
    s = np.sin(th0)
    return (np.sin((1 - t) * th0) / s) * q0 + (np.sin(t * th0) / s) * q1


def interp_pose(times, positions, quats, t_query):
    """Interpolate pose (position linear, rotation slerp) at t_query."""
    i = int(np.searchsorted(times, t_query))
    if i <= 0:
        return positions[0].copy(), quats[0].copy()
    if i >= len(times):
        return positions[-1].copy(), quats[-1].copy()
    t0, t1 = times[i - 1], times[i]
    a = (t_query - t0) / (t1 - t0)
    p = (1 - a) * positions[i - 1] + a * positions[i]
    q = quat_slerp(quats[i - 1], quats[i], a)
    return p, q


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """Least-squares similarity transform dst ≈ s·R@src + t.

    src, dst: (N,3). Returns (s, R, t, residuals(N,)).
    """
    assert src.shape == dst.shape
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = dc.T @ sc / src.shape[0]
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    var_s = (sc ** 2).sum() / src.shape[0]
    s = (np.trace(np.diag(D) @ S) / var_s) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    pred = (s * (R @ src.T)).T + t
    return s, R, t, np.linalg.norm(dst - pred, axis=1)


def so3_log(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> rotation vector (axis*angle)."""
    return Rotation.from_matrix(R).as_rotvec()


def rel_rotation_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Geodesic angle between two rotations."""
    return float(np.degrees(np.linalg.norm(so3_log(Ra.T @ Rb))))
