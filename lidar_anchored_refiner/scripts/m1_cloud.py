"""M1: POS sanity checks, ECEF->projected Umeyama fit, and the voxel-downsampled
working point cloud (local frame, PLY + NPZ cache).

All checks are self-contained: nothing is taken from prior work.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import paths, posdata
from gs200g.geom import umeyama

import laspy

VOXEL = 0.05
CACHE_NPZ = os.path.join(paths.WORK_DIR, "cloud_local.npz")
CACHE_PLY = os.path.join(paths.WORK_DIR, "cloud_local.ply")
UMEMA_NPZ = os.path.join(paths.WORK_DIR, "ecef_to_proj.npz")


def check_pos_pair():
    pos_e = posdata.load_pos(paths.POS_ECEF)
    pos_p = posdata.load_pos(paths.POS_ENH)
    print(f"pos rows: ecef={len(pos_e.t)} enh={len(pos_p.t)}")
    dt = pos_e.t - pos_p.t
    print(f"time offset pos.txt - pos_ENH: mean={dt.mean():.4f}s std={dt.std():.6f}s "
          f"(min {dt.min():.4f} max {dt.max():.4f})")
    # velocity vs position derivative consistency (validates column semantics)
    dP = np.diff(pos_e.xyz, axis=0)
    dtev = np.diff(pos_e.t)
    v_est = dP / dtev[:, None]
    mask = (dtev > 0.001) & (dtev < 0.02)
    err = np.linalg.norm(v_est[mask] - pos_e.vel[:-1][mask], axis=1)
    print(f"pos.txt velocity-vs-dP/dt: median|err|={np.median(err)*1000:.2f} mm/s "
          f"p95={np.percentile(err,95)*1000:.2f} mm/s -> cols 6:8 are ECEF velocity: {np.median(err)<0.05}")
    return pos_e, pos_p


def fit_ecef_to_proj(pos_e, pos_p):
    idx = np.linspace(0, len(pos_e.t) - 1, 2000).astype(int)
    s, R, t, res = umeyama(pos_e.xyz[idx], pos_p.xyz[idx], with_scale=True)
    print(f"Umeyama ECEF->proj: scale={s:.9f} (1-|s|={abs(1-s)*1e6:.2f} ppm) "
          f"residual median={np.median(res)*1000:.2f} mm p95={np.percentile(res,95)*1000:.2f} mm")
    np.savez(UMEMA_NPZ, scale=s, R=R, t=t,
             origin_ecef=pos_e.xyz[0], origin_proj=pos_p.xyz[0])
    return s, R, t


def build_cloud(s, R_e2p, t_e2p):
    origin = None
    if os.path.exists(CACHE_NPZ):
        d = np.load(CACHE_NPZ)
        print("cloud cache found:", d["xyz"].shape[0], "points")
        return d["xyz"], d["rgb"], float(d["origin_proj"][0]), float(d["origin_proj"][1]), float(d["origin_proj"][2])
    # local origin: first POS projected position (constant, documented)
    d0 = np.load(UMEMA_NPZ)
    origin = d0["origin_proj"]
    keys_all = []
    pts_all = []
    rgb_all = []
    n_in = 0
    with laspy.open(paths.LAS_ORIGINAL) as f:
        for chunk in f.chunk_iterator(2_000_000):
            x = np.asarray(chunk.x, dtype=np.float64)
            y = np.asarray(chunk.y, dtype=np.float64)
            z = np.asarray(chunk.z, dtype=np.float64)
            r = np.asarray(chunk.red, dtype=np.uint8)[..., None]
            g = np.asarray(chunk.green, dtype=np.uint8)[..., None]
            b = np.asarray(chunk.blue, dtype=np.uint8)[..., None]
            n_in += x.size
            xyz = np.stack([x, y, z], axis=1)
            xyz_l = xyz - origin
            ix = np.floor(xyz_l[:, 0] / VOXEL).astype(np.int64)
            iy = np.floor(xyz_l[:, 1] / VOXEL).astype(np.int64)
            iz = np.floor(xyz_l[:, 2] / VOXEL).astype(np.int64)
            key = (ix << 42) | (iy << 21) | iz
            _, first = np.unique(key, return_index=True)
            keep = np.sort(first)
            keys_all.append(key[keep])
            pts_all.append(xyz_l[keep].astype(np.float32))
            rgb_all.append(np.concatenate([r[keep], g[keep], b[keep]], axis=1))
    keys = np.concatenate(keys_all)
    pts = np.concatenate(pts_all)
    rgb = np.concatenate(rgb_all)
    _, first = np.unique(keys, return_index=True)
    keep = np.sort(first)
    xyz = pts[keep]
    rgb = rgb[keep]
    print(f"LAS points in={n_in:,} -> voxel{VOXEL}m unique={xyz.shape[0]:,}")
    np.savez_compressed(CACHE_NPZ, xyz=xyz, rgb=rgb, origin_proj=origin, voxel=VOXEL)
    write_ply(CACHE_PLY, xyz, rgb)
    return xyz, rgb, *[float(v) for v in origin]


def write_ply(path, xyz, rgb):
    n = xyz.shape[0]
    with open(path, "wb") as f:
        hdr = (
            "ply\nformat binary_little_endian 1.0\n"
            f"element vertex {n}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n"
        )
        f.write(hdr.encode("ascii"))
        rec = np.zeros(n, dtype=[("xyz", "<f4", 3), ("rgb", "u1", 3)])
        rec["xyz"] = xyz
        rec["rgb"] = rgb
        f.write(rec.tobytes())
    print("wrote", path, f"({os.path.getsize(path)/1e6:.1f} MB)")


if __name__ == "__main__":
    pos_e, pos_p = check_pos_pair()
    s, R, t = fit_ecef_to_proj(pos_e, pos_p)
    xyz, rgb, ox, oy, oz = build_cloud(s, R, t)
    print(f"cloud extent (local m): min={xyz.min(axis=0).round(1)} max={xyz.max(axis=0).round(1)}")
