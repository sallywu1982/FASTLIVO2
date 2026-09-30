"""M1: camera pose computation with empirical convention search.

Nothing from the vendor C2E file or previous scripts is used as input. The
rotation conventions are determined by scoring candidate combinations against
the data: the correct convention makes depth edges of the LAS cloud coincide
with image edges.

Output (best convention): work/poses_{cam}.npz  (fname, sow, p_cam_ecef,
R_c2e, p_cam_local, q_c2local) and work/convention.json.
"""
from __future__ import annotations

import itertools
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import cvyaml, paths, posdata
from gs200g.geom import euler_xyz_deg_to_R

WORK = paths.WORK_DIR
ATT_ORDERS = ("ZXY", "ZYX", "XYZ", "YZX")
EXT_ORDERS = ("ZXY", "ZYX", "XYZ")
SCORE_FRAMES = 8
CLOUD_SUB = 600_000
EDGE_DILATE = 5


def load_context():
    pos = posdata.load_pos(paths.POS_ECEF)
    um = np.load(os.path.join(WORK, "ecef_to_proj.npz"))
    cloud = np.load(os.path.join(WORK, "cloud_local.npz"))
    lat0, lon0, _ = posdata.ecef_to_geodetic(*pos.xyz[0])
    ctx = {
        "pos": pos,
        "s_e2p": float(um["scale"]), "R_e2p": um["R"], "t_e2p": um["t"],
        "origin": um["origin_proj"],
        "cloud_xyz": cloud["xyz"],
        "Rl2e0": posdata.enu_rotation(lat0, lon0),  # site-fixed ENU->ECEF
        "intr": {c: cvyaml.read_intrinsic(paths.intrinsic_yaml(c)) for c in paths.CAMERAS},
        "extr": {c: cvyaml.read_extrinsic(paths.extrinsic_yaml(c)) for c in paths.CAMERAS},
        "frames": {c: posdata.load_images(c) for c in paths.CAMERAS},
    }
    return ctx


def pose_ecef(ctx, sow, cam, att_order, att_T, ext_order, ext_T):
    """Camera pose in ECEF at time `sow` under a candidate convention."""
    xyz_imu, att = ctx["pos"].interp(sow)
    R_b2l = euler_xyz_deg_to_R(att[0], att[1], att[2], att_order)
    if att_T:
        R_b2l = R_b2l.T
    R_b2e = ctx["Rl2e0"] @ R_b2l
    e = ctx["extr"][cam]["euler"]
    R_x = euler_xyz_deg_to_R(e[0], e[1], e[2], ext_order)
    R_c2b = R_x if ext_T else R_x.T
    R_c2e = R_b2e @ R_c2b
    t_ext = ctx["extr"][cam]["translation"]
    p_cam = xyz_imu + R_b2e @ t_ext
    return p_cam, R_c2e


def pose_to_local(ctx, p_cam_ecef, R_c2e):
    s, R_e2p, t_e2p, origin = ctx["s_e2p"], ctx["R_e2p"], ctx["t_e2p"], ctx["origin"]
    p_local = s * (R_e2p @ p_cam_ecef) + t_e2p - origin
    R_c2l = R_e2p @ R_c2e  # camera->local (proj axes == local axes)
    return p_local, R_c2l


def edge_score(cloud_cam, K, shape, gray_u):
    """Depth-edge vs image-edge normalized overlap in [0,1]."""
    z = cloud_cam[:, 2]
    ok = z > 0.3
    if ok.sum() < 500:
        return -1.0, 0.0
    x, y, z = cloud_cam[ok].T
    u = (K[0] * x / z + K[2]).astype(np.int32)
    v = (K[1] * y / z + K[3]).astype(np.int32)
    m = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    if m.sum() < 500:
        return -1.0, float(m.sum()) / ok.sum()
    u, v, z = u[m], v[m], z[m]
    depth = np.full(shape, np.inf, np.float32)
    # z-buffer via sort (np.minimum.at is prohibitively slow)
    pid = v.astype(np.int64) * shape[1] + u
    order = np.lexsort((z, pid))
    pid_s = pid[order]
    first = np.ones(len(pid_s), bool)
    first[1:] = pid_s[1:] != pid_s[:-1]
    sel = order[first]
    depth.ravel()[pid[sel]] = z[sel].astype(np.float32)
    valid = np.isfinite(depth)
    frac = float(valid.mean())
    d = np.where(valid, depth, 0).astype(np.float32)
    gx = cv2.Sobel(d, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(d, cv2.CV_32F, 0, 1, ksize=3)
    gmag = np.sqrt(gx * gx + gy * gy)
    thr = np.percentile(gmag[valid], 85) if valid.sum() > 100 else 1e9
    d_edge = cv2.dilate(((gmag > max(thr, 1e-6)) & valid).astype(np.uint8),
                        np.ones((EDGE_DILATE, EDGE_DILATE), np.uint8))
    i_edge = cv2.dilate(cv2.Canny(gray_u, 40, 120),
                        np.ones((EDGE_DILATE, EDGE_DILATE), np.uint8))
    inter = float((d_edge & i_edge).sum())
    denom = np.sqrt(float((d_edge.sum()) * float(i_edge.sum()))) or 1.0
    return inter / denom, frac


def imread_unicode(path, flags=cv2.IMREAD_GRAYSCALE):
    """cv2.imread cannot open non-ASCII paths on Windows; decode via bytes."""
    with open(path, "rb") as f:
        buf = np.frombuffer(f.read(), dtype=np.uint8)
    return cv2.imdecode(buf, flags)


def undistort_gray(path, intr):
    img = imread_unicode(path)
    K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], np.float64)
    dist = intr["dist"][:5].astype(np.float64)
    return cv2.undistort(img, K, dist, None)


def pick_frames(ctx):
    """SOBOL-ish even sampling within POS coverage, per camera."""
    t0, t1 = ctx["pos"].t0 + 2.0, ctx["pos"].t1 - 2.0
    out = {}
    for cam in paths.CAMERAS:
        frames = [f for f in ctx["frames"][cam] if t0 <= f[2] <= t1]
        if not frames:
            out[cam] = []
            continue
        idx = np.linspace(0, len(frames) - 1, SCORE_FRAMES).astype(int)
        out[cam] = [frames[i] for i in idx]
    return out


def main():
    ctx = load_context()
    rng = np.random.default_rng(0)
    sub = rng.choice(len(ctx["cloud_xyz"]), size=min(CLOUD_SUB, len(ctx["cloud_xyz"])),
                     replace=False)
    cloud = ctx["cloud_xyz"][sub]
    print(f"scoring cloud: {cloud.shape[0]:,} pts")

    sel = pick_frames(ctx)
    for cam in paths.CAMERAS:
        print(f"{cam}: {len(ctx['frames'][cam])} imgs, "
              f"{sum(1 for f in ctx['frames'][cam] if ctx['pos'].t0 <= f[2] <= ctx['pos'].t1)} in POS range, "
              f"{len(sel[cam])} scored")

    imgs_gray = {}
    for cam, fr in sel.items():
        for fname, _, sow in fr:
            p = os.path.join(paths.image_dir(cam), fname)
            g = undistort_gray(p, ctx["intr"][cam])
            imgs_gray[(cam, sow)] = g

    results = []
    for att_order, att_T, ext_order, ext_T in itertools.product(
            ATT_ORDERS, (False, True), EXT_ORDERS, (False, True)):
        scores, fracs = [], []
        for cam in paths.CAMERAS:
            intr = ctx["intr"][cam]
            for fname, _, sow in sel[cam]:
                g = imgs_gray[(cam, sow)]
                sc_shape = (g.shape[0] // 4, g.shape[1] // 4)
                g_small = cv2.resize(g, (sc_shape[1], sc_shape[0]))
                p_e, R_c2e = pose_ecef(ctx, sow, cam, att_order, att_T, ext_order, ext_T)
                p_l, R_c2l = pose_to_local(ctx, p_e, R_c2e)
                Xc = (R_c2l.T @ (cloud - p_l).T).T
                K4 = [intr["fx"] / 4, intr["fy"] / 4, intr["cx"] / 4, intr["cy"] / 4]
                s, frac = edge_score(Xc, K4, sc_shape, g_small)
                if s >= 0:
                    scores.append(s)
                    fracs.append(frac)
        if scores:
            results.append((float(np.median(scores)), float(np.median(fracs)),
                            att_order, att_T, ext_order, ext_T, len(scores)))
    results.sort(reverse=True)
    print("\ntop 12 convention candidates (score, inimg-frac, att, attT, ext, extT, n):")
    for r in results[:12]:
        print(f"  {r[0]:.4f}  {r[1]:.3f}  att={r[2]:3s} T={int(r[3])}  "
              f"ext={r[4]} T={int(r[5])}  n={r[6]}")

    best = results[0]
    if best[0] < 0.12:
        print("WARNING: best score low; conventions may all be wrong!")
    conv = {"att_order": best[2], "att_T": bool(best[3]),
            "ext_order": best[4], "ext_T": bool(best[5]),
            "score": best[0], "frac": best[1]}
    conv_path = os.path.realpath(os.path.join(WORK, "convention.json"))
    work_rp = os.path.realpath(WORK)
    if os.path.commonpath([conv_path, work_rp]) != work_rp:
        raise ValueError("output path escapes work dir: %s" % conv_path)
    Path(conv_path).write_text(json.dumps(conv, indent=2), encoding="utf-8")
    print("saved convention:", conv)


if __name__ == "__main__":
    main()
