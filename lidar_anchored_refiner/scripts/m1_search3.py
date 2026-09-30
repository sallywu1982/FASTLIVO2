"""M1 convention search v3: three-stage funnel.

Stage A  in-image fraction (pure projection, fast)  -> keep top ~200 pairs
Stage B  edge score on 2 frames x L,M               -> keep top ~24
Stage C  full: 3 cams x 4 frames x 2 time bases     -> final ranking
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import paths, posdata
from m1_search import edge_score, load_context, pose_to_local, undistort_gray
from m1_search2 import TIME_OFFSET, all_euler_interpretations, att_R_b2e, prune

WORK = paths.WORK_DIR


def project_frac(cloud, p_l, R_c2l, K4, shape):
    Xc = (R_c2l.T @ (cloud - p_l).T).T
    z = Xc[:, 2]
    ok = z > 0.3
    x, y = Xc[ok, 0] / z[ok], Xc[ok, 1] / z[ok]
    u = K4[0] * x + K4[2]
    v = K4[1] * y + K4[3]
    m = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    return float(m.sum()) / max(ok.sum(), 1)


def pose_for(ctx, a_fn, e_fn, cam, sow):
    xyz_imu, att = ctx["pos"].interp(sow)
    R_b2e = att_R_b2e(a_fn, att, ctx["Rl2e0"])
    R_c2e = R_b2e @ e_fn(ctx["extr"][cam]["euler"])
    p_cam = xyz_imu + R_b2e @ ctx["extr"][cam]["translation"]
    return pose_to_local(ctx, p_cam, R_c2e)


def main():
    ctx = load_context()
    print("stage 0: physics pruning ...", flush=True)
    survivors = prune(ctx)
    pairs = sorted({(a, e) for a, _, e, _, _ in survivors})
    print(f"survivor pairs: {len(pairs)}", flush=True)
    interps = dict(all_euler_interpretations())

    rng = np.random.default_rng(0)
    cloud_small = ctx["cloud_xyz"][rng.choice(len(ctx["cloud_xyz"]), 200_000, replace=False)]
    t0, t1 = ctx["pos"].t0, ctx["pos"].t1
    frames = {}
    for cam in paths.CAMERAS:
        fr = [f for f in ctx["frames"][cam] if t0 + 5 <= f[2] <= t1 - 5]
        frames[cam] = [fr[len(fr) // 2]]

    print("stage A: in-image fraction (L, mid frame) ...", flush=True)
    intrL = ctx["intr"]["L"]
    shape = (750, 1000)
    stageA = []
    sowL = frames["L"][0][2]
    for a_name, e_name in pairs:
        p_l, R_c2l = pose_for(ctx, interps[a_name], interps[e_name], "L", sowL)
        frac = project_frac(cloud_small, p_l, R_c2l,
                            [intrL["fx"] / 4, intrL["fy"] / 4, intrL["cx"] / 4, intrL["cy"] / 4], shape)
        stageA.append((frac, a_name, e_name))
    stageA.sort(reverse=True)
    topA = [x for x in stageA if x[0] > 0.15][:200]
    print(f"stage A kept {len(topA)}; best fracs: {[f'{x[0]:.2f}' for x in topA[:5]]}", flush=True)

    print("stage B: edge score (L+M, 2 frames each) ...", flush=True)
    sel_frames = {c: [] for c in paths.CAMERAS}
    for cam in paths.CAMERAS:
        fr = [f for f in ctx["frames"][cam] if t0 + 5 <= f[2] <= t1 - 5]
        sel_frames[cam] = [fr[len(fr) // 4], fr[3 * len(fr) // 4]]
    gray = {}
    for cam, fr in sel_frames.items():
        for fname, _, sow in fr:
            g = undistort_gray(os.path.join(paths.image_dir(cam), fname), ctx["intr"][cam])
            gray[(cam, sow)] = g
    cloud_mid = ctx["cloud_xyz"][rng.choice(len(ctx["cloud_xyz"]), 400_000, replace=False)]
    stageB = []
    for frac, a_name, e_name in topA:
        scores = []
        for cam in ("L", "M"):
            intr = ctx["intr"][cam]
            for fname, _, sow in sel_frames[cam]:
                g = gray[(cam, sow)]
                sh = (g.shape[0] // 4, g.shape[1] // 4)
                g_small = cv2.resize(g, (sh[1], sh[0]))
                p_l, R_c2l = pose_for(ctx, interps[a_name], interps[e_name], cam, sow)
                sc, _ = edge_score((R_c2l.T @ (cloud_mid - p_l).T).T,
                                   [intr["fx"] / 4, intr["fy"] / 4, intr["cx"] / 4, intr["cy"] / 4],
                                   sh, g_small)
                if sc >= 0:
                    scores.append(sc)
        stageB.append((float(np.median(scores)) if scores else -1, a_name, e_name))
    stageB.sort(reverse=True)
    topB = stageB[:24]
    print("stage B top 6:", [(f"{s:.3f}", a, e) for s, a, e in topB[:6]], flush=True)

    print("stage C: full scoring (3 cams x 4 frames x 2 time bases) ...", flush=True)
    framesC = {}
    for cam in paths.CAMERAS:
        fr = [f for f in ctx["frames"][cam] if t0 + 5 <= f[2] <= t1 - 5]
        framesC[cam] = [fr[i] for i in np.linspace(0, len(fr) - 1, 4).astype(int)]
    grayC = {}
    for cam, fr in framesC.items():
        for fname, _, sow in fr:
            if (cam, sow) not in gray:
                g = undistort_gray(os.path.join(paths.image_dir(cam), fname), ctx["intr"][cam])
                gray[(cam, sow)] = g
            grayC[(cam, sow)] = gray[(cam, sow)]
    results = []
    for sc_b, a_name, e_name in topB:
        for tb in (0.0, TIME_OFFSET):
            scores, fracs = [], []
            for cam in paths.CAMERAS:
                intr = ctx["intr"][cam]
                for fname, _, sow in framesC[cam]:
                    g = grayC[(cam, sow)]
                    sh = (g.shape[0] // 4, g.shape[1] // 4)
                    g_small = cv2.resize(g, (sh[1], sh[0]))
                    p_l, R_c2l = pose_for(ctx, interps[a_name], interps[e_name], cam, sow + tb)
                    sc, fr = edge_score((R_c2l.T @ (cloud_mid - p_l).T).T,
                                        [intr["fx"] / 4, intr["fy"] / 4, intr["cx"] / 4, intr["cy"] / 4],
                                        sh, g_small)
                    if sc >= 0:
                        scores.append(sc)
                        fracs.append(fr)
            results.append((float(np.median(scores)), float(np.median(fracs)), a_name, e_name, tb))
    results.sort(reverse=True)
    print("\nFINAL top 10 (score, inimg, att, ext, tb):")
    for r in results[:10]:
        print(f"  {r[0]:.4f}  {r[1]:.3f}  att={r[2]}  ext={r[3]}  tb={r[4]:.2f}")
    best = results[0]
    conv = {"att": best[2], "ext": best[3], "time_base_offset": best[4],
            "score": best[0], "frac": best[1]}
    conv2_path = os.path.realpath(os.path.join(WORK, "convention2.json"))
    work_rp = os.path.realpath(WORK)
    if os.path.commonpath([conv2_path, work_rp]) != work_rp:
        raise ValueError("output path escapes work dir: %s" % conv2_path)
    Path(conv2_path).write_text(json.dumps(conv, indent=2), encoding="utf-8")
    print("saved:", conv)


if __name__ == "__main__":
    main()
