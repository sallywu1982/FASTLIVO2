"""M1 convention search v2: full angle-axis assignment space with physics pruning.

v1 failed because it only permuted composition order, not WHICH file angle
rotates around WHICH axis. v2 enumerates all 72 interpretations of a 3-angle
euler file (6 axis-assignments x 6 composition orders x 2 transposes) for both
the attitude and the extrinsic, prunes with pure-geometry constraints (optical
axis roughly horizontal, pointing into the site bbox), then edge-scores the
survivors. The image time base (+46.2409 s or not) is also searched.
"""
from __future__ import annotations

import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import paths, posdata
from m1_search import (CLOUD_SUB, edge_score, load_context, pose_to_local,
                       undistort_gray, imread_unicode)
from scipy.spatial.transform import Rotation

WORK = paths.WORK_DIR
TIME_OFFSET = 46.2409  # pos.txt - pos_ENH constant offset


def all_euler_interpretations():
    """Yield (name, fn) where fn(angles_deg[3]) -> R (3x3)."""
    perms = list(itertools.permutations(range(3)))
    out = []
    for axes in itertools.permutations("xyz"):
        for order in perms:
            for T in (False, True):
                def fn(a, axes=axes, order=order, T=T):
                    # R = R_{axes[order[0]]}(a[order[0]]) @ R_{axes[order[1]]}(a[order[1]]) @ R_{axes[order[2]]}(a[order[2]])
                    Rs = [Rotation.from_euler(axes[k], np.deg2rad(a[order[k]])).as_matrix()
                          for k in range(3)]
                    R = Rs[0] @ Rs[1] @ Rs[2]
                    return R.T if T else R
                name = f"{''.join(axes)}|{''.join(map(str, order))}|{'T' if T else 'N'}"
                out.append((name, fn))
    return out


def att_R_b2e(fn, att_row, Rl2e):
    return Rl2e @ fn(att_row)


def prune(ctx, n_samples=400):
    """Physics pruning over 72x72 interpretation pairs."""
    pos = ctx["pos"]
    idx = np.linspace(0, len(pos.t) - 1, n_samples).astype(int)
    att = pos.att[idx]
    xyz = pos.xyz[idx]
    # site bbox in ECEF: map local cloud corners back via inverse umeyama
    um = np.load(os.path.join(WORK, "ecef_to_proj.npz"))
    s, R_e2p, t_e2p, origin = um["scale"], um["R"], um["t"], um["origin_proj"]
    cloud = ctx["cloud_xyz"]
    site_center_local = (cloud.min(axis=0) + cloud.max(axis=0)) / 2
    site_center_proj = site_center_local + origin
    site_center_ecef = ((site_center_proj - t_e2p) / s) @ R_e2p  # R^T applied rowwise
    interps = all_euler_interpretations()  # 72
    ext_angles = {c: ctx["extr"][c]["euler"] for c in paths.CAMERAS}
    ext_fns = interps
    survivors = []
    cam_axis = np.array([0.0, 0.0, 1.0])
    for a_name, a_fn in interps:
        # R_b2e per sample for this attitude interpretation
        Rb2e = np.stack([att_R_b2e(a_fn, att[i], ctx["Rl2e0"]) for i in range(n_samples)])
        for cam in paths.CAMERAS:
            for e_name, e_fn in ext_fns:
                R_b2c = e_fn(ext_angles[cam])
                n_b = R_b2c @ cam_axis  # camera +Z in body frame
                ax_w = Rb2e @ n_b  # camera +Z in ECEF
                elev = np.degrees(np.arcsin(np.clip(ax_w[:, 2] / np.linalg.norm(ax_w, axis=1), -1, 1)))
                to_site = site_center_ecef - xyz
                to_site /= np.linalg.norm(to_site, axis=1, keepdims=True)
                cos_site = np.einsum("ij,ij->i", ax_w, to_site)
                if np.median(np.abs(elev)) < 35.0 and np.median(cos_site) > 0.15:
                    survivors.append((a_name, cam, e_name, float(np.median(np.abs(elev))), float(np.median(cos_site))))
    return survivors


def score_survivors(ctx, survivors, time_bases=(0.0, TIME_OFFSET)):
    """Edge-score each surviving (att, cam, ext, time_base) on real frames."""
    rng = np.random.default_rng(0)
    sub = rng.choice(len(ctx["cloud_xyz"]), size=min(CLOUD_SUB, len(ctx["cloud_xyz"])), replace=False)
    cloud = ctx["cloud_xyz"][sub]
    interps = dict(all_euler_interpretations())
    # frames: 6 per camera spread over coverage
    t0, t1 = ctx["pos"].t0 + 2.0, ctx["pos"].t1 - 2.0
    frames = {}
    for cam in paths.CAMERAS:
        fr = [f for f in ctx["frames"][cam] if t0 + 5 <= f[2] <= t1]
        sel = [fr[i] for i in np.linspace(0, len(fr) - 1, 4).astype(int)]
        frames[cam] = sel
    gray = {}
    for cam, fr in frames.items():
        for fname, _, sow in fr:
            gray[(cam, sow)] = undistort_gray(os.path.join(paths.image_dir(cam), fname), ctx["intr"][cam])

    # group survivors by (att, ext, time_base): score across all cameras jointly
    groups = {}
    for a_name, cam, e_name, _, _ in survivors:
        for tb in time_bases:
            groups.setdefault((a_name, e_name, tb), []).append(cam)

    results = []
    print(f"scoring {len(groups)} groups ...")
    for gi, ((a_name, e_name, tb), cams) in enumerate(sorted(groups.items())):
        if len(set(cams)) < 3:
            continue  # convention must work for ALL cameras simultaneously
        a_fn, e_fn = interps[a_name], interps[e_name]
        scores, fracs = [], []
        for cam in paths.CAMERAS:
            intr = ctx["intr"][cam]
            for fname, _, sow in frames[cam]:
                g = gray[(cam, sow)]
                shape = (g.shape[0] // 4, g.shape[1] // 4)
                g_small = cv2.resize(g, (shape[1], shape[0]))
                xyz_imu, att = ctx["pos"].interp(sow + tb)
                R_b2e = att_R_b2e(a_fn, att, ctx["Rl2e0"])
                R_c2b = e_fn(ctx["extr"][cam]["euler"])
                R_c2e = R_b2e @ R_c2b
                p_cam = xyz_imu + R_b2e @ ctx["extr"][cam]["translation"]
                p_l, R_c2l = pose_to_local(ctx, p_cam, R_c2e)
                Xc = (R_c2l.T @ (cloud - p_l).T).T
                K4 = [intr["fx"] / 4, intr["fy"] / 4, intr["cx"] / 4, intr["cy"] / 4]
                sc, frac = edge_score(Xc, K4, shape, g_small)
                if sc >= 0:
                    scores.append(sc)
                    fracs.append(frac)
        if scores:
            results.append((float(np.median(scores)), float(np.median(fracs)), a_name, e_name, tb, len(scores)))
        if gi % 20 == 0:
            print(f"  group {gi}/{len(groups)} att={a_name} ext={e_name} tb={tb:.1f}", flush=True)
    results.sort(reverse=True)
    return results


def main():
    ctx = load_context()
    print("pruning 72x72 interpretations x 3 cameras ...")
    survivors = prune(ctx)
    print(f"survivors: {len(survivors)}")
    for s in survivors[:20]:
        print("   ", s)
    results = score_survivors(ctx, survivors)
    print("\ntop 15 after edge scoring (score, inimg, att, ext, time_base, n):")
    for r in results[:15]:
        print(f"  {r[0]:.4f}  {r[1]:.3f}  att={r[2]}  ext={r[3]}  tb={r[4]:.2f}  n={r[5]}")
    if results:
        best = results[0]
        conv = {"att": best[2], "ext": best[3], "time_base_offset": best[4],
                "score": best[0], "frac": best[1]}
        with open(os.path.join(WORK, "convention2.json"), "w", encoding="utf-8") as f:
            json.dump(conv, f, indent=2)
        print("saved:", conv)


if __name__ == "__main__":
    import cv2
    main()
