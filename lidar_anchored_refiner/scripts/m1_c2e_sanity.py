"""Scorer sanity check against vendor C2E poses (diagnostic oracle only).

If the vendor-painted cloud + vendor camera poses don't score high with my
edge metric, then the metric or my coordinate chain is broken. C2E quaternion
order/convention is also searched (w-first vs x-first, q vs conj).
Writes overlay PNGs for visual inspection.
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import paths, posdata
from m1_search import edge_score, load_context, pose_to_local, undistort_gray
from m1_search2 import all_euler_interpretations  # noqa: F401 (keep import graph simple)

WORK = paths.WORK_DIR
C2E = os.path.join(paths.PROJECT_DIR, "二楼室外_CameraPos_C2E.txt")


def quat_to_R(q):
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load_c2e():
    rows = []
    with open(C2E, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) == 11 and p[0].endswith(".JPG"):
                rows.append((p[0], int(p[1]), int(p[2]), float(p[3]),
                             *[float(x) for x in p[4:]]))
    return rows  # (fname, camid, week, sow, X,Y,Z, q1..q4)


def overlay_png(cloud_cam, K, shape, gray, out_path):
    z = cloud_cam[:, 2]
    ok = z > 0.3
    x, y, z = cloud_cam[ok].T
    u = (K[0] * x / z + K[2]).astype(np.int32)
    v = (K[1] * y / z + K[3]).astype(np.int32)
    m = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    u, v, z = u[m], v[m], z[m]
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    zn = np.clip((z - np.percentile(z, 5)) / max(np.percentile(z, 95) - np.percentile(z, 5), 1e-6), 0, 1)
    colors = (255 * (1 - zn))  # near=bright
    order = np.argsort(-z)  # draw far first
    vis[v[order], u[order]] = np.stack([colors[order]] * 3, axis=1).astype(np.uint8)
    cv2.imwrite(out_path, vis)


def main():
    ctx = load_context()
    rng = np.random.default_rng(0)
    cloud = ctx["cloud_xyz"][rng.choice(len(ctx["cloud_xyz"]), 600_000, replace=False)]
    rows = load_c2e()
    print("C2E rows:", len(rows))
    cams = {1: "R", 2: "M", 3: "L"}
    picked = {}
    for cid, cname in cams.items():
        cr = [r for r in rows if r[1] == cid]
        if not cr:
            print(f"{cname}: absent in C2E")
            continue
        picked[cname] = cr[len(cr) // 2]

    for cname, row in picked.items():
        fname, cid, week, sow, X, Y, Z, q1, q2, q3, q4 = row
        intr = ctx["intr"][cname]
        img_path = os.path.join(paths.image_dir(cname), fname)
        if not os.path.exists(img_path):
            print(f"{cname}: image missing {fname}")
            continue
        g = undistort_gray(img_path, intr)
        shape = (g.shape[0] // 4, g.shape[1] // 4)
        g_small = cv2.resize(g, (shape[1], shape[0]))
        K4 = [intr["fx"] / 4, intr["fy"] / 4, intr["cx"] / 4, intr["cy"] / 4]
        print(f"\n== {cname} {fname} sow={sow:.3f}")
        for label, q in [("wxyz", np.array([q1, q2, q3, q4])),
                         ("xyzw", np.array([q2, q3, q4, q1])),
                         ("wxyz-conj", np.array([q1, -q2, -q3, -q4])),
                         ("xyzw-conj", np.array([q2, -q3, -q4, q1]))]:
            R_c2e = quat_to_R(q)
            p_l, R_c2l = pose_to_local(ctx, np.array([X, Y, Z]), R_c2e)
            Xc = (R_c2l.T @ (cloud - p_l).T).T
            sc, frac = edge_score(Xc, K4, shape, g_small)
            print(f"  {label:10s} score={sc:.4f} inimg={frac:.3f}")
            if label == "wxyz":
                overlay_png(Xc, K4, shape, g_small,
                            os.path.join(WORK, f"overlay_c2e_{cname}.png"))


if __name__ == "__main__":
    main()
