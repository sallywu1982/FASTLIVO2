"""M1 step 1: inspect the original LAS deliverable (CRS, ranges, RGB) and POS files."""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs200g import paths

import laspy


def inspect_las(path: str) -> None:
    print(f"== LAS: {os.path.basename(path)} ({os.path.getsize(path)/1e6:.1f} MB)")
    with laspy.open(path) as f:
        h = f.header
        print("  point_format:", h.point_format.id,
              "| num_points:", f"{h.point_count:,}",
              "| version:", h.version)
        print("  offsets:", h.offsets, "| scales:", h.scales)
        print("  mins:", np.round(h.mins, 3))
        print("  maxs:", np.round(h.maxs, 3))
        crs = h.parse_crs()
        print("  CRS:", crs)
        names = [n for n in h.point_format.dimension_names]
        print("  dims:", names)
        # sample first 200k points for RGB / return sanity
        with laspy.open(path) as f2:
            pts = next(f2.chunk_iterator(200_000))
            rgb_names = [n for n in ("red", "green", "blue") if n in names]
            if rgb_names:
                for n in rgb_names:
                    v = np.asarray(getattr(pts, n))
                    print(f"  {n}: min={v.min()} max={v.max()} mean={v.mean():.1f}")
            print("  gps_time present:", "gps_time" in names,
                  "| intensity mean:", float(np.asarray(pts.intensity).mean()))


def inspect_pos(path: str, label: str) -> None:
    print(f"== {label}: {os.path.basename(path)}")
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.split()
            if len(parts) == 11:
                rows.append([float(x) for x in parts])
    arr = np.array(rows)
    print("  rows:", len(rows), "| cols:", arr.shape[1])
    t = arr[:, 1]
    print(f"  week={arr[0,0]:.0f} t0={t[0]:.3f} t1={t[-1]:.3f} span={t[-1]-t[0]:.1f}s")
    print("  col3..5 min:", np.round(arr[:, 2:5].min(axis=0), 3))
    print("  col3..5 max:", np.round(arr[:, 2:5].max(axis=0), 3))
    dt = np.diff(t)
    print(f"  dt median={np.median(dt)*1000:.2f} ms  min={dt.min()*1000:.2f}  max={dt.max()*1000:.2f}")
    print("  velocity |v| median:", float(np.median(np.linalg.norm(arr[:, 5:8], axis=1))))
    print("  attitude cols (first/last):", np.round(arr[0, 8:11], 3), np.round(arr[-1, 8:11], 3))


if __name__ == "__main__":
    print("PROJECT_DIR:", paths.PROJECT_DIR)
    for cam in paths.CAMERAS:
        imgs = paths.list_images(cam)
        print(f"{cam}: {len(imgs)} images  {os.path.basename(imgs[0])} .. {os.path.basename(imgs[-1])}")
    inspect_pos(paths.POS_ECEF, "POS ECEF")
    inspect_pos(paths.POS_ENH, "POS ENH")
    inspect_las(paths.LAS_ORIGINAL)
