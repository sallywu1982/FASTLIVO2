"""Minimal reader for the OpenCV FileStorage YAML flavour used by GS-200G Para files.

Only what the Para yamls contain is supported: `!!opencv-matrix` blocks with
rows/cols/dt/data. Self-contained on purpose (no OpenCV dependency for parsing).
"""
from __future__ import annotations

import re

import numpy as np

_MAT_RE = re.compile(
    r"^(\w+):\s*!!opencv-matrix\s*\n(.*?)(?=^\w+:|\Z)", re.S | re.M
)


def _parse_matrix(body: str) -> np.ndarray:
    rows = int(re.search(r"rows:\s*(\d+)", body).group(1))
    cols = int(re.search(r"cols:\s*(\d+)", body).group(1))
    data_m = re.search(r"data:\s*\[(.*?)\]", body, re.S)
    if not data_m:
        raise ValueError("opencv-matrix without data block")
    nums = [float(x) for x in data_m.group(1).replace("\n", " ").split(",") if x.strip()]
    arr = np.array(nums, dtype=np.float64)
    if arr.size != rows * cols:
        raise ValueError(f"matrix size mismatch: {rows}x{cols} vs {arr.size}")
    return arr.reshape(rows, cols)


def read_opencv_yaml(path: str) -> dict[str, np.ndarray]:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    out: dict[str, np.ndarray] = {}
    for m in _MAT_RE.finditer(text):
        out[m.group(1)] = _parse_matrix(m.group(2))
    return out


def read_intrinsic(path: str) -> dict:
    d = read_opencv_yaml(path)
    k = d["camera_matrix"]
    dist = d["distortion_coefficient"].ravel()
    return {
        "fx": float(k[0, 0]), "fy": float(k[1, 1]),
        "cx": float(k[0, 2]), "cy": float(k[1, 2]),
        "dist": dist,  # 4 params: k1,k2,p1,p2  |  5 params: k1,k2,p1,p2,k3
    }


def read_extrinsic(path: str) -> dict:
    d = read_opencv_yaml(path)
    return {
        "euler": d["euler"].ravel(),      # ZXY order per Para convention (to be verified)
        "translation": d["translation"].ravel(),
    }
