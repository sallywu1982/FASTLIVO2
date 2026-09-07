"""Loaders for GS-200G POS / image lists / geodesy helpers (all self-written)."""
from __future__ import annotations

import os
import re

import numpy as np

from . import paths

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)


def ecef_to_geodetic(x, y, z):
    """Bowring's method. Returns lat, lon (rad), h (m)."""
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1 - _E2))
    for _ in range(6):
        N = _A / np.sqrt(1 - _E2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - _E2 * N / (N + h)))
    return lat, lon, h


def enu_rotation(lat, lon):
    """Rotation ENU->ECEF columns are east,north,up: p_ecef = R @ p_enu."""
    sl, cl = np.sin(lon), np.cos(lon)
    sp, cp = np.sin(lat), np.cos(lat)
    return np.array([
        [-sl, -sp * cl, cp * cl],
        [cl, -sp * sl, cp * sl],
        [0.0, cp, sp],
    ])


class PosData:
    """pos.txt: week, sow, X, Y, Z, vx, vy, vz, r, p, y (11 cols)."""

    def __init__(self, week, t, xyz, vel, att):
        self.week = week
        self.t = t
        self.xyz = xyz
        self.vel = vel
        self.att = att  # degrees, (N,3)

    @property
    def t0(self):
        return self.t[0]

    @property
    def t1(self):
        return self.t[-1]

    def interp(self, tq):
        """Linear position + attitude-as-euler interpolation (angles are smooth,
        far from gimbal lock here; slerp refinement happens on quaternions of
        the composed rotation later)."""
        i = int(np.searchsorted(self.t, tq))
        i = min(max(i, 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        a = float(np.clip((tq - t0) / (t1 - t0), 0.0, 1.0))
        xyz = (1 - a) * self.xyz[i - 1] + a * self.xyz[i]
        att = (1 - a) * self.att[i - 1] + a * self.att[i]
        return xyz, att


def load_pos(path: str) -> PosData:
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) == 11:
                rows.append([float(x) for x in p])
    arr = np.asarray(rows)
    return PosData(arr[:, 0], arr[:, 1], arr[:, 2:5], arr[:, 5:8], arr[:, 8:11])


_IMG_RE = re.compile(r"^(\d{4})-(\d{5})-(\d{3})_(\d{5})-([LMR])\.JPG$")


def image_time_and_cam(fname: str):
    """'2430-04780-611_00000-L.JPG' -> (week=2430, sow=4780.611, cam='L')."""
    m = _IMG_RE.match(fname)
    if not m:
        return None
    week = int(m.group(1))
    sow = int(m.group(2) + m.group(3)) / 1000.0
    return week, sow, m.group(5)


def load_images(cam: str):
    """Returns list of (fname, week, sow) sorted by sow."""
    out = []
    for f in paths.list_images(cam):
        r = image_time_and_cam(os.path.basename(f))
        if r:
            out.append((os.path.basename(f), r[0], r[1]))
    out.sort(key=lambda x: x[2])
    return out
