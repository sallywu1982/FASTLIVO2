# -*- coding: utf-8 -*-
"""把 FAST-LIVO2 增量保存的 PCD 分片合并成单个二进制 PCD，便于一次性加载查看。

所有读写路径均限制在 convert_project10/out/ 目录内（realpath 白名单校验）。
"""
import glob
import os
import re
import sys
from pathlib import Path

BASE = Path(r"E:\StoneRecord\ws-Work\06_FAST-LIVO2学习\convert_project10")
ALLOWED_ROOT = (BASE / "out").resolve()


def safe_path(p):
    rp = Path(p).resolve()
    if not str(rp).lower().startswith(str(ALLOWED_ROOT).lower() + "\\"):
        raise RuntimeError("path escapes allowed dir: " + str(rp))
    return rp


def main():
    pcd_dir = BASE / "out" / "pcd"
    out_path = BASE / "out" / "map_all.pcd"
    files = sorted(pcd_dir.glob("*.pcd"))
    files = [safe_path(f) for f in files]
    if not files:
        print("no pcd shards in", pcd_dir)
        sys.exit(1)
    print(f"{len(files)} shards")

    header = None
    total = 0
    pts_check = []
    chunks = []
    for f in files:
        data = f.read_bytes()
        idx = data.find(b"DATA binary")
        if idx < 0:
            raise RuntimeError("not a binary pcd: " + str(f))
        hdr_end = data.find(b"\n", idx) + 1
        hdr_txt = data[:hdr_end].decode("ascii")

        h = {}
        for line in hdr_txt.splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                k, v = line.split(None, 1)
                h[k] = v
        n = int(h.get("POINTS", h.get("WIDTH", "0")))
        pt_size = sum(int(s) for s in h["SIZE"].split())
        assert len(data) - hdr_end == n * pt_size, (f, len(data) - hdr_end, n, pt_size)

        if header is None:
            header = hdr_txt
        total += n
        pts_check.append(n)
        chunks.append(data[hdr_end:])

    hdr_new = re.sub(r"(?m)^WIDTH\s+\d+", f"WIDTH {total}", header)
    hdr_new = re.sub(r"(?m)^HEIGHT\s+\d+", "HEIGHT 1", hdr_new)
    hdr_new = re.sub(r"(?m)^POINTS\s+\d+", f"POINTS {total}", hdr_new)
    body = b"".join(chunks)

    safe_path(out_path).write_bytes(hdr_new.encode("ascii") + body)

    import numpy as np
    arr = np.frombuffer(body, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]))
    assert arr.shape[0] == total
    print(f"merged {len(files)} shards -> {out_path}")
    print(f"total {total} points ({len(body)/1e6:.1f} MB)")
    print(f"per-shard points: min {min(pts_check)}, max {max(pts_check)}")
    print(f"bounds x[{arr['x'].min():.1f},{arr['x'].max():.1f}] "
          f"y[{arr['y'].min():.1f},{arr['y'].max():.1f}] "
          f"z[{arr['z'].min():.1f},{arr['z'].max():.1f}]")


if __name__ == "__main__":
    main()
