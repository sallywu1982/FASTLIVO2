# -*- coding: utf-8 -*-
"""
decode_lid.py — 公司 .lid 雷达文件解码器
格式（逆向确认）:
  文件 = [1717B 参数头(含全部标定)] + [数据区]
  数据区 = 预滚杂项 + 连续 1380B 子包
  子包(1380B) = [4B 魔数 XX 00 01 02 (XX=计数器)]
                + [16B 保留(全零)]
                + [8B u64 时间戳 epoch-ns]
                + [1348B 点记录区: 14B/点, 3×i32(mm) + u16(属性), 零填充]
                + [4B 尾(低u16=0x6012, 高u16=包序)]
  200 子包 = 1 帧 (~96ms, ~12000 点)
输出: out/lidar_frames/frame_XXXXX.npz  (xyz_m, attr, t_off_ns, ts_ns)
"""
import os, sys, glob
import numpy as np

REC = np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'), ('a', '<u2')])  # 14B
PKT = 1380
PKT_BODY = 28            # 魔数4+保留16+时间戳8
PKT_TAIL = 4
BODY = PKT - PKT_BODY - PKT_TAIL   # 1348
FRAME_PKTS = 200
HDR = 0x6B5

DATA_DIR = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Lidar"
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out", "lidar_frames")
os.makedirs(OUT_DIR, exist_ok=True)


def _pkt_ok(raw, off):
    """检查 off 处是否为合法子包：魔数 + 合理 epoch-ns 时间戳"""
    if off + 28 > len(raw):
        return False, None
    if raw[off+1:off+4] != b'\x00\x01\x02':
        return False, None
    ts = int.from_bytes(raw[off+20:off+28], 'little')
    if not (1700000000000000000 < ts < 1850000000000000000):
        return False, None
    return True, ts


def find_first_packet(raw, verify=8):
    """在数据区前 4KB 内找第一个子包：要求后续 verify 个包也合法且时间戳递增"""
    for off in range(0, min(len(raw), 4096)):
        ok, ts0 = _pkt_ok(raw, off)
        if not ok:
            continue
        good = True
        prev = ts0
        for k in range(1, verify + 1):
            ok, ts = _pkt_ok(raw, off + k * PKT)
            if not ok or ts <= prev or ts - prev > 10_000_000:  # 单步 ≤10ms
                good = False
                break
            prev = ts
        if good:
            return off
    return -1


def decode_file(path, frame_offset=0, max_frames=None):
    raw = open(path, 'rb').read()
    start = find_first_packet(raw)
    if start < 0:
        print(f"!! no packet found in {path}")
        return 0
    # 计算包数（真实总数用于返回，max_frames 只限制实际解码数量）
    npkts = (len(raw) - start) // PKT
    nframes_total = npkts // FRAME_PKTS
    nframes = nframes_total if not max_frames else min(nframes_total, max_frames)
    print(f"{os.path.basename(path)}: data={len(raw)}B, first_pkt@{start}, pkts={npkts}, frames={nframes_total}" + (f" (decode {nframes})" if max_frames else ""))

    stats = []
    for fi in range(nframes):
        pts = []
        tss = []
        for k in range(FRAME_PKTS):
            off = start + (fi*FRAME_PKTS + k)*PKT
            ts = int.from_bytes(raw[off+20:off+28], 'little')
            body = raw[off+PKT_BODY: off+PKT-PKT_TAIL]
            recs = np.frombuffer(body[:len(body)//14*14], dtype=REC)
            nz = recs[np.any(recs.view('<u2').reshape(-1, 7) != 0, axis=1)] if len(recs) else recs
            if len(nz):
                pts.append(nz)
                tss.append((ts, len(nz)))
        if not pts:
            continue
        rec = np.concatenate(pts)
        ts0 = tss[0][0]
        ts1 = tss[-1][0] + 480000
        # 每点时间 = 所在包 ts + 包内序号线性插值（近似）
        t_off = np.empty(len(rec), dtype=np.int64)
        p0 = 0
        for ts, n in tss:
            t_off[p0:p0+n] = ts - ts0 + (np.arange(n) * 480000 // max(n, 1))
            p0 += n
        xyz_m = np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float64) / 1000.0
        # 过滤明显无效点（坐标为0且attr为0 的已除；距离过近/过远的在应用端处理）
        fn = os.path.join(OUT_DIR, f"frame_{frame_offset+fi:05d}.npz")
        np.savez_compressed(fn, xyz=xyz_m, attr=rec['a'].astype(np.float64),
                            t_off_ns=t_off, ts0_ns=np.int64(ts0), ts1_ns=np.int64(ts1))
        if fi % 100 == 0 or fi == nframes-1:
            d = np.linalg.norm(xyz_m, axis=1)
            print(f"  frame {frame_offset+fi}: {len(rec)} pts, dist[5,50,95]%="
                  f"{np.percentile(d,[5,50,95]).round(2)}, t={ts0/1e9:.3f}")
        stats.append((frame_offset+fi, len(rec), ts0, ts1))
    return nframes_total


if __name__ == '__main__':
    max_frames = int(sys.argv[1]) if len(sys.argv) > 1 else None
    files = sorted(glob.glob(os.path.join(DATA_DIR, "*.lid")))
    total = 0
    for p in files:
        n = decode_file(p, frame_offset=total, max_frames=max_frames)
        total += n
    print(f"\nTOTAL frames decoded: {total}")
