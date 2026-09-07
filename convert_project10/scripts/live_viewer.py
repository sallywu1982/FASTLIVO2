#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FAST-LIVO2 实时回放查看器（matplotlib TkAgg，纯软件渲染，不依赖 OpenGL）。

订阅 /cloud_registered（配准点云）与 /aft_mapped_to_init（里程计），
累积栅格去重后绘制：俯视地图(按高度着色) + 侧视 XZ + 高度剖面 + 轨迹。
每周期另存一张 latest.png 到 out/live/ 供外部查看。
用法（需先 source ROS 与 catkin 环境，DISPLAY 可用）:
  python3 live_viewer.py
"""
import os
import threading

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry

GRID = 0.25            # 地图栅格去重粒度 (m)
MAX_CELLS = 320_000    # 累积地图点上限
REDRAW_MS = 2500       # 重绘周期 (ms)
DS_MAX = 140_000       # 单次绘制点数上限（超出随机下采样）
OUT_PNG = "/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习/convert_project10/out/live/latest.png"

lock = threading.Lock()
map_cells = {}                      # (ix,iy,iz) -> [x,y,z]
traj = []                           # [t,x,y,z]
frame_cnt = [0]
cloud_cnt = [0]
last_cloud_t = [0.0]                # 用于检测循环回放导致的时间倒退


def reset_all():
    map_cells.clear()
    del traj[:]
    frame_cnt[0] = 0
    cloud_cnt[0] = 0


def on_cloud(msg):
    t = msg.header.stamp.to_sec()
    if last_cloud_t[0] > 0 and t < last_cloud_t[0] - 3.0:
        with lock:
            reset_all()
        rospy.logwarn("时间倒退 %.1f <- %.1f，重置累积（新循环）", t, last_cloud_t[0])
    last_cloud_t[0] = t
    try:
        raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = raw if isinstance(raw, np.ndarray) else np.array(list(raw), dtype=np.float64)
        if arr.dtype.names is not None:      # structured array
            x, y, z = arr["x"], arr["y"], arr["z"]
        elif arr.ndim == 1:                  # 一维平铺 x,y,z,x,y,z,...
            x, y, z = arr.reshape(-1, 3).T
        else:                                # NxM
            x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        z = np.asarray(z, dtype=np.float64)
    except Exception as e:
        rospy.logwarn("cloud parse: %s", e)
        return
    m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[m], y[m], z[m]
    if x.size == 0:
        return
    ix = np.round(x / GRID).astype(np.int64)
    iy = np.round(y / GRID).astype(np.int64)
    iz = np.round(z / GRID).astype(np.int64)
    cells = np.unique(np.stack([ix, iy, iz], axis=1), axis=0)
    added = 0
    with lock:
        frame_cnt[0] += 1
        for c in cells:
            k = (int(c[0]), int(c[1]), int(c[2]))
            if k not in map_cells:
                if len(map_cells) >= MAX_CELLS:
                    break
                map_cells[k] = (c[0] * GRID, c[1] * GRID, c[2] * GRID)
                added += 1
        cloud_cnt[0] = len(map_cells)


def on_odom(msg):
    p = msg.pose.pose.position
    t = msg.header.stamp.to_sec()
    with lock:
        traj.append((t, p.x, p.y, p.z))


def main():
    rospy.init_node("live_viewer", anonymous=True, disable_signals=True)
    rospy.Subscriber("/cloud_registered", PointCloud2, on_cloud, queue_size=2)
    rospy.Subscriber("/aft_mapped_to_init", Odometry, on_odom, queue_size=50)

    fig = plt.figure("FAST-LIVO2 live replay", figsize=(13.5, 7.5))
    ax1 = fig.add_axes([0.03, 0.05, 0.55, 0.88])   # 俯视 XY
    ax2 = fig.add_axes([0.66, 0.53, 0.31, 0.40])   # 侧视 XZ
    ax3 = fig.add_axes([0.66, 0.05, 0.31, 0.40])   # 高度剖面 z(t)
    state = {"n": 0}

    def redraw():
        with lock:
            pts = np.array(list(map_cells.values()), dtype=np.float64) if map_cells else np.zeros((0, 3))
            tr = np.array(traj, dtype=np.float64) if traj else np.zeros((0, 4))
            nframe = frame_cnt[0]
        npts = pts.shape[0]
        if npts > DS_MAX:
            keep = np.random.choice(npts, DS_MAX, replace=False)
            pts = pts[keep]

        for ax in (ax1, ax2, ax3):
            ax.clear()

        if npts:
            zmin, zmax = np.percentile(pts[:, 2], [2, 98])
            if zmax - zmin < 1:
                zmax = zmin + 1
            sc = ax1.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], s=0.4,
                             cmap="jet", vmin=zmin, vmax=zmax, linewidths=0)
            ax2.scatter(pts[:, 0], pts[:, 2], c=pts[:, 1], s=0.4,
                        cmap="jet", linewidths=0)
        ax1.set_title("top view (color=z)", fontsize=9)
        ax1.set_aspect("equal", adjustable="datalim")
        ax2.set_title("side view XZ", fontsize=9)

        if tr.shape[0]:
            ax1.plot(tr[:, 1], tr[:, 2], "r-", lw=1.2)
            ax1.plot(tr[-1, 1], tr[-1, 2], "g.", ms=9)
            ax2.plot(tr[:, 1], tr[:, 3], "r-", lw=1.2)
            ax3.plot(tr[:, 0] - tr[0, 0], tr[:, 3], "b-", lw=1)
            ax3.set_title("height vs t", fontsize=9)
            ax1.set_xlabel("x [m]", fontsize=8)
            ax1.set_ylabel("y [m]", fontsize=8)

        fig.suptitle("FAST-LIVO2 live  |  lidar frames %d  |  map %d pts (grid %.2fm)  |  traj %d"
                     % (nframe, npts, GRID, tr.shape[0]), fontsize=10)

        state["n"] += 1
        if state["n"] % 2 == 1:  # 每两个周期存一张
            try:
                os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
                fig.savefig(OUT_PNG, dpi=72)
            except Exception as e:
                rospy.logwarn_throttle(30, "save png: %s", e)

        fig.canvas.draw_idle()
        fig.canvas.get_tk_widget().after(REDRAW_MS, redraw)

    fig.canvas.get_tk_widget().after(1200, redraw)
    try:
        plt.show()
    finally:
        rospy.signal_shutdown("window closed")


if __name__ == "__main__":
    main()
