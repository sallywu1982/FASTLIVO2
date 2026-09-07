# -*- coding: utf-8 -*-
"""
make_bag.py — 把解码产物打包成 ROS1 rosbag（纯 Python rosbags，无需安装 ROS）
话题:
  /livox/lidar          livox_ros_driver/CustomMsg   (3004 帧, 10.4Hz)
  /livox/imu            sensor_msgs/Imu              (55559 条, 200Hz, acc 单位转 m/s^2)
  /left_camera/image/compressed  sensor_msgs/CompressedImage (M 相机 3327 张原始 JPEG)
时间基准: epoch-ns (Unix UTC)；相机 GPS 周内秒 -> epoch = week_start + SOW - 18s(闰秒)
"""
import os, re, glob, struct, sys
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(BASE, "out")
IMG_DIR = r"D:\Image2ImageTestData_20260813\Project_job10-08180901\Image\M"
BAG = os.path.join(OUT, "project10.bag")
WEEK_START = 1786838400          # 2026-08-16 00:00:00 UTC (GPS 周起始, 周 2432)
LEAP = 18                        # GPS-UTC 闰秒
G = 9.80665
BLIND = 0.3                      # 剔除过近点 (m)

from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ts = get_typestore(Stores.ROS1_NOETIC)
CP_DEF = "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n"
CM_DEF = "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n"
ts.register(get_types_from_msg(CP_DEF, 'livox_ros_driver/msg/CustomPoint'))
ts.register(get_types_from_msg(CM_DEF, 'livox_ros_driver/msg/CustomMsg'))

PT_DTYPE = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                     ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])


def custommsg_bytes(timebase_ns, pts):
    """手工序列化 livox CustomMsg（与 rosbags typestore 字节布局完全一致, 性能: numpy 打包 30M 点）"""
    sec, nsec = divmod(timebase_ns, 1_000_000_000)
    frame = b'livox'
    head = struct.pack('<III', 0, sec, nsec) + struct.pack('<I', len(frame)) + frame
    head += struct.pack('<QIB', timebase_ns, len(pts), 0) + b'\x00\x00\x00'
    head += struct.pack('<I', len(pts))
    return head + pts.tobytes()


def load_lidar_events():
    evs = []
    for fn in sorted(glob.glob(os.path.join(OUT, 'lidar_frames', 'frame_*.npz'))):
        d = np.load(fn)
        xyz, attr, t_off, ts0 = d['xyz'], d['attr'], d['t_off_ns'], int(d['ts0_ns'])
        r = np.linalg.norm(xyz, axis=1)
        m = (r > BLIND) & (r < 80.0)
        pts = np.empty(m.sum(), dtype=PT_DTYPE)
        pts['offset_time'] = np.clip(t_off[m], 0, 4_294_000_000).astype('<u4')
        pts['x'], pts['y'], pts['z'] = xyz[m].T
        pts['reflectivity'] = np.clip(attr[m], 0, 255).astype('u1')
        pts['tag'] = 0
        pts['line'] = 0
        evs.append((ts0, 'lidar', pts))
    return evs


def load_imu_events():
    d = np.load(os.path.join(OUT, 'imu.npz'))
    t, gyro, acc = d['t_ns'], d['gyro'], d['acc']
    # 去重 + 排序
    _, idx = np.unique(t, return_index=True)
    t, gyro, acc = t[idx], gyro[idx], acc[idx]
    return [(int(t[i]), 'imu', (gyro[i], acc[i] * G)) for i in range(len(t))]


def load_img_events():
    evs = []
    pat = re.compile(r'^\d+-(\d+)-(\d+)_(\d+)-M\.JPG$', re.I)
    for p in glob.glob(os.path.join(IMG_DIR, '**', '*.JPG'), recursive=True):
        m = pat.match(os.path.basename(p))
        if not m:
            continue
        sow = int(m.group(1)) + int(m.group(2)) / 1000.0
        t_ns = int(round((WEEK_START + sow - LEAP) * 1e9))
        evs.append((t_ns, 'img', p))
    evs.sort(key=lambda e: e[0])
    return evs


def main():
    print("loading events ...")
    evs = load_lidar_events() + load_imu_events() + load_img_events()
    evs.sort(key=lambda e: e[0])
    print(f"events: {sum(1 for e in evs if e[1]=='lidar')} lidar, "
          f"{sum(1 for e in evs if e[1]=='imu')} imu, {sum(1 for e in evs if e[1]=='img')} img")
    t0, t1 = evs[0][0], evs[-1][0]
    print(f"time span: {t0/1e9:.3f} -> {t1/1e9:.3f} ({(t1-t0)/1e9:.1f}s)")

    if os.path.exists(BAG):
        os.remove(BAG)
    with Writer(BAG) as w:
        c_lid = w.add_connection('/livox/lidar', 'livox_ros_driver/msg/CustomMsg', typestore=ts)
        c_imu = w.add_connection('/livox/imu', 'sensor_msgs/msg/Imu', typestore=ts)
        c_img = w.add_connection('/left_camera/image/compressed', 'sensor_msgs/msg/CompressedImage', typestore=ts)
        Header = ts.types['std_msgs/msg/Header']
        Time = ts.types['builtin_interfaces/msg/Time']
        Imu = ts.types['sensor_msgs/msg/Imu']
        Vec3 = ts.types['geometry_msgs/msg/Vector3']
        Quat = ts.types['geometry_msgs/msg/Quaternion']
        CI = ts.types['sensor_msgs/msg/CompressedImage']
        Z9 = np.zeros(9)

        for i, (t_ns, kind, payload) in enumerate(evs):
            if kind == 'lidar':
                w.write(c_lid, t_ns, custommsg_bytes(t_ns, payload))
            elif kind == 'imu':
                g, a = payload
                sec, nsec = divmod(t_ns, 1_000_000_000)
                imu = Imu(header=Header(seq=0, stamp=Time(sec=sec, nanosec=nsec), frame_id='imu'),
                          orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=Z9,
                          angular_velocity=Vec3(x=float(g[0]), y=float(g[1]), z=float(g[2])),
                          angular_velocity_covariance=Z9,
                          linear_acceleration=Vec3(x=float(a[0]), y=float(a[1]), z=float(a[2])),
                          linear_acceleration_covariance=Z9)
                w.write(c_imu, t_ns, ts.serialize_ros1(imu, 'sensor_msgs/msg/Imu'))
            else:
                sec, nsec = divmod(t_ns, 1_000_000_000)
                with open(payload, 'rb') as f:
                    jpg = np.frombuffer(f.read(), dtype=np.uint8)
                ci = CI(header=Header(seq=0, stamp=Time(sec=sec, nanosec=nsec), frame_id='camera'),
                        format='jpeg', data=jpg)
                w.write(c_img, t_ns, ts.serialize_ros1(ci, 'sensor_msgs/msg/CompressedImage'))
            if i % 5000 == 0:
                print(f"  {i}/{len(evs)} ...")
    sz = os.path.getsize(BAG) / 1e9
    print(f"\ndone: {BAG} ({sz:.2f} GB)")


if __name__ == '__main__':
    main()
