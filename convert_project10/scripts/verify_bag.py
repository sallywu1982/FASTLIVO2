# -*- coding: utf-8 -*-
"""verify_bag.py — 读回 project10.bag 验证：消息数、时间戳、内容解码"""
import os
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BAG = os.path.join(BASE, "out", "project10.bag")

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ts = get_typestore(Stores.ROS1_NOETIC)
CP = "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n"
CM = "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n"
ts.register(get_types_from_msg(CP, 'livox_ros_driver/msg/CustomPoint'))
ts.register(get_types_from_msg(CM, 'livox_ros_driver/msg/CustomMsg'))

counts = {}
first, last = {}, {}
samples = {'lidar': [], 'imu': [], 'img': []}

with Reader(BAG) as r:
    print(f"bag messages: {r.message_count}, duration: {r.duration/1e9:.1f}s")
    conns = {c.topic: c for c in r.connections}
    print("topics:", [(t, conns[t].msgtype) for t in conns])
    for conn, t_ns, raw in r.messages():
        counts[conn.topic] = counts.get(conn.topic, 0) + 1
        if conn.topic not in first: first[conn.topic] = t_ns
        last[conn.topic] = t_ns
        if conn.topic == '/livox/lidar' and counts[conn.topic] % 800 == 1:
            m = ts.deserialize_ros1(raw, 'livox_ros_driver/msg/CustomMsg')
            samples['lidar'].append((m.timebase, m.point_num, len(m.points),
                                     m.points[0].x, m.points[0].z, m.points[len(m.points)//2].offset_time))
        if conn.topic == '/livox/imu' and counts[conn.topic] % 15000 == 1:
            m = ts.deserialize_ros1(raw, 'sensor_msgs/msg/Imu')
            samples['imu'].append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                                   m.linear_acceleration.z, m.angular_velocity.x))
        if conn.topic == '/left_camera/image/compressed' and counts[conn.topic] % 900 == 1:
            m = ts.deserialize_ros1(raw, 'sensor_msgs/msg/CompressedImage')
            samples['img'].append((m.header.stamp.sec, len(m.data), bytes(m.data[:4]).hex()))

print("\ncounts:", counts)
for t in first:
    print(f"  {t}: {first[t]/1e9:.3f} -> {last[t]/1e9:.3f}")
print("\nlidar samples (timebase, point_num, len, x0, z0, mid_off_ns):")
for s in samples['lidar']: print("  ", s)
print("imu samples (t, acc_z, gyro_x):")
for s in samples['imu']: print("  ", s)
print("img samples (t, size, magic):")
for s in samples['img']: print("  ", s)
