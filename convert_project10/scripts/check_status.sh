#!/bin/bash
# check_status.sh — 查看当前 WSL 内 ROS 相关残留进程 + 上一轮节点统计
echo "=== 残留进程 ==="
ps aux | grep -E "fastlivo_mapping|rosbag|rostopic|live_viewer|rqt_image|rosmaster|roscore|rosout|republish" | grep -v grep | head -15
echo "=== 节点统计 ==="
if [ -f /tmp/lio_node.log ]; then
  echo "lidar_frames: $(grep -ac "Get LiDAR" /tmp/lio_node.log)"
  echo "errors: $(grep -ac ERROR /tmp/lio_node.log)"
fi
echo "=== 里程计样本 ==="
[ -f /tmp/odom_rec.log ] && grep -ac "seq:" /tmp/odom_rec.log
