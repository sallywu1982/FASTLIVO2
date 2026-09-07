#!/bin/bash
# stop_live.sh — 停止循环回放并清理全部相关进程
pkill -9 -f "run_live_loop" 2>/dev/null
pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "rosbag play" 2>/dev/null
pkill -9 -f "republish" 2>/dev/null
pkill -9 -f "live_viewer.py" 2>/dev/null
pkill -9 -f "rqt_image_view" 2>/dev/null
pkill -9 -f "rosmaster|roscore|rosout" 2>/dev/null
sleep 1
LEFT=$(ps aux | grep -E "fastlivo_mapping|rosbag play|live_viewer|rqt_image|rosmaster" | grep -v grep | wc -l)
echo "=== 清理完成，残留相关进程: $LEFT ==="
