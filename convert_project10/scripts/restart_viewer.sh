#!/bin/bash
# restart_viewer.sh — 回放过程中热重启 live_viewer.py（不影响节点与 rosbag play）
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export DISPLAY=:0
pkill -9 -f "live_viewer\.py$"
sleep 1
nohup python3 "/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习/convert_project10/scripts/live_viewer.py" >/tmp/viewer.log 2>&1 &
sleep 12
if pgrep -f "live_viewer\.py$" >/dev/null; then echo "viewer RESTARTED"; else echo "viewer FAILED"; fi
echo "parse_warn=$(grep -c 'cloud parse' /tmp/viewer.log)"
