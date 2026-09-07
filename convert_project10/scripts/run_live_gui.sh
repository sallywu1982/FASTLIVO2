#!/bin/bash
# run_live_gui.sh — 实时回放 project10.bag 并弹出 GUI 观看（WSLg）
#   窗口1: matplotlib 实时地图(俯视/侧视/高度剖面+轨迹)  窗口2: rqt_image_view 相机画面
# 用法: MSYS_NO_PATHCONV=1 wsl -d Ubuntu-20.04 -- bash -c 'bash "/mnt/e/.../run_live_gui.sh" 294'
# 注意: 本脚本文件名不得包含下列 pkill 模式子串（避免自杀）
DUR="${1:-294}"
E="/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习"
BAG="$E/convert_project10/out/project10.bag"
LOG=/tmp/lio_node.log

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export DISPLAY=:0

pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "rosbag play" 2>/dev/null
pkill -9 -f "republish" 2>/dev/null
pkill -9 -f "live_viewer.py" 2>/dev/null
pkill -9 -f "rqt_image_view" 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null
sleep 2
mkdir -p "$E/convert_project10/out/live"
rm -f "$E/convert_project10/out/live/latest.png"

roscore >/tmp/roscore.log 2>&1 &
for i in $(seq 1 15); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list >/dev/null 2>&1 || { echo "roscore 失败"; exit 1; }
echo "roscore OK"

rosparam set /use_sim_time true
rosparam load "$E/FAST-LIVO2/config/project10.yaml"
rosparam load "$E/FAST-LIVO2/config/camera_project10.yaml" /laserMapping

rosrun fast_livo fastlivo_mapping >"$LOG" 2>&1 &
NODE=$!
rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image >/tmp/republish.log 2>&1 &
REP=$!
sleep 5
kill -0 $NODE 2>/dev/null || { echo "节点启动失败"; tail -10 "$LOG"; exit 1; }

python3 "$E/convert_project10/scripts/live_viewer.py" >/tmp/viewer.log 2>&1 &
VIEW=$!
rosrun rqt_image_view rqt_image_view /left_camera/image_raw >/tmp/rqt.log 2>&1 &
RQT=$!
sleep 8
kill -0 $VIEW 2>/dev/null || { echo "viewer 启动失败:"; tail -10 /tmp/viewer.log; } && echo "viewer PID=$VIEW"
kill -0 $RQT 2>/dev/null || { echo "rqt 启动失败:"; tail -5 /tmp/rqt.log; } && echo "rqt PID=$RQT"

echo "=== 开始实时回放 ${DUR}s（两个 GUI 窗口应已显示）==="

timeout $((DUR+30)) rostopic echo --noarr /aft_mapped_to_init > /tmp/odom_rec.log 2>/dev/null &
REC=$!

timeout $((DUR+60)) rosbag play --clock "$BAG" -u "$DUR" >/tmp/play.log 2>&1
echo "=== 回放结束 rc=$? ==="

echo "=== 保留 45s 供查看最终地图 ==="
sleep 45
kill $REC 2>/dev/null

kill -INT $NODE 2>/dev/null
for i in $(seq 1 20); do kill -0 $NODE 2>/dev/null || break; sleep 1; done
kill -9 $NODE $REP $VIEW $RQT 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null

echo "=== 处理统计 ==="
echo "雷达帧: $(grep -ac 'Get LiDAR' $LOG)  错误: $(grep -ac 'ERROR' $LOG)"
echo "位姿样本: $(grep -ac 'seq:' /tmp/odom_rec.log)"
echo "viewer 日志尾部:"; tail -3 /tmp/viewer.log 2>/dev/null
echo "=== 完成 ==="
