#!/bin/bash
# run_rviz_live.sh — RViz 实时观看 FAST-LIVO2 处理 project10.bag 的完整回放
# 用法: wsl -d Ubuntu-20.04 -- bash run_rviz_live.sh [时长秒数, 默认294]
# 依赖 WSLg（DISPLAY=:0），RViz 窗口会出现在 Windows 桌面
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
pkill -9 -f "rviz -d" 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null
sleep 2

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

rosrun rviz rviz -d ~/catkin_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz >/tmp/rviz.log 2>&1 &
RVIZ=$!
sleep 8
kill -0 $RVIZ 2>/dev/null || { echo "RViz 启动失败:"; tail -15 /tmp/rviz.log; kill $NODE $REP 2>/dev/null; exit 1; }
echo "节点+republish+RViz 已启动 (RVIZ_PID=$RVIZ)"

timeout $((DUR+30)) rostopic echo --noarr /aft_mapped_to_init > /tmp/odom_rec.log 2>/dev/null &
REC=$!

echo "=== 开始实时回放 ${DUR}s（RViz 窗口应已显示）==="
timeout $((DUR+60)) rosbag play --clock "$BAG" -u "$DUR" >/tmp/play.log 2>&1
echo "=== 回放结束 rc=$? ==="

echo "=== 保留 45s 供查看最终地图 ==="
sleep 45
kill $REC 2>/dev/null

kill -INT $NODE 2>/dev/null
for i in $(seq 1 20); do kill -0 $NODE 2>/dev/null || break; sleep 1; done
kill -9 $NODE $REP $RVIZ 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null

echo "=== 处理统计 ==="
echo "雷达帧: $(grep -ac 'Get LiDAR' $LOG)  错误: $(grep -ac 'ERROR' $LOG)"
echo "位姿样本: $(grep -ac 'seq:' /tmp/odom_rec.log)"
echo "=== 完成 ==="
