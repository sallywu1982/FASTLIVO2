#!/bin/bash
# run_live_loop.sh — FAST-LIVO2 实时回放·无限循环版（rosbag play --loop）
# 循环时 bag 时间戳回到起点，live_viewer.py 检测到时间倒退会自动清空重新累积
# 停止方式: 运行 stop_live.sh（推荐），或对本进程 Ctrl-C
E="/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习"
BAG="$E/convert_project10/out/project10.bag"
LOG=/tmp/lio_node.log

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export DISPLAY=:0

NODE=""; REP=""; VIEW=""; RQT=""
cleanup() {
  echo "=== 清理中 ==="
  [ -n "$NODE" ] && kill -INT $NODE 2>/dev/null && sleep 3
  kill -9 $NODE $REP $VIEW $RQT 2>/dev/null
  pkill -9 -f "rosbag play" 2>/dev/null
  pkill -9 -f "rosmaster|roscore|rosout" 2>/dev/null
  echo "=== 已停止 ==="
}
trap cleanup INT TERM

pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "rosbag play" 2>/dev/null
pkill -9 -f "republish" 2>/dev/null
pkill -9 -f "live_viewer.py" 2>/dev/null
pkill -9 -f "rqt_image_view" 2>/dev/null
pkill -9 -f "rosmaster|roscore|rosout" 2>/dev/null
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
kill -0 $NODE 2>/dev/null || { echo "节点启动失败"; tail -10 "$LOG"; cleanup; exit 1; }

python3 "$E/convert_project10/scripts/live_viewer.py" >/tmp/viewer.log 2>&1 &
VIEW=$!
rosrun rqt_image_view rqt_image_view /left_camera/image_raw >/tmp/rqt.log 2>&1 &
RQT=$!
sleep 8
kill -0 $VIEW 2>/dev/null && echo "viewer PID=$VIEW" || { echo "viewer 失败"; tail -5 /tmp/viewer.log; }
kill -0 $RQT 2>/dev/null && echo "rqt PID=$RQT" || echo "rqt 失败"

echo "=== 开始循环回放（每轮约295s，无限循环；stop_live.sh 停止）==="
rosbag play --clock --loop "$BAG" >/tmp/play.log 2>&1
PLAY=$!
wait $PLAY
cleanup
