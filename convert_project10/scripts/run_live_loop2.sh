#!/bin/bash
# run_live_loop2.sh — FAST-LIVO2 实时回放·逐轮循环版
# 说明: FAST-LIVO2 节点遇到 IMU 时间倒退会拒收(每个 --loop 周期刷 "imu loop back" 错误)，
#       因此每轮循环重启 LIVO 节点保证状态干净；viewer/rqt/republish/roscore 跨轮复用，
#       viewer 检测到时间戳回到起点会自动清空重新累积，画面无缝衔接。
# 停止: stop_live.sh
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

# 跨轮复用的三个进程
rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image >/tmp/republish.log 2>&1 &
REP=$!
python3 "$E/convert_project10/scripts/live_viewer.py" >/tmp/viewer.log 2>&1 &
VIEW=$!
rosrun rqt_image_view rqt_image_view /left_camera/image_raw >/tmp/rqt.log 2>&1 &
RQT=$!
sleep 6
kill -0 $VIEW 2>/dev/null && echo "viewer PID=$VIEW" || echo "viewer 失败"
kill -0 $RQT 2>/dev/null && echo "rqt PID=$RQT" || echo "rqt 失败"
kill -0 $REP 2>/dev/null && echo "republish PID=$REP" || { echo "republish 失败"; cleanup; exit 1; }

CYCLE=0
while true; do
  CYCLE=$((CYCLE+1))
  echo "=== 第 $CYCLE 轮开始 ==="
  rosrun fast_livo fastlivo_mapping >"$LOG" 2>&1 &
  NODE=$!
  sleep 5
  if ! kill -0 $NODE 2>/dev/null; then
    echo "节点启动失败，3s 后重试"
    sleep 3
    continue
  fi
  rosbag play --clock "$BAG" -u 294 >/tmp/play.log 2>&1
  echo "--- 第 $CYCLE 轮回放完成: frames=$(grep -ac 'Get LiDAR' $LOG) err=$(grep -ac ERROR $LOG) ---"
  sleep 8   # 停留看最终地图
  kill -INT $NODE 2>/dev/null
  for i in $(seq 1 12); do kill -0 $NODE 2>/dev/null || break; sleep 1; done
  kill -9 $NODE 2>/dev/null
  sleep 1
done
