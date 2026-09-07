#!/bin/bash
# run_viz.sh — 全量 LIVO 运行 + 优雅退出保存 PCD 地图 + 拷回成果到 E 盘
DUR="${1:-294}"
E="/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习"
BAG="$E/convert_project10/out/project10.bag"
LOG=/tmp/lio_node.log

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "rosbag play" 2>/dev/null
pkill -9 -f "republish" 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null
sleep 2
rm -rf ~/catkin_ws/src/FAST-LIVO2/Log/pcd 2>/dev/null
mkdir -p ~/catkin_ws/src/FAST-LIVO2/Log/pcd

roscore >/tmp/roscore.log 2>&1 &
for i in $(seq 1 15); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list >/dev/null 2>&1 || { echo "roscore 失败"; exit 1; }
echo "roscore OK"

rosparam set /use_sim_time true
rosparam load "$E/FAST-LIVO2/config/project10_viz.yaml"
rosparam load "$E/FAST-LIVO2/config/camera_project10.yaml" /laserMapping

rosrun fast_livo fastlivo_mapping >"$LOG" 2>&1 &
NODE=$!
rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image >/tmp/republish.log 2>&1 &
REP=$!
sleep 5
kill -0 $NODE 2>/dev/null || { echo "节点启动失败"; tail -10 "$LOG"; exit 1; }
echo "节点+republish 已启动"

timeout $((DUR+30)) rostopic echo --noarr /aft_mapped_to_init > /tmp/odom_rec.log 2>/dev/null &
REC=$!

echo "=== 播放全量 ${DUR}s ==="
timeout $((DUR+90)) rosbag play --clock "$BAG" -u "$DUR" >/tmp/play.log 2>&1
echo "=== 播放结束 rc=$? ==="
sleep 3
kill $REC 2>/dev/null

# 优雅退出节点（触发 savePCD）
echo "=== SIGINT 节点, 等待保存 PCD ==="
kill -INT $NODE
for i in $(seq 1 60); do kill -0 $NODE 2>/dev/null || break; sleep 1; done
if kill -0 $NODE 2>/dev/null; then
  echo "节点未自行退出, 再等/强杀"
  kill -INT $NODE; sleep 10; kill -9 $NODE 2>/dev/null
fi
kill -9 $REP 2>/dev/null

echo "=== PCD 产物 ==="
ls -la ~/catkin_ws/src/FAST-LIVO2/Log/pcd/ 2>/dev/null
echo "=== 拷回 E 盘 ==="
mkdir -p "$E/convert_project10/out/pcd"
cp ~/catkin_ws/src/FAST-LIVO2/Log/pcd/*.pcd "$E/convert_project10/out/pcd/" 2>/dev/null && echo "PCD copied"
cp /tmp/odom_rec.log "$E/convert_project10/out/odom_rec.log" && echo "odom copied"
echo "=== 日志统计 ==="
grep -ac "Get LiDAR" "$LOG"
grep -ac "ERROR" "$LOG"
grep -a "saved to" "$LOG" | head -3
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null
echo "=== 完成 ==="
