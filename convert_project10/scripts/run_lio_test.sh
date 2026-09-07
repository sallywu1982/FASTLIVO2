#!/bin/bash
# run_lio_test.sh — WSL 中无头运行 FAST-LIVO2 冒烟测试（自防御版）
# 用法: bash run_lio_test.sh [yaml名] [播放秒数]
CFG="${1:-project10_lio.yaml}"
DUR="${2:-100}"
E="/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习"
BAG="$E/convert_project10/out/project10.bag"
LOG=/tmp/lio_node.log

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# 1) 彻底清理
pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "rosbag play" 2>/dev/null
pkill -9 -f "roscore\|rosmaster\|rosout" 2>/dev/null
sleep 2

# 2) 全新 roscore
roscore >/tmp/roscore.log 2>&1 &
for i in $(seq 1 15); do
  rosnode list >/dev/null 2>&1 && break
  sleep 1
done
rosnode list >/dev/null 2>&1 || { echo "!! roscore 启动失败"; tail /tmp/roscore.log; exit 1; }
echo "roscore OK"

# 3) 参数 + 节点
rosparam set /use_sim_time true
rosparam load "$E/FAST-LIVO2/config/$CFG"
rosparam load "$E/FAST-LIVO2/config/camera_project10.yaml" /laserMapping
rosrun fast_livo fastlivo_mapping >"$LOG" 2>&1 &
NODE=$!
# 图像 republish（bag 为 compressed；img_en=0 时无图像也无害）
rosrun image_transport republish compressed in:=/left_camera/image raw out:=/left_camera/image >/tmp/republish.log 2>&1 &
REP=$!
echo "republish 已启动"
sleep 5
kill -0 $NODE 2>/dev/null || { echo "!! 节点启动即退出:"; tail -15 "$LOG"; exit 1; }
echo "节点已启动 (pid $NODE)"

# 4) 播放（同时录制 Odometry 到文件）
echo "=== 播放前 ${DUR}s ==="
timeout $((DUR+40)) rostopic echo --noarr /aft_mapped_to_init > /tmp/odom_rec.log 2>/dev/null &
REC=$!
timeout $((DUR+90)) rosbag play --clock "$BAG" -u "$DUR" >/tmp/play.log 2>&1
echo "=== 播放结束 rc=$? ==="
sleep 3
kill $REC 2>/dev/null
# Odometry 轨迹统计（话题 /aft_mapped_to_init）
python3 - <<'PYEOF'
import re, math
txt = open('/tmp/odom_rec.log', errors='ignore').read()
pos = re.findall(r'position:\s*\n\s*x: ([-\d.eE+]+)\s*\n\s*y: ([-\d.eE+]+)\s*\n\s*z: ([-\d.eE+]+)', txt)
if not pos:
    print("Odometry: 无样本")
else:
    p = [(float(a), float(b), float(c)) for a, b, c in pos]
    xs = [q[0] for q in p]; ys = [q[1] for q in p]; zs = [q[2] for q in p]
    finite = all(math.isfinite(v) for v in (*xs, *ys, *zs))
    print(f"Odometry 样本 {len(p)} 条, 全部有限: {finite}")
    print(f"x: {min(xs):.2f}..{max(xs):.2f}  y: {min(ys):.2f}..{max(ys):.2f}  z: {min(zs):.2f}..{max(zs):.2f}")
    print(f"水平位移跨度: {math.hypot(max(xs)-min(xs), max(ys)-min(ys)):.2f} m")
PYEOF

# 5) 结果检查（master 还活着才查）
if rosnode list >/dev/null 2>&1; then
  echo "=== 节点存活: $(kill -0 $NODE 2>/dev/null && echo YES || echo NO) ==="
  echo "=== 活跃话题（有订阅者的） ==="
  rostopic list 2>/dev/null | grep -v "^/ros" | head -12
  echo "=== /Odometry 最后一条位姿 ==="
  timeout 8 rostopic echo -n 1 --noarr /Odometry 2>/dev/null | grep -A4 "position:" | head -6
  timeout 8 rostopic echo -n 1 --noarr /Odometry 2>/dev/null | grep -A8 "orientation:" | head -9
  echo "=== Odometry 频率 ==="
  timeout 8 rostopic hz /Odometry 2>/dev/null | head -2
else
  echo "!! master 已死（节点可能崩溃）"
fi
echo "=== 节点日志（错误/警告/关键行） ==="
grep -ai "error\|warn\|reset\|nan\|imu init\|lidar" "$LOG" | grep -av "XmlRpc" | head -12
echo "=== 日志行数: $(wc -l < "$LOG") ==="
echo "=== VIO 状态（视觉帧处理行） ==="
grep -ac "vio\|VIO\|image\|img" "$LOG" | head -1
pkill -9 -f fastlivo_mapping 2>/dev/null
pkill -9 -f "republish" 2>/dev/null
echo "=== 完成 ==="
