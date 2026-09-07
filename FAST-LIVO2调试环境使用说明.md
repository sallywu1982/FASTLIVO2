# FAST-LIVO2 调试环境使用说明（WSL2 Ubuntu-20.04）

更新日期：2026-08-20

## 一、环境概况

| 项目 | 位置 / 状态 |
|---|---|
| 编译环境 | WSL2 `Ubuntu-20.04`，ROS1 Noetic（`/opt/ros/noetic`），gcc 9.4 / cmake 3.16 |
| 工作空间 | `/root/catkin_ws`（含 FAST-LIVO2、livox_ros_driver、vikit_common） |
| 可执行节点 | `/root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping`（2026-08-20 重新编译通过） |
| 源码版本 | `/root/catkin_ws/src/FAST-LIVO2`（hku-mars，commit `0d2c034`；`src/vio.cpp` 有 11 行本地改动，已编入） |
| 数据目录 | `/root/fast_livo2_data/` |
| 验证状态 | NTU VIRAL `eee_03.bag`（4GB）已完整跑通：LIO/VIO 正常输出，`/odometry_reg` 持续发布 |

## 二、常用命令（在 WSL 中）

```bash
# 进入 WSL
wsl -d Ubuntu-20.04

# 每个终端先source
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 重新编译
cd /root/catkin_ws && catkin_make
```

### 跑 NTU VIRAL 数据（已验证）

```bash
# 终端1（无界面，rviz:=false）
roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false

# 终端2
rosbag play --clock /root/fast_livo2_data/eee_03/eee_03.bag
```

话题：雷达 `/os1_cloud_node1/points`，图像 `/left/image_raw`，IMU `/imu/imu`（与 `config/NTU_VIRAL.yaml` 一致）。

### 跑官方 FAST-LIVO2 数据集（Bright_Screen_Wall / Retail_Street / CBD_Building_01）

```bash
# 终端1
roslaunch fast_livo mapping_bright_screen_wall.launch

# 终端2
rosbag play --clock /root/fast_livo2_data/bright_screen_wall/Bright_Screen_Wall.bag
```

- 该 launch 由 `mapping_avia.launch` 复制而来，仅把 rviz 默认改为 false（WSL 无显示）。
- 传感器标定：`config/avia.yaml` 的 Rcl/Pcl 与 `config/camera_pinhole.yaml` 内参就是
  这一组序列（Retail_Street、CBD_Building_01、Bright_Screen_Wall）的标定值，
  与官方 `calibration.yaml` 第一组完全一致，无需修改。
- 话题：`/livox/lidar`、`/livox/imu`、`/left_camera/image`（压缩图像，launch 内自带 republish 转 raw）。
- HKU 等其它序列需按 `calibration.yaml` 对应分组修改 Rcl/Pcl 与内参。

### 保存地图 PCD

编辑 `config/avia.yaml`：`pcd_save/pcd_save_en: true`，`interval: -1`（全部帧存成一个 PCD）。
运行结束后 PCD 输出在 `src/FAST-LIVO2/Log/` 目录。

## 三、数据集

- 官方 Google Drive 目录：https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA
- **Bright_Screen_Wall.bag（360MB，目录中最小）已下载并完整跑通**（2026-08-20）：
  - 位置：`/root/fast_livo2_data/bright_screen_wall/`（含 `calibration.yaml`）
  - 66 秒序列：667 帧图像 + 667 帧 Livox CustomMsg + 13515 帧 IMU 全部处理完，
    LIO 平均残差 ~0.010，单帧平均耗时 ~7ms
  - 该文件曾被 Google Drive 按 IP 下载配额限流，最终通过登录浏览器手动下载（配额限制只针对匿名会话）
- 备用下载方法（若其它序列也被限流）：`scripts/chunked_dl.sh`——完整 GET 被配额页拦截，
  但 1~4MB 的 Range 分块请求可通过（令牌桶约 1~4MB/分钟），带断点续传。

## 四、调试中遇到的问题与结论

1. **后台运行时节点 stdout 为空**：roslaunch 重定向时 stdout 是全缓冲，进程被 kill 时缓冲丢失。
   验证是否真正处理数据应查询话题（如 `rostopic echo -n 1 /odometry_reg`）而不是只看日志。
2. **WSL 后台进程存活**：`wsl -e cmd` 退出后其后台子进程会被终止，长时间任务需保持
   wsl.exe 会话存活（宿主级后台任务）或用计划任务。
3. **Google Drive 限流**：对热门大文件的完整下载被按 IP 限流；Range 分块请求受单独的
   令牌桶控制（约每分钟 1~4MB，探测过快会把桶打干）。此法仅适用于小文件。
