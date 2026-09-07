# convert_project10 — 公司 shuttle 数据 → FAST-LIVO2 rosbag 转换工具链

把 `D:\Image2ImageTestData_20260813\Project_job10-08180901` 的专有格式数据
（.lid 雷达 / .dat Pos-IMU / JPG 相机）转换成 FAST-LIVO2 可直接处理的 ROS1 rosbag。

## 产物

| 文件 | 说明 |
|---|---|
| `out/project10.bag` | **2.21GB**, 294.3s, 61,890 条消息（读回验证通过） |
| `FAST-LIVO2/config/project10.yaml` | 主配置（含外参候选） |
| `FAST-LIVO2/config/camera_project10.yaml` | M 相机内参 |
| `FAST-LIVO2/launch/mapping_project10.launch` | 启动文件 |

话题（均为 epoch-UTC 时间戳）：

| 话题 | 类型 | 数量 | 频率 |
|---|---|---|---|
| `/livox/lidar` | livox_ros_driver/CustomMsg | 3004 | 10.4Hz, ~7-14k 点/帧 |
| `/livox/imu` | sensor_msgs/Imu | 55559 | 200Hz, acc 已转 m/s², gyro rad/s |
| `/left_camera/image/compressed` | sensor_msgs/CompressedImage | 3327 | 11.3Hz (M 相机 1280×1080 原始 JPEG) |

## 逆向得到的文件格式（核心成果）

### .lid（雷达原始码流）
```
[~1045B 参数头(含全部标定: 相机内参/外参/杆臂)] [数据区]
数据区 = 连续 1380B 子包:
  +0   4B  魔数 XX 00 01 02 (XX=包计数)
  +4   16B 保留(全零)
  +20  8B  u64 时间戳 epoch-ns   ← 相邻包差恒 480,000ns
  +28  96×14B 点记录: 3×i32(mm) + u16(属性/反射率)
  +1376 4B 尾(低u16=0x6012, 高u16=序号)
200 包 = 1 帧 ≈ 96ms
```
3 个文件共 3004 帧，帧时间与官方 `logLidarParse.txt` 对齐误差 1ms（首帧）。

### .dat（KQI 容器，Pos/IMU）
```
记录 = ['KQI\0'][u16 type][u16 payload_len][u8 sub_len] + payload
首条大记录内嵌与 .lid 相同的参数头
type=1 payload = n × 60B IMU 样本块:
  样本 58B: [u32 0x01000000][u16 计数器×256][保留][u64 epoch-ns @+26][6×f32 @+34][2B CRC]
  6×f32 = 陀螺(rad/s) ×3 + 加计(g) ×3
```
共 55,559 样本 ≈ 200Hz；静止段 |acc|=0.994g 验证通过。

### 相机文件名时间戳
`2432-176546-579_00000-M.JPG` → GPS 周内秒 176546.579 →
epoch = 1786838400(周起始 2026-08-16) + SOW − 18s(闰秒)。

## ⚠️ 关键发现：点云被设备 INS 调平（已修复）

设备输出点云前用内部 INS 消除了 roll/pitch（偏航保留）：
- 证据：t=45.6s 载体俯仰 ~12°（陀螺积分）而点云地面倾角 0.04°；
  t>60s IMU 重力方向永久倾斜 34.5°（长坡道）而点云地面全程水平。
- IMU 芯片相对机体另有 19.8° 安装楔角（静止段重力实测，恒定）。
- FAST-LIVO2 要求雷达-IMU 刚体固定外参 ⇒ 必须还原原始坐标。

**修复**（`scripts/unlevel_lidar.py`）：对 IMU 跑 Mahony 互补滤波得姿态 R(t)，
逐包应用 `p_raw = W·R(t)ᵀ·Rz(ψ(t))·p_leveled`（W=初始楔角=R(0)，推导保证 t=0 为单位阵、
坡道上恰消去调平量）。验证：还原后地面法向 = W·g_chip(t)（倾斜时段内点比例 +44~50%）。

## 流水线（可重复执行）

```
python scripts/decode_lid.py        # .lid → out/lidar_frames/frame_XXXXX.npz (3004帧)
python scripts/decode_dat.py        # .dat → out/imu.npz (55559样本)
python scripts/unlevel_lidar.py     # Mahony 姿态重建 + 反旋转还原原始坐标
python scripts/make_bag.py          # → out/project10.bag (2.21GB)
python scripts/verify_bag.py        # 读回验证
```
分析/验证脚本：`analyze_*.py`（格式逆向过程）、`check_extrinsic.py`、`validate_phase1.py`。

## 标定换算（来源：.lid 文件头 + 静止段重力实测）

- **M 相机内参**（camera_project10.yaml）：f=4.1618mm / 2.4µm → fx=fy=1734.083px；
  cx=640+0.076mm/2.4µm=671.67，cy=540−19.046/2.4=520.73；畸变 K1=0.0812, K2=−0.0069,
  P1=−0.0032, P2=0.0005（文件头 K3=−1.48 疑非标准径向项，未采用，如成像边缘畸变大需复核）
- **extrinsic_T**: [0.023, −0.011, 0.044]（文件头中心偏移）
- **extrinsic_R**: W(19.8° 楔角, 重力实测) × 偏航候选。主候选 β=−90°(M2I Z=−90)，
  另 3 个备选已注释在 project10.yaml
- **Rcl/Pcl**: 文件头 M 相机 Camera2B1(−2.685°,−71.275°,92.764°)+杆臂链式合成初值

## Linux 端运行（已全自动验证 ✅）

本机 WSL Ubuntu-20.04 已有编译好的 FAST-LIVO2 环境（~/catkin_ws）。
免拷贝直接跑（配置从 /mnt/e 加载）：

```bash
# LIO 冒烟测试（视觉关闭, 前 100s）:
wsl -d Ubuntu-20.04 -- bash "/mnt/e/StoneRecord/ws-Work/06_FAST-LIVO2学习/convert_project10/scripts/run_lio_test.sh" project10_lio.yaml 100
# 完整 LIVO（全量 294s, 含视觉）:
wsl -d Ubuntu-20.04 -- bash ".../run_lio_test.sh" project10.yaml 294
```
脚本自动：清进程 → roscore → 加载参数 → rosrun 节点 + image republish →
rosbag play --clock → 录制 /aft_mapped_to_init → 输出轨迹统计/存活/NaN 标志。

### 实测结果（2026-08-24, WSL Noetic）
| 测试 | 结果 |
|---|---|
| LIO 100s | ✅ 988 帧全部处理，0 ERROR/NaN，残差 4-6cm |
| LIO 70s + 轨迹 | ✅ 673 条 odometry 全有限，水平 25m + 爬升 14.6m（坡道段） |
| **LIVO 294s 全量** | ✅ **3263 条 odometry 全有限，水平 253m + 爬升 83m，视觉处理 19,635 行** |
| 外参 β | ✅ 数据测定 β=0°，首次尝试即收敛（无需试跑换候选） |

## 遗留不确定性

1. ~~extrinsic_R 偏航 β（4 选 1）~~ **已用数据测定：β≈0°（2°）**——俯仰过渡段旋转轴向拟合
   （`solve_beta.py`，β=0 轴向误差 27.8° vs ±90°/180° 的 87-144°，判别决定性）。
   project10.yaml 主候选已更新为 β=0；若 LIO 仍发散按 yaml 注释换备选。
2. **Rcl/Pcl 为合成初值**：欧拉角序（假定 ZYX）与 B1 参考系未完全确认，建议 FAST-Calib 标定。
3. 反旋转姿态来自自建互补滤波（稳态误差 ~2-3°，坡道离心加速度所致），LIO 在线优化可吸收。
4. IMU 有 ~9% 样本缺失（dt 异常），FAST-LIVO2 的 imu 缓冲容忍，如影响可再插值补齐。
5. 相机主点/畸变符号约定（±31.7px 偏移方向）未实拍验证。
