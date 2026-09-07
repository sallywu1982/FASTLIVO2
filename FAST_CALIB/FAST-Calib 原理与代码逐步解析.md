# FAST-Calib 原理与代码逐步解析

> **论文**：FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second（arXiv:2507.17210，香港大学 MARS 实验室，Chunran Zheng & Fu Zhang）
> **代码**：[hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib)（本地克隆于 `FAST-Calib/`，主线 `main` 分支，另拉取 `fast-calib2` 分支作差异对照）
> **本文定位**：把论文的标定流程、数学公式与开源代码逐行对应，并回答五个核心问题（Velo2Cam 对比、LIO 聚合稠密点云、直通滤波、相机孔心求解、Kabsch 法）；另附靶标摆放距离与场景个数的选取准则（§8.4）。
> 所有行号均已对照本地 `main` 分支源码核实（`fast-calib2` 行号以 `FETCH_HEAD` 核实）。

---

## 目录

- [一、总览](#一总览)
- [二、数据准备与输入](#二数据准备与输入)
- [三、LiDAR 数据处理管线逐步解析](#三lidar-数据处理管线逐步解析)
- [四、相机数据处理管线逐步解析](#四相机数据处理管线逐步解析)
- [五、外参配准：Kabsch / SVD 闭式解](#五外参配准kabsch--svd-闭式解)
- [六、五个核心问题的代码级结论](#六五个核心问题的代码级结论)
- [七、FAST-Calib 与 Velo2Cam 对比](#七fast-calib-与-velo2cam-对比)
- [八、实操指南](#八实操指南)
- [九、附录：公式汇总 / 参数对照 / 链接](#九附录)

---

# 一、总览

## 1.1 论文回顾

FAST-Calib 是一个**基于定制 3D 靶标**的 LiDAR-相机外参标定工具，服务于 FAST-LIVO2 等多传感器融合系统：

| 指标 | 数值 |
|---|---|
| 总处理时间 | < 0.7 s（三种配置：Ouster OS1-128 / Livox Avia / Mid360） |
| 点对点配准残差 | 稳定 < 6.5 mm |
| 雷达类型支持 | 机械式（多线）+ 固态（非重复扫描） |
| 初始外参需求 | **无需任何初值** |

**三大贡献**：
1. 对 LiDAR 扫描模式不敏感的圆孔边缘提取（机械式/固态通吃）；
2. 椭圆拟合补偿光斑扩散导致的边缘膨胀（见 §1.4 差异表：**开源代码实际为 2D 圆拟合**）；
3. 高效自动流水线 + 多场景联合优化，4(或 4N) 对 3D-3D 点闭式求解。

## 1.2 仓库结构与文件职责

```
FAST-Calib/
├── CMakeLists.txt                  # 产出 fast_calib 与 multi_fast_calib 两个可执行（-O3）
├── config/qr_params.yaml           # 唯一配置：内参/畸变、靶标几何、距离滤波、输入输出路径
├── launch/calib.launch             # 单场景标定入口（加载 yaml + rviz）
├── launch/multi_calib.launch       # 多场景联合标定入口
├── include/
│   ├── common_lib.h                # 参数加载、Square 几何校验、sortPatternCenters、
│   │                               #   RMSE、投影上色、结果保存（FAST-LIVO2 格式）
│   ├── CustomMsg.h / CustomPoint.h # livox_ros_driver 自定义消息（固态点云）
│   └── color.h                     # 终端彩色输出宏
├── src/
│   ├── main.cpp                    # 单场景主入口（161 行，串联全流程）
│   ├── data_preprocess.hpp         # 读 bag+图像，自动识别雷达类型（167 行）
│   ├── lidar_detect.hpp            # LiDAR 侧孔心提取完整管线（515 行，核心）
│   ├── qr_detect.hpp               # ArUco 检测、位姿平均、孔心计算（366 行）
│   └── multi_scene.cpp             # 多场景联合标定：加权 Kabsch（229 行）
├── scripts/distance_filter_tool.py # 交互式生成距离滤波参数
├── workflow.md                     # 官方算法流程文档（与论文 III-C 对应）
├── calib_data/                     # 放置输入：*.bag + 对应 *.jpg
└── output/                         # 输出：标定结果 txt、彩色点云 pcd、调试图
```

## 1.3 数据流总图

```
 calib_data/xx.bag ──┐
                     ├─► DataPreprocess (data_preprocess.hpp:42)
 calib_data/xx.jpg ──┘     ├─ cv::imread → img_input_
                           └─ rosbag::View → cloud_input_(XYZ+ring) + 自动判型 lidar_type_
                                        │
          ┌─────────────────────────────┴─────────────────────────────┐
          ▼ (switch, main.cpp:43)                                     ▼
 QRDetect::detect_qr                                        LidarDetect::detect_solid_lidar
 (qr_detect.hpp:98)                                         / detect_mech_lidar (lidar_detect.hpp)
 ArUco检测→PnP→4码位姿平均                                   直通滤波→[体素]→RANSAC平面→z=0对齐
 →estimatePoseBoard精化→板布局推算孔心                       →边缘点(角度间隙/ring跳变)→聚类
 →几何一致性筛选                                             →圆拟合→校验→逆变换还原
          │ P_C（相机系 4 孔心）                                       │ P_L（LiDAR系 4 孔心）
          └────────────────────────┬────────────────────────────────┘
                                   ▼
            sortPatternCenters ×2（common_lib.h:320，建立一一对应）
                                   ▼
            TransformationEstimationSVD（main.cpp:71，Kabsch 闭式解）→ T_cam_lidar
                                   ▼
            RMSE(common_lib.h:101) + 投影上色(common_lib.h:169) + 保存(common_lib.h:264)
            → single_calib_result.txt / colored_cloud.pcd / qr_detect.png
            → circle_center_record.txt（多场景联合标定的输入接口）
                                   ▼
            multi_scene.cpp：读最后 3 组记录 → 加权 Kabsch（12 点对）→ multi_calib_result.txt
```

## 1.4 ⚠️ 论文方法 vs 开源实现差异总表（阅读前必看）

对照论文与 `main` 分支代码，存在以下实质性差异（学习时务必区分"论文怎么写"与"代码怎么做"）：

| # | 项目 | 论文（v1, 2025-07） | 开源代码（main） | fast-calib2 分支 |
|---|---|---|---|---|
| 1 | 孔拟合模型 | **Fitzgibbon 椭圆拟合**（一般二次曲线，$B^2-4AC<0$，式 3–4），补偿光斑扩散 | **2D 圆拟合**：固态 `SACMODEL_CIRCLE2D` RANSAC（lidar_detect.hpp:460）；机械迭代 Circle2D RANSAC（:195） | **代数初值 + Huber 加权 Gauss-Newton 圆拟合**（fitCircleRobust, :547），仍非椭圆 |
| 2 | 边缘角度阈值 | 最大角间隙 > **25°** | `M_PI/4` = **45°**（lidar_detect.hpp:420；workflow.md:25 亦写 π/4） | 改用**高反射强度圆环提取**，不用边界角判据 |
| 3 | 体素降采样 | 平面分割后 8 mm | **5 mm**，且在平面分割**之前**（仅固态路径，lidar_detect.hpp:355-358） | auto ROI 10 mm / 圆环 5 mm |
| 4 | 拟合校验 | 半长轴与孔半径差 < 4 cm + 离心率足够低 | 固态：内点到拟合圆心距离与**已知半径** $r_{known}$ 的平均绝对误差 < 2.5 cm（:477-483）；机械：RANSAC 半径限制 $r\pm3$ cm（:201） | $\|r-0.12\|<0.05$ 且拟合误差 < 0.03 |
| 5 | 机械式边缘提取 | （论文统一用 2D 角度间隙法） | **ring 内相邻点距离跳变 > 0.10 m**（:108-165）——与 Velo2Cam 同思路 | 高反射强度圆环点提取 |
| 6 | 相机位姿求解 | 4 个标记**平均位姿** | 平均位姿作初值 → **`estimatePoseBoard` 联合精化**（qr_detect.hpp:219-225） | 同 main |

> 其余步骤（直通滤波、RANSAC 平面阈值 0.01、z=0 对齐、多场景 12 点对联合求解、FAST-LIVO2 输出格式）论文与代码一致。

---

# 二、数据准备与输入

## 2.1 输入形式与采集

- **一对输入 = 一个 rosbag（点云） + 一张对应图像**，静态采集（设备与靶标均不动），放入 `calib_data/`；
- 官方提供示例数据（Mid360 / Avia / Ouster 单场景 + Avia 多场景，Google Drive，见 README:35）与靶标 CAD 模型；
- 多场景采集：至少 3 个场景，靶标分别**朝前 / 朝右 / 朝左**摆放（README 图片说明）。

## 2.2 稠密点云从哪里来（对应问题 2：LIO 聚合）

论文 III-A 原文逻辑：

- 固态雷达（Avia/Mid360）扫描模式**不重复**，多帧点云天然互补，直接融合多帧即得稠密点云；
- **16 线机械雷达每帧扫描线位置固定**，多帧叠加只是同几条线重复，靶板上的圆孔可能采不满。此时采用 **FAST-LIVO2 的 LiDAR 里程计（LIO）模式**：设备带着**轻微俯仰（pitch）运动**扫几秒，LIO 实时估计每帧位姿，把各帧点云变换到统一坐标系合并——俯仰运动让扫描线在靶板上"错开涂抹"，等效提高角分辨率，把孔边缘采满。

要点：
1. **无循环依赖**：LIO 只用 LiDAR+IMU，不需要相机外参（外参正是待标定量）；
2. **"轻微"即可**：运动只需覆盖"扫描线角间隔对应的角度"，短时 LIO 漂移远小于毫米级标定需求；
3. 合并后的稠密点云配合同期一帧图像，即构成一对标定输入。

## 2.3 DataPreprocess：输入读取与雷达类型自动识别

代码位置：`src/data_preprocess.hpp`。无在线订阅，构造函数中**一次性离线读取**（这正是"一秒标定"的工程取向——离线批处理，无实时压力）。

**图像**（:50-56）：`cv::imread(image_path, IMREAD_UNCHANGED)`，失败即报错返回。

**点云 + 判型**（:69-159）：`rosbag::View` 只看 `lidar_topic` 一个话题，逐消息尝试实例化：

```cpp
// data_preprocess.hpp:84  路径 1：Livox 自定义消息 → 固态
if (auto livox_custom_msg = m.instantiate<livox_ros_driver::CustomMsg>()) {
    lidar_type_ = LiDARType::Solid;
    // 逐点拷贝 xyz，line 字段写入 ring（:88-97）
}
// data_preprocess.hpp:102  路径 2：通用 PointCloud2
bool has_ring = ...;                        // 扫描字段表找 "ring"（:106-109）
if (has_ring)  lidar_type_ = LiDARType::Mech;   // 有线号 → 机械式（:121）
else           lidar_type_ = LiDARType::Solid;  // 无线号 → 固态（:125）
```

判型逻辑总结：

| 消息类型 | 判定结果 | ring 来源 |
|---|---|---|
| `livox_ros_driver::CustomMsg` | Solid | 每点的 `line` 字段 |
| `PointCloud2` 且含 `ring` 字段 | Mech | `ring` 字段 |
| `PointCloud2` 无 `ring` 字段 | Solid | 0xFFFF（未知，:150） |

`Common::Point`（common_lib.h:41-55）= XYZ + `ring`，为机械式分支按 ring 分组提取边缘做准备。

## 2.4 距离滤波参数怎么定：distance_filter_tool.py

yaml 中的直通滤波区间（`x_min/x_max/y_min/...`）可用 `scripts/distance_filter_tool.py` 交互式生成：脚本读 bag → 导出带 intensity 的 PCD → Open3D 窗口中人工点选 ≥4 个靶标上的点 → 自动算包围盒并写出同名 txt，把数值抄进 yaml 即可。README 明确说明"滤波框住靶标即可，**允许多余点存在**"（见 §3.1 原因）。

---

# 三、LiDAR 数据处理管线逐步解析

对应论文 III-C 与 workflow.md 第 1 章，输出：LiDAR 系下 4 个孔心 **P_L**。
固态路径：`detect_solid_lidar`（lidar_detect.hpp:330-504）；机械式路径：`detect_mech_lidar`（:65-328）。

## 3.1 步骤 1：直通滤波粗 ROI（对应问题 3）

**这是什么**：直通滤波（pass-through filter）是最简单的点云裁剪——沿某坐标轴设保留区间 `[min, max]`，只留该轴坐标落在区间内的点。"沿 X/Y/Z 三轴"即三个方向的区间过滤**依次串行执行**，等效于一个轴对齐长方体框住靶标区域。

**代码**（固态 :335-351，机械 :70-86，两处相同）：

```cpp
pcl::PassThrough<Common::Point> pass_x;
pass_x.setInputCloud(cloud);
pass_x.setFilterFieldName("x");
pass_x.setFilterLimits(x_min_, x_max_);   // 区间来自 yaml
pass_x.filter(*filtered_cloud_);
// 同样方式再对 y、z 各做一次，共三次
```

**参数**（config/qr_params.yaml:51-56 注释给出推荐值）：

| 轴 | Avia / Mid360 | Ouster | multi_scene_33（示例） |
|---|---|---|---|
| x | [2.0, 5.0] | [1.5, 5.0] | [2.0, 5.0] |
| y | [-0.5, 3.0] | — | [-1.0, 4.0] |
| z | [-0.5, 2.0] | [-0.8, 2.0] | [0.0, 2.0] |

**设计哲学**：阈值极宽松、无需精细调参——框大一点没关系，漏进来的靶板后方/侧方背景点**不影响后续**，因为下一步 RANSAC 平面分割会自动挑出内点最多的平面（即靶板）。对比 Velo2Cam 类方法在靶标任意摆放时需要人工精细调深度阈值，这里用"粗裁剪 + RANSAC 兜底"实现全自动化。

## 3.2 步骤 2：体素降采样（仅固态路径）

```cpp
// lidar_detect.hpp:355-358
pcl::VoxelGrid<Common::Point> voxel_filter;
voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);   // 5 mm 体素
```

- 位置在**平面分割之前**（论文写的是平面分割后 8 mm，见 §1.4 差异 #3）；
- 目的：稠密固态点云降到可计算规模，且体素内取质心相当于轻微软化噪声；
- 机械式路径**不做**体素滤波（点本来就少）。

## 3.3 步骤 3：RANSAC 平面分割

**数学**：平面模型 $ax+by+cz+d=0$。RANSAC 每轮随机采 3 点确定一个平面假设，统计距离小于阈值的内点数，迭代取内点最多者。点到平面距离：

$$\mathrm{dist}(p) = \frac{|a x_p + b y_p + c z_p + d|}{\sqrt{a^2+b^2+c^2}}$$

**代码**（固态 :364-376，机械 :93-106）：

```cpp
pcl::SACSegmentation<Common::Point> plane_segmentation;
plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
plane_segmentation.setMethodType(pcl::SAC_RANSAC);
plane_segmentation.setDistanceThreshold(0.01);   // 内点阈值 1 cm（论文一致）
plane_segmentation.segment(*plane_inliers, *plane_coefficients);
pcl::ExtractIndices<Common::Point> extract;      // 内点即靶板平面点云 plane_cloud_
```

`plane_coefficients->values = [a, b, c, d]`，法向量 $(a,b,c)$ 在下一步立即用到。

## 3.4 步骤 4：旋转对齐到 z=0 平面（3D → 2D 的关键）

**数学**：设平面单位法向量为 $\mathbf{n}$，目标方向 $\mathbf{z}=(0,0,1)$。把 $\mathbf{n}$ 转到 $\mathbf{z}$ 的旋转由**旋转轴 + 旋转角**唯一确定：

$$\mathbf{a} = \mathbf{n} \times \mathbf{z}, \qquad \theta = \arccos(\mathbf{n}\cdot\mathbf{z}), \qquad R = \exp\big([\mathbf{a}]_\times\,\theta\big) \;(\text{即 AngleAxis}(\theta,\mathbf{a}))$$

对每个平面点 $p' = R\,p$，全部点落到 $z' \approx$ 常数的平面内，得到 2D 点云 $(x', y')$。

**代码**（固态 :379-404，机械 :167-185）：

```cpp
Eigen::Vector3d normal(coefficients->values[0], [1], [2]);  // :382-384
normal.normalize();
Eigen::Vector3d z_axis(0, 0, 1);
Eigen::Vector3d axis = normal.cross(z_axis);                 // 旋转轴 :388
double angle = acos(normal.dot(z_axis));                     // 旋转角 :389
Eigen::AngleAxisd rotation(angle, axis);
Eigen::Matrix3d R = rotation.toRotationMatrix();             // :391-392

float average_z = 0.0;
for (const auto& pt : *plane_cloud_) {
    Eigen::Vector3d aligned_point = R * point;
    aligned_cloud_->push_back(...(x, y, 0.0));   // z 强制写 0，只留 (x,y) 做 2D 处理
    average_z += aligned_point.z();               // 记录真实高度，供逆变换还原 :401
}
average_z /= cnt;
```

> 细节：对齐后点的 z 不严格为 0（RANSAC 平面有噪声），代码把它**强制置 0** 但累加 `average_z`，第 9 步逆变换时把圆心高度补回，减小还原误差。

## 3.5 步骤 5：边缘点检测（核心创新所在）

### 3.5.1 论文方法：邻域方向角 + 最大角度间隙

**原理**：z=0 平面内的内部点，其邻域各方向都有点；**边缘点的邻域存在方向性空隙**（朝孔内/板外一侧没有点）。

论文式 (1)：对每个 2D 点 $p_i$，半径内每个邻点 $p_j$ 的方向角

$$\theta_j = \mathrm{atan2}\big((p_j-p_i)_y,\ (p_j-p_i)_x\big)$$

论文式 (2)：角度升序排序后取相邻差（含首尾闭环），最大间隙超过阈值（论文 25°）判为边缘：

$$\Delta\theta_k = \theta_{k+1}-\theta_k \;(k=1..N-1), \qquad \Delta\theta_N = \theta_1 + 2\pi - \theta_N$$

### 3.5.2 代码实现：pcl::BoundaryEstimation 就是论文方法

**已核对 PCL 源码**（features/impl/boundary.hpp），`pcl::BoundaryEstimation` 的内部实现**正是上述算法**：

1. 用该点法向量的正交基 $(u, v)$ 定义切平面坐标系（`getCoordinateSystemOnPlane`）；
2. 半径内每个邻点 $\Delta p_k = p_k - p_i$，投影方向角 $\alpha_k = \mathrm{atan2}(v\cdot\Delta p_k,\ u\cdot\Delta p_k)$（即论文式 1 的切平面版）；
3. 角度排序 → 求相邻最大差 + 首尾回绕差 $2\pi - \alpha_{last} + \alpha_{first}$（即论文式 2）；
4. `return max_dif > angle_threshold;`

**代码**（固态 :407-428）：

```cpp
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
normal_estimator.setRadiusSearch(0.03);                    // 法线估计搜索半径 3 cm

pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
boundary_estimator.setInputCloud(aligned_cloud_);
boundary_estimator.setInputNormals(normals);
boundary_estimator.setRadiusSearch(0.03);                  // 邻域半径 3 cm（论文一致）
boundary_estimator.setAngleThreshold(M_PI / 4);            // 最大角间隙阈值 = 45°（论文写 25°）
boundary_estimator.compute(boundaries);
```

> **这就是"扫描模式不敏感"的根源**：判据只用局部点分布的几何空隙，完全不看点属于哪条扫描线、扫描图案如何，因此机械式与固态通用。

### 3.5.3 机械式路径的替代实现：ring 内距离跳变

机械式分支（:108-165）没有用 BoundaryEstimation，而是沿**扫描环（ring）**找深度不连续：

1. 按 `ring` 分组，每组 < 10 点跳过（:128）；
2. 只保留距拟合平面 **< 0.03 m** 的点（:142-143）；
3. 环内当前点与**前一/后一相邻点的欧氏距离任一 > 0.10 m**（:127 `neighbor_gap_threshold`）→ 判为边缘点（:158-161）。

> 本质：激光扫过孔洞时相邻点距会跳变（孔里没点），与 Velo2Cam 的逐环深度不连续检测同思路（见 §7），但先做了平面约束滤波，且只作**候选边缘**，后续仍走 z=0 对齐 + 圆拟合 + 矩形几何校验的统一管线。

## 3.6 步骤 6：欧式聚类（仅固态路径）

```cpp
// lidar_detect.hpp:430-443
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(0.05);   // 聚类距离阈值 5 cm
ec.setMinClusterSize(50);
ec.setMaxClusterSize(1000);
```

每个孔的边缘点连成一个簇 → 每簇送入圆拟合。机械式路径不用聚类（其边缘点更稀疏），改用迭代 RANSAC 逐个"抠圆"（见 §3.7.3）。

## 3.7 步骤 7：圆/椭圆拟合

### 3.7.1 论文方法：Fitzgibbon 直接最小二乘椭圆拟合

对每个边缘点簇，用一般二次曲线（conic）拟合（论文式 3）：

$$Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0$$

Fitzgibbon 直接最小二乘法在约束 $B^2-4AC<0$（保证解为椭圆）下求解；椭圆中心解析式（论文式 4）：

$$x_c = \frac{2CD - BE}{B^2-4AC}, \qquad y_c = \frac{2AE - BD}{B^2-4AC}$$

**开源代码（main 分支）中不存在此实现**——实际是 2D 圆拟合（下两小节）。圆是椭圆的特例；论文的椭圆拟合用于补偿光斑扩散（激光光斑直径使孔边缘点外扩，投影成"胖圆"= 椭圆），代码用"已知半径先验收误差"来兜底。

### 3.7.2 固态路径：RANSAC 2D 圆 + 已知半径误差校验

```cpp
// lidar_detect.hpp:458-467（对每个聚类）
pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_CIRCLE2D);     // 2D 圆 (xc, yc, r)
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.01);               // 内点阈值 1 cm
seg.setMaxIterations(1000);
seg.segment(*inliers, *coefficients);

// :472-483 已知半径先验校验
for (const auto& idx : inliers->indices)
    error += abs( sqrt(dx*dx + dy*dy) - circle_radius_ );   // |到圆心距离 − 已知半径 0.12m|
error /= inliers->indices.size();
if (error < 0.025)                            // 平均绝对径向误差 < 2.5 cm 才接受
```

模型系数 `coefficients->values = [xc, yc, r]` 即孔心（z=0 平面内）。

### 3.7.3 机械式路径：迭代 RANSAC 抠圆

```cpp
// lidar_detect.hpp:195-258
circle_segmentation.setDistanceThreshold(0.02);                              // 阈值 2 cm
circle_segmentation.setRadiusLimits(circle_radius_-0.03, circle_radius_+0.03); // 半径 0.09~0.15 m
while (xy_cloud->points.size() > 3) {
    segment(inliers, coefficients);          // 找一个圆
    if (无内点 || inliers < 5) break;        // :216-228
    center_z0_cloud_->push_back([xc, yc, 0]);// :242-246
    extract2.setNegative(true);              // 剔除该圆内点，剩余点继续找下一个 :249-254
}
```

一次分割可能同时发现多个孔（尤其 4 孔布局时），用 while 循环"找一个 → 删内点 → 再找"，直到无圆可找。半径限制 ±3 cm 即论文"半长轴差 < 4 cm"校验的代码对应物（数值更紧）。

### 3.7.4 fast-calib2 分支：代数初值 + Huber 鲁棒 Gauss-Newton（现代工程化重写）

```cpp
// fast-calib2: src/lidar_detect.hpp:547 fitCircleRobust
// ① 代数（最小二乘）初值：把 (x²+y²) 线性化
//    x²+y² + a·x + b·y + c = 0  →  [x y 1]·[a b c]ᵀ = −(x²+y²)
//    colPivHouseholderQr 解出后：cx = −a/2, cy = −b/2, r = √(cx²+cy²−c)
// ② Huber 加权 Gauss-Newton 迭代（≤20 次，huber_delta = 0.02，:571）
//    残差 r_i = √(dx²+dy²) − r；Jacobian J = [dx/d, dy/d, −1]
//    权重 w = (|r| ≤ δ) ? 1 : δ/|r|                     (:586)
// ③ 校验：|r − 0.12| < 0.05 且 mean_abs_error < 0.03
```

相比 RANSAC：确定性（无随机性）、抗外点（Huber 降权）、亚厘米内亚毫米精度。

## 3.8 步骤 8：几何一致性校验（4 孔矩形布局）

圆拟合可能输出假孔（板外噪声成环）。利用**已知布局**（孔心间距 0.5 m × 0.4 m）筛除假圆：

```cpp
// lidar_detect.hpp:261-299（机械式；固态在 qr_detect.hpp:289-338 同款；common_lib.h:390-484 Square 类）
comb(center_z0_cloud_->size(), TARGET_NUM_CIRCLES, groups);  // 枚举所有 4 心组合
Square square_candidate(candidates, delta_width_circles_, delta_height_circles_);
groups_scores[i] = square_candidate.is_valid() ? 1.0 : -1;
// 选最高分组；若多于一个组合同时有效 → 报错退出（参数可能设错）
```

`Square::is_valid()`（common_lib.h:431-483）三道检查：

1. **对角线检查**：每点到 4 心质心的距离 ≈ 半对角线 $\frac{1}{2}\sqrt{w^2+h^2}$（相对容差 `2×GEOMETRY_TOLERANCE`，宏为 0.08，common_lib.h:38）；
2. **边长检查**：角排序后四边必须满足"宽-高-宽-高"或"高-宽-高-宽"两种排列（每边相对容差 0.08）。注释（:427-430）说明这是对原 velo2cam 版本的放宽改造；
3. **周长检查**：$|perimeter - 2(w+h)|/(2(w+h)) < 0.08$。

`comb`（common_lib.h:137-167）是标准位掩码组合枚举，源自 velo2cam_calibration。

## 3.9 步骤 9：逆变换回 LiDAR 坐标系

把 z=0 平面内的孔心还原到原始 LiDAR 系（固态 :492-500，机械 :309-327）：

$$p_{origin} = R^{-1}\,\begin{bmatrix} x_c \\ y_c \\ 0 + \bar z \end{bmatrix}$$

```cpp
// lidar_detect.hpp:492-494
Eigen::Vector3d aligned_point(center_point.x, center_point.y, center_point.z + average_z);
Eigen::Vector3d original_point = R_inv * aligned_point;    // R_inv = R.inverse()，:447
```

输出的 `center_cloud`（4 点）即 **P_L**，送入 main.cpp 做配准。

---

# 四、相机数据处理管线逐步解析

对应论文 III-B，代码 `src/qr_detect.hpp`（main 与 fast-calib2 完全一致），输出：相机系下 4 个孔心 **P_C**。
核心思想：**相机自始至终不在图像里找圆孔**——ArUco 码定位靶板位姿，孔心由板坐标系下**已知的 CAD 布局**几何推算。

## 4.1 前提：靶标的已知几何（构造函数 :32-53 与 detect_qr 开头）

```cpp
// qr_detect.hpp:42-47  相机模型
cameraMatrix_ = [fx 0 cx; 0 fy cy; 0 0 1];
distCoeffs_   = [k1, k2, p1, p2, 0];            // 径向 k1,k2 + 切向 p1,p2

// :50  ArUco 字典
dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
```

板的 CAD 参数（yaml:43-48，均为物理制造值）：

| 参数 | 值 | 含义 |
|---|---|---|
| `marker_size` | 0.20 m（示例数据实为 0.16） | ArUco 码边长 |
| `delta_width_qr_center` | 0.55 m | 两码中心水平距离的**一半** |
| `delta_height_qr_center` | 0.35 m | 两码中心竖直距离的**一半** |
| `delta_width_circles` | 0.5 m | 两孔心水平间距 |
| `delta_height_circles` | 0.4 m | 两孔心竖直间距 |
| `circle_radius` | 0.12 m | 孔半径 |

## 4.2 步骤 1：生成板坐标系下的已知 3D 点（:116-149）

以板中心为原点、板面为 z=0 平面：

- 4 个码的角点（每码 4 角，共 16 个 3D 点）：码中心位于 $(\pm 0.55, \pm 0.35, 0)$，角点再偏 $\pm marker\_size/2$（:123-145）；
- **4 个孔心**（:133-135）：

$$p^{board} \in \{(\pm 0.25, \pm 0.20, 0)\} \quad (\pm\,delta\_width\_circles/2,\ \pm\,delta\_height\_circles/2)$$

- 码 ID 顺序 `{1, 2, 4, 3}`（:147，注意与几何位置顺序不同，注释有明确说明），构建 `cv::aruco::Board`。

## 4.3 步骤 2：ArUco 检测（:151-168）

```cpp
parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;  // 亚像素角点精化
cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters);
```

输出每个检出码的 4 个**亚像素级角点像素坐标** + ID。接受条件：检出数在 `[min_detected_markers_, 4]` 内，`min_detected_markers_ = 3`（common_lib.h:86），即**最少 3 个码也能算**（第 4 个码被遮挡时仍可标定）。

## 4.4 步骤 3：逐码 PnP 求位姿（:181-182）

```cpp
cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix_, distCoeffs_, rvecs, tvecs);
```

PnP（Perspective-n-Point）数学形式：已知 3D 点 $X_i$ 与像素观测 $(u_i, v_i)$，求 $[R|t]$ 最小化重投影误差

$$\min_{R,t} \sum_i \left\| \begin{bmatrix} u_i \\ v_i \\ 1 \end{bmatrix} - \frac{1}{s_i} K [R|t] \begin{bmatrix} X_i \\ 1 \end{bmatrix} \right\|^2$$

每个码 4 个角点（3D 由码边长定尺度）→ 一个独立位姿 $(rvec_i, tvec_i)$。

## 4.5 步骤 4：多码位姿平均（:185-211）

单码只有 4 个角点且远离相机，位姿噪声不小；多码独立解位姿后**平均**降噪。难点在旋转平均——欧拉角直接平均会被 $\pm\pi$ 回绕破坏，代码用 **sin/cos 分量平均**（角度圆均值）：

$$\bar t = \frac{1}{m}\sum_i t_i, \qquad \bar r_j = \mathrm{atan2}\Big(\tfrac{1}{m}\sum_i \sin r_j^{(i)},\ \tfrac{1}{m}\sum_i \cos r_j^{(i)}\Big), \quad j \in \{x,y,z\}$$

```cpp
// qr_detect.hpp:194-211
tvec[0..2] += tvecs[i][0..2];                 // 平移累加
rvec_sin[j] += sin(rvecs[i][j]);  rvec_cos[j] += cos(rvecs[i][j]);   // 旋转按 sin/cos 累加
tvec = tvec / ids.size();
rvec[j] = atan2(rvec_sin[j], rvec_cos[j]);    // 平均后还原
```

## 4.6 步骤 5：estimatePoseBoard 联合精化（:219-225）

```cpp
int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_, distCoeffs_, rvec, tvec, true);
```

以平均位姿为**初值**，把所有检出码的全部角点作为一组 3D-2D 对应，做一次联合 PnP 精化（内部等效多点 solvePnP）。这比单纯平均更准——平均只是初值稳健，联合优化才利用了"同一块刚体板"的全部约束。

## 4.7 步骤 6：孔心从板系变换到相机系（:232-271）

```cpp
cv::Rodrigues(rvec, R);                            // 旋转向量 → 3×3 旋转矩阵
board_transform = [R | t]  (3×4);                  // :241-243
// 对 4 个孔心 p_board = (±0.25, ±0.20, 0)：
mat_qr = board_transform * [p_board; 1];           // :254   齐次变换
// → center3d 存入 candidates_cloud
```

$$p^C = R^C_{board}\, p^{board} + t^C_{board}$$

调试可视化：把推算的孔心用 `projectPointDist`（带畸变，:55-64）回投到图像画绿点，人工确认几何一致（qr_detect.png）。

## 4.8 步骤 7：几何一致性筛选（:289-343）

与 LiDAR 侧完全相同的 `comb + Square` 检查（宽 0.5 × 高 0.4 矩形布局）。代码注释（:273-288）说明：此环节理论上不可能有多个候选集，保留它是为了防"偶发的误检测"，并与其它模式保持一致。

最终输出 4 点 → **P_C**，返回 main.cpp。

---

# 五、外参配准：Kabsch / SVD 闭式解

## 5.1 对应关系怎么建立：sortPatternCenters（common_lib.h:320-388）

SVD 配准要求两侧点集**顺序一一对应**（P_L 的第 i 点 ↔ P_C 的第 i 点）。检测过程不保证顺序，代码用确定性的几何排序统一：

1. **轴系统一**（:332-342）：LiDAR 系与相机系朝向约定不同，先把 LiDAR 点临时变换到相机轴序（`x=−y, y=−z, z=x`），排序后再变换回去（:378-387）；
2. **质心 + 极角排序**（:346-359）：算 4 点质心，`atan2` 按相对质心的方位角排序；
3. **手性修正**（:367-375）：`v01 × v12` 的 z 分量判断绕向，若非逆时针交换 `v[1]` 与 `v[3]`，保证两侧排序方向一致。

## 5.2 Kabsch 法原理（对应问题 5）

**问题**（已知对应关系的刚体配准，1976 年 Kabsch 提出于蛋白质结构比对）：给定 $N$ 对对应点 $\{p_i\}$(源/LiDAR) 与 $\{q_i\}$(目标/相机)，求

$$\min_{R,t} \sum_{i=1}^{N} \lVert R\,p_i + t - q_i \rVert^2$$

**闭式解四步**：

1. **去质心**（解耦平移）：$p'_i = p_i - \bar p,\ q'_i = q_i - \bar q$；
2. **3×3 交叉协方差矩阵**：$H = \sum_i p'_i {q'_i}^{\mathsf T}$；
3. **SVD 分解** $H = U\Sigma V^{\mathsf T}$，最优旋转

$$R = V\,D\,U^{\mathsf T}, \qquad D = \mathrm{diag}\big(1,\,1,\,\mathrm{sign}\,\det(VU^{\mathsf T})\big)$$

   $D$ 的末元素是**反射矫正项**：$\det(VU^{\mathsf T})=-1$ 时翻转第三个奇异方向，保证解是纯旋转（$\det R = +1$）而非镜像；
4. **回代平移**：$t = \bar q - R\,\bar p$。

**关键性质**：全局最优、无初值依赖、不迭代、复杂度 O(N)（3×3 SVD 是常数运算）——与 ICP 的本质区别是 ICP 用于"对应未知"场景，需交替"最近邻匹配 + Kabsch 求解"；FAST-Calib 中孔的对应天然已知，一次 Kabsch 即达全局最优。这正是配准阶段仅 2–3 μs 的原因。

## 5.3 论文目标函数（式 5）

$$E = \frac{1}{4N} \sum_{i=1}^{4N} \big\| p_i^C - T_{CL}\cdot p_i^L \big\|^2$$

单场景 N=1（4 对点）；多场景 N 次采集共 4N 对，仍是同一个闭式问题——多场景联合优化在数学上只是"往 H 矩阵里多堆点"。

## 5.4 单场景代码：PCL 封装直接调用（main.cpp:70-72）

```cpp
Eigen::Matrix4f transformation;
pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);
// 输出 T_cam_lidar：4×4，把 LiDAR 系点变到相机系
```

`TransformationEstimationSVD` 内部即上述 Kabsch/Umeyama SVD 流程（不含缩放）。

## 5.5 多场景代码：手写加权 Kabsch 逐行解析（multi_scene.cpp:28-80）

```cpp
// ① 加权质心（:43-50）
muL = Σ w_i·l_i / Σ w_i;   muC = Σ w_i·c_i / Σ w_i;
// ② 加权交叉协方差（:52-58）
Sigma = Σ w_i · (l_i − muL) · (c_i − muC)ᵀ;
// ③ SVD + 反射矫正（:60-69）
Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, ComputeFullU | ComputeFullV);
R = V * Uᵀ;
if (R.determinant() < 0) { D(2,2) = −1; R = V * D * Uᵀ; }   // 与 Kabsch 的 D 矩阵等价
// ④ 平移回代（:70）
t = muC − R · muL;
// ⑤ 加权 RMSE（:72-78）用于自检输出
```

当前调用 `SolveRigidTransformWeighted(L, C, nullptr)`（:194）权重全为 1，加权接口为后续按精度赋权预留。

## 5.6 多场景联合标定流程（multi_scene.cpp:110-229）

1. 每个场景跑一次单场景标定，`saveTargetHoleCenters`（common_lib.h:227-262）向 `output/circle_center_record.txt` **追加**三行：`time:` + `lidar_centers: {x,y,z}×4` + `qr_centers: {x,y,z}×4`；
2. `multi_fast_calib` 节点读该文件，正则解析（`parseCentersLine`, :82-108），按三行一组切 block；
3. **取最后 3 个 block**（:167-178，即最近 3 次采集 = README 要求的前/右/左三个姿态），拼成 **12 对点**；
4. 一次加权 Kabsch 求解 → 打印 RMSE → 写 `multi_calib_result.txt`（同 FAST-LIVO2 格式 Rcl/Pcl）。

> 为什么多场景能提精度：单场景 4 点共面（z=0 平面内），旋转绕板法线的分量约束弱；3 个不同朝向的板位姿让 12 个点在 3D 空间非共面分布，6 自由度全部强约束。

---

# 六、五个核心问题的代码级结论

> 把前几轮问答的结论与代码对照，标明"论文说法"与"代码实现"的异同。

## 6.1 Velo2Cam 对比（问题 1）→ 详见 §7；代码层面的补充

- FAST-Calib **直接沿用 Velo2Cam 的靶标设计**（README:67 附录声明 based on velo2cam_calibration），`comb`/`Square` 组合校验也继承自 velo2cam 代码（common_lib.h:427-430 注释自述"放宽了原版过于严格的校验"）；
- 差异在特征提取：Velo2Cam 逐扫描环找深度不连续；FAST-Calib 固态路径用**与扫描模式无关的 2D 边界角间隙法**（BoundaryEstimation，§3.5）；
- 有趣的是：代码的**机械式路径**（:108-165）反而保留了 Velo2Cam 式的 ring 距离跳变思想做边缘候选——即"机械雷达用旧思路够用，固态雷达才是新算法的主战场"。

## 6.2 LIO 聚合稠密点云（问题 2）→ §2.2

代码本身不做聚合（输入就是聚合好的 bag），但 `DataPreprocess` 的判型设计（CustomMsg→固态）印证了主线场景是固态雷达；16 线机械雷达的数据在采集端用 FAST-LIVO2 LIO 模式 + 轻微俯仰运动预聚合。

## 6.3 直通滤波（问题 3）→ §3.1

三轴串行 `pcl::PassThrough`（lidar_detect.hpp:335-351 / :70-86），区间来自 yaml，允许漏点（RANSAC 兜底），可用 `distance_filter_tool.py` 自动生成参数。

## 6.4 相机孔心求解（问题 4）→ §4

完整链路：ArUco 检测（DICT_6X6_250 + 亚像素精化，最少 3 码可用）→ 逐码 PnP（estimatePoseSingleMarkers）→ tvec 算术平均 + rvec sin/cos 圆均值 → estimatePoseBoard 联合精化 → 板布局 $(\pm0.25, \pm0.20, 0)$ 经 $[R|t]$ 变换为 P_C → Square 校验。论文只写了"平均位姿"，代码多了 Board 精化一步（§1.4 差异 #6）。

## 6.5 Kabsch 法（问题 5）→ §5.2

去质心 → $H=\Sigma p'q'^{\mathsf T}$ → SVD → $R=VDU^{\mathsf T}$（反射矫正）→ $t=\bar q-R\bar p$。代码两处实现：单场景调 PCL `TransformationEstimationSVD`（main.cpp:71）；多场景手写加权版（multi_scene.cpp:28-80）。

---

# 七、FAST-Calib 与 Velo2Cam 对比

## 7.1 设计原理对比

| 维度 | Velo2Cam（Beltrán 等, IEEE T-ITS 2022） | FAST-Calib |
|---|---|---|
| 靶标 | 3D 结构板（圆孔 + ArUco） | **同一设计**（直接沿用，CAD 公开） |
| 孔心定位 | 沿每条**扫描环**逐环检测深度不连续 | 固态：2D 角度间隙边界法（扫描模式无关）；机械：ring 距离跳变 + 统一后端 |
| 扫描模式依赖 | 强（需足够密扫描线穿过孔） | 弱（固态路径完全无关） |
| 固态雷达 | 理论只适用多线机械式；适配 Avia/Mid360 仍失败 | 原生支持 |
| 光斑膨胀 | 无处理 | 论文：椭圆拟合；代码：已知半径先验收紧（误差 <2.5 cm / 半径 ±3 cm） |
| ROI 提取 | 任意摆放需人工精细调深度滤波 | 直通滤波粗框 + RANSAC 兜底，全自动 |
| 多场景 | 常需多次采集逐次处理 | `circle_center_record.txt` 累积 + 12 点对一次 Kabsch |
| 速度 | 多步骤、无法秒级 | 全流程 < 0.7 s |
| 精度评估方式 | — | RMSE + 投影上色点云人工质检（colored_cloud.pcd） |

## 7.2 论文 Table I 实测（残差单位 cm，5 次实验，Run5 = 4 对数据全量联合优化）

| 传感器 | FAST-Calib | Velo2Cam |
|---|---|---|
| Avia | 0.15 / 0.22 / 0.38 / 0.65 / 0.25 | 15.2 / 10.9 / 16.4 / 12.0 / 16.6 |
| Mid360 | 0.23 / 0.14 / 0.15 / 0.10 / 0.16 | **全部失败（×）** |
| Ouster OS1-128 | 0.29 / 0.17 / 0.21 / 0.30 / 0.24 | 0.27 / 0.22 / 0.24 / 0.31 / 0.24 |

结论：**128 线机械雷达上两者相当；固态雷达上 Velo2Cam 因光束稀疏/不规则而劣化甚至完全失效**（Mid360 仅 4 个 EEL 激光发射器，扫描环上没有足够点穿过圆孔）。论文所有配置点对点残差 < 6.5 mm。

## 7.3 耗时对比（论文 Table II，i7-10700K，4 对数据联合）

| 阶段 | Avia | Mid360 | Ouster |
|---|---|---|---|
| LiDAR 数据处理（ROI + 孔提取） | 0.633 s | 0.675 s | 0.647 s |
| 相机数据处理 | 0.031 s | 0.018 s | 0.028 s |
| 配准（Kabsch） | 2.3 μs | 2.2 μs | 3.1 μs |
| **总计** | **0.664 s** | **0.693 s** | **0.675 s** |

快的原因：只在降采样后的滤波点云上操作；边缘提取在 2D 平面进行；配准是 O(1) 级闭式解；多数据对可并行。

---

# 八、实操指南

## 8.1 环境与编译

- 依赖：ROS1 catkin、PCL ≥ 1.8（CMakeLists 实际要求 ≥ 1.10）、OpenCV ≥ 4.0（含 aruco 模块）、livox_ros_driver；
- 标准 catkin 工作空间：

```bash
cd ~/calib_ws/src && git clone https://github.com/hku-mars/FAST-Calib.git
cd ~/calib_ws && catkin_make
source devel/setup.bash
```

## 8.2 单场景标定

1. 数据放 `calib_data/`（一个 bag + 一张对应 jpg）；
2. 改 `config/qr_params.yaml`：
   - **内参**：yaml 注释里附 mid360 / avia / ouster / multi-scene 四组实测值，取消注释对应那组即可（当前生效的是 multi-scene 组，yaml:33-40）；
   - **距离滤波**：框住靶标即可（参考 yaml:51-56 的推荐值，或用脚本生成）；
   - `lidar_topic`：`/livox/lidar` 或 `/ouster/points`；
3. 运行：

```bash
roslaunch fast_calib calib.launch
```

终端打印 `RMSE` 与 `T_cam_lidar`；`output/` 生成：

| 文件 | 内容 |
|---|---|
| `single_calib_result.txt` | **FAST-LIVO2 标定格式**：cam_model/cam_fx…/cam_d0..d3 + `Rcl: [...]` + `Pcl: [...]`（common_lib.h:278-298），可直接给 FAST-LIVO2 用 |
| `colored_cloud.pcd` | 按外参投影上色的点云（定性质检，看孔边缘是否对齐图像） |
| `qr_detect.png` | ArUco 检测 + 孔心回投调试图（绿点应落在图像孔位上） |
| `circle_center_record.txt` | 追加式中间记录（多场景标定输入） |

## 8.3 多场景联合标定

1. 按 README 摆 3 个场景：靶标**朝前 / 朝右 / 朝左**，各采一对 bag+image；
2. 每个场景改 yaml 的 bag_path/image_path/滤波区间后各跑一次 §8.2（记录自动累积）；
3. 联合求解：

```bash
roslaunch fast_calib multi_calib.launch
# 读 circle_center_record.txt 最后 3 组 → 12 点对加权 Kabsch → output/multi_calib_result.txt
```

> 注意：程序取的是**最后 3 组**记录（multi_scene.cpp:167-178）。若历史记录混入了旧传感器数据，先清空 `circle_center_record.txt`。

## 8.4 靶标摆放距离与场景个数的选取

针对"LiDAR 水平 360°、垂直各线角度不同、与相机重叠区域可能不大"的实际顾虑，结论先行：**靶标距离 2~5 m（推荐 3 m 附近），场景数 ≥3 个（板朝前/朝右/朝左三种摆放）**。重叠视场不需要大，只要整块板（约 1.4 m × 1.0 m）完整落在两传感器的共同可见区内即可。

### 8.4.1 距离为什么是 2~5 m

官方实测依据即 `config/qr_params.yaml:51-56` 的距离滤波推荐值：**Avia/Mid360 x_min=2.0、Ouster x_min=1.5，x_max 统一 5.0**。这个区间由两侧传感器的采样约束共同夹出：

**LiDAR 侧——孔必须被点"描满"。** 靶板上的点间距 ≈ 距离 × 角分辨率，而孔直径仅 0.24 m（半径 0.12 m）：

| 雷达 | 角分辨率 | 3 m 处点间距 | 24 cm 孔内采样 |
|---|---|---|---|
| Ouster OS1-128 | 水平 ~0.18° | ~9 mm | ~26 点，充裕 |
| Avia / Mid360（非重复扫描，录 3~5 s 融合） | 积分密度随时间提高 | 数毫米级 | 充裕 |
| 16 线机械雷达 | 垂直 ~2° | 线间距 ~10 cm | 每孔仅 2 条线扫过，必须 LIO+俯仰聚合 |

经验准则：孔直径内 ≥10~20 个点、点间距 ≤2~3 cm，反推距离上限约 5~6 m（高角分辨率雷达）——这正是 yaml 上限取 5.0 的原因，再远孔就"描"不圆了。低线数机械雷达要么更近，要么必须运动聚合（§2.2）。

**相机侧——ArUco 码要占够像素。** 板位姿由 PnP 解出，平移误差大致 ∝ 距离 / 码的像素宽度，而码的像素宽度 = $marker\_size \cdot f_x / d$。以 0.16 m 码、$f_x \approx 1200$ px 估算：

| 距离 d | 码宽像素 | 评价 |
|---|---|---|
| 3 m | ~65 px | 亚像素角点精化后位姿很稳 |
| 5 m | ~39 px | 可用 |
| 8 m | ~24 px | 板位姿噪声明显放大，孔心推算随之劣化 |

准则：码宽 ≥30~40 px。

**为什么不能更近（<1.5~2 m）**：整块板需完整进入相机画面且避开广角畸变最大的画面边缘；Livox 有最近量程限制；过近时板也容易贴近距离滤波框边界，留给"粗框 + RANSAC 兜底"策略（§3.1）的余量变小。

### 8.4.2 重叠视场：不求大，只求"板完整"

- **机械式 360° 雷达 + 前向相机**：水平方向永远重叠，真正要留意的是**垂直 FoV**（如 OS1 垂直 45°）与相机俯仰安装角的交集——把板中心放在与传感器大致等高的位置即可；
- **固态雷达（Avia 70.4°×77.2° 锥形 FoV）**：板必须完整进入锥体内且相机同时看全。布置时自检：图像里 4 个 ArUco 都清晰（代码允许最少 3 个，`min_detected_markers=3`，common_lib.h:86，但 4 个更稳），rviz 点云里 4 个孔的边缘都采到；
- 量级感：1.4×1.0 m 的板在 3 m 处张角约 26°×19°——任何前向相机与 360° 雷达的公共区域都放得下。

### 8.4.3 场景个数：3 个是下限，也是代码的固定取数

- **单场景**可出结果（`single_calib_result.txt`），但 4 个孔心**共面**，绕板法线方向的旋转分量约束弱，精度与稳定性打折；
- **多场景联合**：README 要求至少 3 个场景；`multi_scene.cpp:167-178` 硬编码**只取记录文件的最后 3 组**（3 场景 × 4 孔 = 12 点对）一次 Kabsch 求解。板朝前/朝右/朝左三种摆放的本质是让孔心点在三维空间**不共面**，使 6 自由度全部获得强约束（§5.6）；
- 论文每组用 4 对数据（Table I 中 Run5 = 4 对全量联合），一致性评估显示任取 3 对组合的结果方差已很低——**3 个高质量场景优于更多个平庸场景**，超过 3 个收益递减；
- 注意：代码只用最后 3 组，多录的场景不会被用到；换设备/重新标定前先清空 `output/circle_center_record.txt`。

## 8.5 自制传感器套件 checklist

- [ ] 靶标加工：4 圆孔（半径 0.12 m、孔距 0.5×0.4 m）+ 4 个 ID 为 **1/2/4/3** 的 6×6 ArUco 码（CAD 见 README:50）；
- [ ] 相机内参 + 畸变预先标定（k1 k2 p1 p2）；
- [ ] yaml 填内参与靶标实际尺寸（尤其 `marker_size` 要按实测改）；
- [ ] 靶标置于 2~5 m（推荐 ~3 m）、板中心与传感器大致等高、整板落在重叠 FoV 内（准则详见 §8.4）；
- [ ] 距离滤波框住靶标（`distance_filter_tool.py` 生成）；
- [ ] 稠密点云：固态直接多帧融合；16 线机械雷达用 LIO + 轻微俯仰聚合；
- [ ] 单场景验证 RMSE（米制，合格应在毫米~厘米级）+ 看 colored_cloud.pcd 是否色彩对齐；
- [ ] 多场景（前/右/左）联合出最终结果。

---

# 九、附录

## 9.1 公式速查表

| 名称 | 公式 | 出处 |
|---|---|---|
| 邻域方向角 | $\theta_j = \mathrm{atan2}\big((p_j-p_i)_y,(p_j-p_i)_x\big)$ | 论文式(1)；PCL boundary 的 $\alpha_k=\mathrm{atan2}(v\cdot\Delta p,\,u\cdot\Delta p)$ |
| 最大角间隙 | $\Delta\theta_N = \theta_1 + 2\pi - \theta_N$（含闭环） | 论文式(2)；判据：论文 >25° / 代码 >π/4 |
| 平面距离 | $\mathrm{dist}(p)=\lvert ax_p+by_p+cz_p+d\rvert/\sqrt{a^2+b^2+c^2}$ | RANSAC 内点判据（阈值 0.01） |
| 平面对齐旋转 | $\mathbf{a}=\mathbf{n}\times\mathbf{z},\ \theta=\arccos(\mathbf{n}\cdot\mathbf{z}),\ R=\exp([\mathbf{a}]_\times\theta)$ | lidar_detect.hpp:388-392 |
| 逆变换还原 | $p_{origin}=R^{-1}\,(x_c, y_c, \bar z)^{\mathsf T}$ | lidar_detect.hpp:493-494 |
| 一般二次曲线 | $Ax^2+Bxy+Cy^2+Dx+Ey+F=0$，椭圆约束 $B^2-4AC<0$ | 论文式(3)（代码未实现椭圆版） |
| 椭圆中心 | $x_c=\frac{2CD-BE}{B^2-4AC},\ y_c=\frac{2AE-BD}{B^2-4AC}$ | 论文式(4) |
| 代数圆拟合 | $[x\ y\ 1]\,[a\ b\ c]^{\mathsf T}=-(x^2+y^2)$；$c_x=-a/2,\ c_y=-b/2,\ r=\sqrt{c_x^2+c_y^2-c}$ | fast-calib2 fitCircleRobust |
| Huber 权重 | $w=\lvert r\rvert\le\delta\ ?\ 1:\delta/\lvert r\rvert$（$\delta=0.02$） | fast-calib2 :571/:586 |
| PnP | $\min_{R,t}\sum_i\big\|[u_i,v_i,1]^{\mathsf T}-\tfrac{1}{s}K[R\lvert t][X_i,1]^{\mathsf T}\big\|^2$ | estimatePoseSingleMarkers 内部 |
| 旋转圆均值 | $\bar r_j=\mathrm{atan2}\big(\tfrac1m\sum_i\sin r_j^{(i)},\ \tfrac1m\sum_i\cos r_j^{(i)}\big)$ | qr_detect.hpp:209-211 |
| 板系→相机系 | $p^C=R^C_{board}p^{board}+t^C_{board}$，$p^{board}\in\{(\pm0.25,\pm0.20,0)\}$ | qr_detect.hpp:254 |
| 配准目标 | $E=\frac{1}{4N}\sum_i^{4N}\lVert p_i^C-T_{CL}p_i^L\rVert^2$ | 论文式(5) |
| Kabsch | $H=\Sigma p'q'^{\mathsf T}$；$H=U\Sigma V^{\mathsf T}$；$R=VDU^{\mathsf T}$，$D=\mathrm{diag}(1,1,\mathrm{sign}\det(VU^{\mathsf T}))$；$t=\bar q-R\bar p$ | common_lib / PCL TE-SVD；multi_scene.cpp:52-70 |

## 9.2 关键参数对照表（论文 / main / fast-calib2）

| 参数 | 论文 | main | fast-calib2 |
|---|---|---|---|
| 平面 RANSAC 内点阈值 | 0.01 m | 0.01（:369/:98） | 同 |
| 体素 leaf | 8 mm（平面分割后） | 5 mm（平面分割前，固态） | ROI 10 mm / 圆环 5 mm |
| 边缘邻域半径 | 0.03 m | 0.03（:412/:419） | 改高反强度法（平面距 <0.03） |
| 边缘角间隙阈值 | 25° | π/4 = 45°（:420） | 不适用 |
| 聚类容差/点数 | — | 0.05 / 50~1000（:436-438） | 0.02 / 200~50000 |
| 圆拟合 | 椭圆（Fitzgibbon） | Circle2D RANSAC：固态 0.01/误差<0.025；机械 0.02/半径±0.03 | 代数+Huber GN：\|r−0.12\|<0.05、err<0.03 |
| 几何校验容差 | — | GEOMETRY_TOLERANCE = 0.08（相对值） | 6 边距离平方和最小化 + validateTargetGeometry |
| 最少 ArUco 数 | — | 3 / 4（min_detected_markers） | 同 |
| 多场景点对数 | 4N | 12（3 场景 × 4） | 同 |

## 9.3 参考链接

- 论文：https://arxiv.org/abs/2507.17210
- 仓库：https://github.com/hku-mars/FAST-Calib （分支：`main`、`fast-calib2`）
- Velo2Cam：Beltrán 等, *Automatic Extrinsic Calibration for 2D-LIDAR and Camera...*, IEEE T-ITS 23(10), 2022；代码 https://github.com/beltransen/velo2cam_calibration
- ArUco：Garrido-Jurado 等, Pattern Recognition 47(6), 2014（OpenCV `cv::aruco`，DICT_6X6_250）
- Fitzgibbon 椭圆拟合：*Direct Least Squares Fitting of Ellipses*, ICPR 1996
- Kabsch：W. Kabsch, *A solution for the best rotation to relate two sets of vectors*, Acta Cryst. A, 1976
- FAST-LIVO2：Zheng 等, IEEE T-RO, 2024（本工具的服务对象；输出格式兼容）
- PCL BoundaryEstimation 源码：https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/impl/boundary.hpp

---

*文档生成于 2026-08-28；基于本地克隆的 FAST-Calib（main @ 克隆时最新、fast-calib2 @ FETCH_HEAD）与论文 arXiv v1 逐行核对。若后续仓库更新，行号可能漂移，函数名与参数值以仓库为准。*
