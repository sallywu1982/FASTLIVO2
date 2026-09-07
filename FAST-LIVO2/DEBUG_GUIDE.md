# FAST-LIVO2 C++ 代码调试操作手册

更新日期：2026-08-23。本文档按步骤编号，每步均可直接复制粘贴执行。

**适用环境**：WSL2 `Ubuntu-20.04` + ROS1 Noetic
- 工作空间：`/root/catkin_ws`（含 FAST-LIVO2、livox_ros_driver、vikit_common）
- 可执行节点：`/root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping`
- 数据：`/root/fast_livo2_data/`（`Bright_Screen_Wall.bag`、NTU VIRAL `eee_03.bag` 已跑通）

---

## 〇、环境现状：哪些已配好，哪些待做

以下配置 **2026-08-23 已全部完成并验证**，可直接从"第 0 步"开始：

| 项目 | 状态 | 位置 |
|---|---|---|
| gdb 9.2 / gdbserver | ✅ 已安装 | WSL 内 |
| VSCode WSL Remote 扩展 | ✅ 已安装（v0.104.3，Windows 端） | — |
| 调试配置 launch.json | ✅ 已创建 | `/root/catkin_ws/.vscode/launch.json` |
| Eigen gdb printer | ✅ 已下载并注册 | `/root/eigen_printers.py` + `~/.gdbinit` |
| ptrace_scope | ✅ 已设为 0 | **WSL 重启后失效，见第 2.4 节重设** |
| Debug 编译 | ❌ 待做 | 第 0 步 |
| VSCode 连接 WSL + 装 C/C++ 扩展 | ❌ 待做（图形界面操作） | 第 1 步 |

---

## 一、第 0 步：Debug 编译（一次性，约几分钟）

默认编译不带调试符号，断点处变量会显示 `value optimized out`。**这是断点调试的必要条件。**

```bash
# 1. 进入 WSL
wsl -d Ubuntu-20.04

# 2. 编译（在 WSL 内执行）
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug
```

**如何确认成功**（编译完执行）：

```bash
file /root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping
# 预期输出包含 "with debug_info, not stripped"

readelf -S /root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping | grep debug
# 预期看到 .debug_info .debug_line 等多个 section
```

**切回 Release（测性能时必须，Debug 的耗时不代表真实性能）**：

```bash
cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

> `CMAKE_BUILD_TYPE` 会缓存在 `build/CMakeCache.txt`，切换后需要重编才会生效。

---

## 二、第 1 步：VSCode 连接 WSL（图形界面操作，一次性）

1. 打开 Windows 的 VSCode；
2. 点击**左下角绿色「远程窗口」按钮** → 选 **「连接到 WSL」**（若提示选发行版，选 `Ubuntu-20.04`）；
   - 首次连接会自动在 WSL 内安装 VSCode Server，等 1~2 分钟；
   - 等价命令行方式：在 WSL 终端里执行 `code /root/catkin_ws`；
3. 连接成功后（左下角显示 `WSL: Ubuntu-20.04`）：**文件 → 打开文件夹 → 输入 `/root/catkin_ws` → 确定**；
4. 按 `Ctrl+Shift+X` 打开扩展面板，搜索 `C/C++`，在 **ms-vscode.cpptools** 一项上点 **「在 WSL 中安装 / Install in WSL」**。
   - Windows 端装过也需要再点这一次，远程端是独立安装的。

**如何确认成功**：扩展面板中 C/C++ 一项显示"已安装在 WSL: Ubuntu-20.04 中"；且打开 `src/FAST-LIVO2/src/LIVMapper.cpp` 时代码有颜色高亮、函数可跳转。

---

## 三、第 2 步：启动一次完整的断点调试

### 2.1 启动节点和回放（WSL 里开两个终端）

```bash
# 终端1：启动节点（WSL 无显示，rviz:=false）
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_bright_screen_wall.launch

# 终端2：慢速回放（调试时必须慢放，否则节点停住时数据会堆积）
rosbag play --clock -r 0.3 /root/fast_livo2_data/bright_screen_wall/Bright_Screen_Wall.bag
```

其它数据集的启动命令（任选）：

```bash
# NTU VIRAL
roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false
rosbag play --clock -r 0.3 /root/fast_livo2_data/eee_03/eee_03.bag
```

### 2.2 attach 调试器

1. 在 VSCode 里打开源文件（如 `src/FAST-LIVO2/src/LIVMapper.cpp`），**在行号左侧单击**设断点（首次建议：`LIVMapper.cpp` 第 267 行 `stateEstimationAndMapping`，或任一回调函数）；
2. 按 **F5** → 选 **"Attach to fastlivo_mapping"**；
3. 弹出进程选择框：输入 `fastlivo` → 选中 `fastlivo_mapping` → 回车。

**如何确认成功**：底部状态栏变成橙色/显示调试工具条；顶部出现调试控制条（继续/单步/停止按钮）。下一帧数据到来时节点停在断点上，左侧面板显示局部变量和调用栈。

### 2.3 单步与查看

| 按键/操作 | 作用 |
|---|---|
| F5 | 继续运行到下一个断点 |
| F10 | 单步跳过（不进入函数内部） |
| F11 | 单步进入 |
| Shift+F11 | 跳出当前函数 |
| 左侧 VARIABLES 面板 | 展开查看局部变量（Eigen 矩阵直接显示数值） |
| WATCH 面板 | 添加表达式持续监视，如 `state.pos` |
| 调用栈面板 | 点不同栈帧切换上下文 |
| 红色断开按钮 | 结束调试（**不会杀掉节点**，可反复 F5 重新 attach） |

### 2.4 ptrace 权限（attach 报错时看这里）

若 attach 报 `ptrace: Operation not permitted`，在 WSL 里执行：

```bash
echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
```

> 此设置 **WSL 重启后还原**。`wsl --shutdown` 或重启电脑后第一次 attach 前都要重跑这条命令。可以把它写进 `~/.bashrc` 省事：
> ```bash
> echo 'echo 0 > /proc/sys/kernel/yama/ptrace_scope 2>/dev/null' | sudo tee -a /etc/profile.d/ptrace.sh
> ```

---

## 四、断点速查表（真实文件 + 行号 + 看什么）

按数据流顺序排列。行号基于当前源码版本（commit `0d2c034`，`src/vio.cpp` 有 11 行本地改动），如有偏移按函数名搜索。

### 4.1 入口与主状态机

| 位置 | 用途 / 看什么 |
|---|---|
| `src/main.cpp:3` `main` | 启动流程、参数加载 |
| `src/LIVMapper.cpp:267` `stateEstimationAndMapping` | **主调度状态机**，每帧 LIO/VIO 都经过这里；看当前是处理雷达帧还是图像帧 |
| `src/LIVMapper.cpp:884` `sync_packages` | 时间对齐/同步逻辑；排查"图像和雷达没对上"的问题 |

### 4.2 传感器回调（确认数据到达）

| 位置 | 用途 / 看什么 |
|---|---|
| `LIVMapper.cpp:703` `standard_pcl_cbk` | PointCloud2 雷达回调（NTU VIRAL 走这个）；看点数、时间戳 |
| `LIVMapper.cpp:726` `livox_pcl_cbk` | Livox CustomMsg 回调（官方数据集走这个） |
| `LIVMapper.cpp:769` `imu_cbk` | IMU 回调；看加速度/角速度原始值 |
| `LIVMapper.cpp:829` `img_cbk` | 图像回调；看图像时间戳 |

### 4.3 IMU 处理

| 位置 | 用途 / 看什么 |
|---|---|
| `LIVMapper.cpp:248` `processImu` | IMU 前向传播入口 |
| `LIVMapper.cpp:556` `prop_imu_once` | 单步传播；看传播后的姿态/速度 |
| `LIVMapper.cpp:576` `imu_prop_callback` | 高频 IMU 传播定时器 |
| `src/IMU_Processing.cpp` `IMUProcess` | 预积分、去畸变；排查去畸变错误 |

### 4.4 LIO 更新与 ESIKF 迭代（本仓库 ESIKF 为手写实现）

| 位置 | 用途 / 看什么 |
|---|---|
| `LIVMapper.cpp:336` `handleLIO` | LIO 流程入口 |
| `src/voxel_map.cpp:338` `StateEstimation` | **ESIKF 迭代核心**；看迭代次数、收敛的 `norm_dx`（diag_dx）、迭代结束后的状态修正量 |
| `src/voxel_map.cpp:643` `BuildResidualListOMP` | 残差列表构建 |
| `src/voxel_map.cpp:713` `build_single_residual` | 单点到面残差；看残差值、平面法向量，判断是不是坏点/退化平面 |

### 4.5 VIO 更新

| 位置 | 用途 / 看什么 |
|---|---|
| `LIVMapper.cpp:281` `handleVIO` | VIO 流程入口 |
| `src/vio.cpp:1787` `processFrame` | 单帧图像直接法主函数；看 patch 对齐、金字塔层数 |
| `src/vio.cpp:1521` `updateState` | VIO 的 ESIKF 更新；看视觉残差与迭代收敛 |
| `src/vio.cpp:1399` `updateStateInverse` | 逆深度参数化版本 |

### 4.6 体素地图

| 位置 | 用途 / 看什么 |
|---|---|
| `src/voxel_map.cpp:532` `BuildVoxelMap` | 初始建图 |
| `src/voxel_map.cpp:609` `UpdateVoxelMap` | 每帧地图更新；排查地图膨胀/漂移 |

---

## 五、进阶技巧

### 5.1 条件断点：只停在关心的帧

右键断点 → **编辑断点 / Edit Breakpoint** → **表达式 / Condition** 填：

```cpp
frame_id > 200        // 只在第 200 帧后停（按实际变量名，回调里常用时间戳变量）
```

gdb 命令行等价写法：`break vio.cpp:1787 if idx > 200`

### 5.2 跳过前面正常的数据，直接调到出问题的时间段

先用 Release 版正常跑一遍，记下出问题的 rosbag 时间（终端输出或 rviz 轨迹突变处），然后：

```bash
# 从第 40 秒开始播，不用干等
rosbag play --clock -s 40 -r 0.3 /root/fast_livo2_data/bright_screen_wall/Bright_Screen_Wall.bag

# 播放中按空格键：暂停 / 继续（暂停后单步最稳）
```

### 5.3 查看 Eigen 变量（已配置好，直接用）

```gdb
p state.pos          # Vector3d 直接显示三个数
p cov                # 矩阵显示为行列形式
p rot_matrix         # 任意 Eigen::Matrix
```

VSCode VARIABLES/WATCH 面板里同样直接展开看数值。若某次没加载，在 DEBUG CONSOLE（调试控制台）输入 `-exec source /root/eigen_printers.py` 手动加载。

### 5.4 查看点云/图像数据

- 点云：`p plvec_tran->size()`、循环内 `p plvec_tran->at(0)`；
- 图像（cv::Mat）：`p img.rows`、`p img.cols`、`p img.at<uchar>(100,100)`（灰度单像素）；想看整图最方便是在代码里临时 `cv::imwrite` 保存后查看。

---

## 六、命令行 GDB 方式（VSCode 的替代/补充）

### 6.1 attach 到运行中的节点

```bash
gdb -p $(pgrep -f fastlivo_mapping)
# 进入后按 c 继续跑；断点命中时停住
# 退出用 Ctrl+D 选 detach——不会杀掉节点
```

### 6.2 调试启动阶段（launch-prefix 方式）

编辑 launch 文件（如 `mapping_bright_screen_wall.launch`），给 node 标签加一行：

```xml
<node pkg="fast_livo" type="fastlivo_mapping" name="fastlivo_mapping"
      launch-prefix="gdb -ex run --args" output="screen" ... />
```

再次 `roslaunch` 后节点停在 gdb 提示符，输入 `run` 启动；崩溃时自动停住，`bt` 看堆栈。

### 6.3 常用命令速查

| 命令 | 作用 |
|---|---|
| `bt` / `bt full` | 崩溃堆栈（带局部变量） |
| `f 3` | 切到堆栈第 3 帧 |
| `p var` | 打印变量 |
| `display residual` | 每次停下自动显示 |
| `watch cov(0,0)` | 值变化即停 |
| `info threads` / `t 2` | 多线程切换 |
| `c` / `n` / `s` / `u` | 继续 / 下一行 / 步入 / 跳出 |

### 6.4 崩溃 core dump 分析（偶发崩溃无法交互复现时）

```bash
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=/root/corefiles/core.%e.%p

# 拿到 core 文件后
gdb /root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping core.fastlivo_mapping.<pid>
gdb> bt
```

---

## 七、故障排查表

| 现象 | 原因 / 解决命令 |
|---|---|
| attach 报 `ptrace: Operation not permitted` | `echo 0 \| sudo tee /proc/sys/kernel/yama/ptrace_scope`（WSL 重启后需重跑，见 2.4） |
| 变量显示 `value optimized out` | 没有 Debug 编译：`catkin_make -DCMAKE_BUILD_TYPE=Debug` 后重试 |
| 断点是灰色空心圆、打不上 | 改了代码没重编；或 Debug 版本与节点不一致 → 重新 catkin_make 并重启 roslaunch |
| 进程列表里找不到 fastlivo_mapping | 节点没起来：查 roslaunch 终端报错；确认 `roscore` 正常、launch 名拼写正确 |
| attach 后节点完全卡住不动 | 调试控制台输入 `-exec c` 继续；或 gdb 拦了信号：`-exec handle SIG64 nostop noprint pass` |
| 断点停住后 IMU 队列爆 / 报同步错误 | 回放降速 `-r 0.1`，或 rosbag 空格暂停后再单步 |
| Eigen 变量只显示一串内部字段 | `-exec source /root/eigen_printers.py`；仍不行用 `p v.data()[0]` 逐元素看 |
| 后台跑看不到 printf 输出 | stdout 全缓冲，被 kill 时丢失；用 `rostopic echo -n 1 /odometry_reg` 验证处理是否进行，或 roslaunch 加 `--screen` |
| `wsl -e cmd` 退出后 WSL 里进程全没了 | WSL 会话结束带走子进程；长时间调试保持一个 wsl 终端不关 |

---

## 八、推荐调试工作流

1. **平时**：Release 编译 + 正常速回放，观察轨迹与统计输出；
2. **发现问题**：记下 rosbag 时间点 → `catkin_make -DCMAKE_BUILD_TYPE=Debug`；
3. **定位**：`rosbag play --clock -s <秒> -r 0.3` 跳到现场 → F5 attach → 条件断点 → 用变量面板看 Eigen 数值；
4. **验证规律**：WATCH 面板监视关键变量 + rosbag 空格暂停单步；
5. **完成后**：切回 Release。
