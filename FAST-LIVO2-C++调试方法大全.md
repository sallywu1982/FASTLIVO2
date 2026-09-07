# FAST-LIVO2 C++ 调试方法大全

更新日期：2026-08-23

适用环境：WSL2 `Ubuntu-20.04` + ROS1 Noetic，工作空间 `/root/catkin_ws`，
可执行节点 `/root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping`，
数据目录 `/root/fast_livo2_data/`（`eee_03.bag`、`Bright_Screen_Wall.bag` 均已跑通）。

---

## 一、编译准备：先拿到带调试符号的可执行文件

默认 `catkin_make` 不带 `-g`，断点处会看到 `value optimized out`，必须先改成 Debug 编译。

```bash
# 进入 WSL
wsl -d Ubuntu-20.04
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 确认 gdb 已装
sudo apt install -y gdb gdbserver

# Debug 编译（-O0 不优化 + -g 带符号）
cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug
```

要点：

- `CMAKE_BUILD_TYPE=Debug` 会被缓存在 `build/CMakeCache.txt` 里，之后普通 `catkin_make` 也保持 Debug；想切回性能测试版本用 `catkin_make -DCMAKE_BUILD_TYPE=Release`（ESIKF 迭代耗时测量必须用 Release，否则数据不代表真实性能）。
- Debug 版本运行会明显变慢（实时性可能跟不上 rosbag 播放，用 `-r 0.2` 慢放即可，见第五节）。
- FAST-LIVO2 的 CMakeLists 默认 `CMAKE_BUILD_TYPE` 为空（无优化也无符号），所以 Debug 编译是断点调试的**必要条件**。

---

## 二、VSCode 图形化断点调试（推荐）

在 Windows 的 VSCode 里通过 Remote-WSL 连进 WSL2，获得 IDE 级调试体验：断点、单步、调用栈、变量监视。

### 2.1 环境搭建（一次性）

1. Windows 端 VSCode 安装扩展 **WSL**（Remote Development 套件里）。
2. 在 WSL 终端执行以下命令打开工作空间（`code` 命令由 WSL 扩展自动注入，首次会安装 VSCode Server）：

   ```bash
   code /root/catkin_ws
   ```

3. 在打开的 VSCode 里安装扩展 **C/C++**（ms-vscode.cpptools，选 "Install in WSL"）。

### 2.2 launch.json 配置

在 `/root/catkin_ws/.vscode/launch.json` 里写两种配置，日常用 attach 模式：

```jsonc
{
  "version": "0.2.0",
  "configurations": [
    {
      // 方式A（推荐）：attach 到已运行的节点。
      // roslaunch 负责加载 yaml 参数、话题重映射，调试器只管停在哪
      "name": "Attach to fastlivo_mapping",
      "type": "cppdbg",
      "request": "attach",
      "program": "/root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping",
      "processId": "${command:pickProcess}",
      "MIMode": "gdb",
      "setupCommands": [
        { "text": "set pagination off", "ignoreFailures": true }
      ]
    },
    {
      // 方式B：gdb 直接启动节点进程本身，适合调试启动早期/初始化阶段的代码。
      // 需要先在另一个终端 roscore，且参数要与 launch 文件传的一致，较繁琐
      "name": "Launch fastlivo_mapping (gdb)",
      "type": "cppdbg",
      "request": "launch",
      "program": "/root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping",
      "args": ["_config_path:=/root/catkin_ws/src/FAST-LIVO2/config/avia.yaml",
               "_camera_config_path:=/root/catkin_ws/src/FAST-LIVO2/config/camera_pinhole.yaml"],
      "cwd": "/root/catkin_ws",
      "environment": [{ "name": "ROS_MASTER_URI", "value": "http://localhost:11311" }],
      "MIMode": "gdb"
    }
  ]
}
```

### 2.3 Attach 调试流程

```bash
# 终端1：正常启动节点（rviz:=false，WSL 无显示）
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_bright_screen_wall.launch

# 终端2：慢速回放数据
rosbag play --clock -r 0.3 /root/fast_livo2_data/bright_screen_wall/Bright_Screen_Wall.bag
```

VSCode 里：F5 → 选 "Attach to fastlivo_mapping" → 输入进程名 `fastlivo` 选中 → 在源码里点行号设断点 → 节点停住后用 F10/F11 单步，左侧面板看局部变量和调用栈。

### 2.4 让 Eigen 变量可读（强烈建议）

FAST-LIVO2 里位姿、协方差全是 `Eigen::Vector3d / Matrix3d / MatrixXd`，gdb 原生打印不出来。装一个 gdb 的 Eigen pretty printer：

```bash
# WSL 里
wget -O /root/eigen_printers.py \
  https://raw.githubusercontent.com/rc/eigen-cpp-gdb-printers/master/eigen_printers.py
```

在 launch.json 两条配置的 `setupCommands` 里追加（或写进 `~/.gdbinit` 一劳永逸）：

```jsonc
"setupCommands": [
  { "text": "set pagination off", "ignoreFailures": true },
  { "text": "source /root/eigen_printers.py" }
]
```

之后 `p state.pos`、`p cov` 就能直接看到矩阵数值。没装时也可以临时用 `p pos.data()[0]` 逐元素看。

### 2.5 推荐断点位置（结合算法流程）

| 位置 | 文件 | 用途 |
|---|---|---|
| 雷达/IMU/图像回调入口 | `src/livo_node.cpp` 的 `lidar_cb` / `imu_cb` / `img_cb` | 确认数据进来、时间戳对齐 |
| 预积分/去畸变 | `src/pre_process.cpp` 的 `process` / UndistortPcl | IMU 预积分、点云去畸变问题 |
| LIO 观测更新 | `src/livo.cpp`、`src/voxel_map_util.cpp` 的 `build_plane` / estimator 部分 | 点到面残差、平面退化 |
| ESIKF 迭代 | `include/use_ikfom.hpp` 迭代收敛判断处 | 看迭代次数、`norm_dx` 收敛性 |
| VIO 更新 | `src/vio.cpp` 的 `process_image` / patch 对齐部分 | 图像对齐残差 |
| 地图/关键帧 | `src/voxel_map_util.cpp` 的地图更新 | 体素地图膨胀、关键帧选择 |

**条件断点**示例：右键断点 → Edit Breakpoint → Condition 填 `frame_count > 200`（按实际变量名）；gdb 命令行为 `break vio.cpp:123 if idx > 200`。调试回放数据时用条件断点直接跳到出问题的帧，比一次次按继续快得多。

---

## 三、命令行 GDB 调试

### 3.1 roslaunch 前缀方式（调试启动/初始化阶段）

编辑 launch 文件（如 `mapping_avia.launch`），给 node 加 `launch-prefix`：

```xml
<node pkg="fast_livo" type="fastlivo_mapping" name="fastlivo_mapping"
      launch-prefix="gdb -ex run --args" ... />
```

节点起来后停在 gdb 提示符里，输入 `run` 启动；崩溃时直接 `bt` 看堆栈。

### 3.2 attach 到运行中的节点（调试运行阶段，最常用）

```bash
# 节点已由 roslaunch 启动后
gdb -p $(pgrep -f fastlivo_mapping)
```

进入后按 `c` 继续跑，断点命中时停下来。**detach（Ctrl+D 选 detach）不会杀掉节点**，可以反复挂上/摘下。

### 3.3 常用命令速查

| 命令 | 作用 |
|---|---|
| `bt` / `bt full` | 崩溃堆栈（带局部变量） |
| `f 3` | 跳到第 3 帧 |
| `p var` | 打印变量 |
| `p pos.data()[0]` | Eigen 向量第 0 个元素（没装 printer 时的土办法） |
| `display residual` | 每次停下自动显示 |
| `watch cov(0,0)` | 值变化时停住 |
| `b vio.cpp:120 if frame_id>200` | 条件断点 |
| `info threads` / `t 2` | 查看多线程（ROS 回调 spinner 与主线程） |
| `c` / `n` / `s` / `u` | 继续 / 下一行 / 步入 / 跳出 |

### 3.4 崩溃 core dump 分析

```bash
# 开启 core（WSL 里默认可能受限）
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=/root/corefiles/core.%e.%p

# 用 gdb 分析
gdb /root/catkin_ws/devel/lib/fast_livo/fastlivo_mapping core.fastlivo_mapping.<pid>
gdb> bt
```

崩溃偶发、无法交互复现时最有用：跑完整包后直接对着 core 看崩溃现场。

---

## 四、日志/打印调试

不改调试器、最轻量的方式，适合看算法整体行为和统计量。

1. **代码自带打印**：`src/vio.cpp`、`src/pre_process.cpp` 里有多处 `printf` / `std::cout`（帧计数、残差、耗时），按需打开或加自己的。建议用自己的宏包起来方便关闭：

   ```cpp
   #define DBG printf
   // DBG("frame %d residual %.4f\n", frame_id, residual);
   ```

2. **ROS 日志级别**：

   ```bash
   # 运行时打开 debug 输出
   rosconsole set /fastlivo_mapping ros.console default debug
   # 或启动前环境变量
   ROSCONSOLE_DEFAULT_SEVERITY=debug roslaunch ...
   ```

   日志同时落在 `~/.ros/log/` 下，方便事后翻。

3. **建议加打印的位置**：
   - 每帧：图像/雷达时间戳、IMU 队列长度、帧号；
   - LIO 更新后：`res_prior / res_opt`、迭代次数、`norm_dx`；
   - VIO 更新后：patch 对齐成功率、金字塔各层残差；
   - 耗时统计每 100 帧打一次，避免刷屏。

4. **已知坑**（沿用调试环境文档结论）：roslaunch 后台运行时 stdout 全缓冲，进程被 kill 时缓冲丢失——验证是否真正处理数据要查话题（`rostopic echo -n 1 /odometry_reg`），不要只看终端输出；前台调试加 `--screen` 参数。

---

## 五、数据回放配合调试

断点会把节点停住，而 rosbag 还在（`--clock` 虚拟时钟下）播放，所以调试时必须配合慢放/暂停：

```bash
# 慢放到 0.2 倍速（Debug 版本 + 断点时建议 0.1~0.3）
rosbag play --clock -r 0.2 xxx.bag

# 直接跳到出问题的时间段，例如第 40 秒开始（不用干等前面 40 秒）
rosbag play --clock -s 40 xxx.bag

# 播放中按空格键暂停/继续
rosbag play --clock xxx.bag
```

组合技巧：

- **最快的定位路径**：先跑一遍 Release 版，记录出问题的 rosbag 时间点/帧号 → Debug 编译 + `-s <秒>` 直接跳过去 + 条件断点，避免在前面的正常数据上反复停。
- 节点被断点停住时 ROS 时间（`use_sim_time`）随 `--clock` 缺心跳而暂停，恢复后一般能续上；如果 IMU 缓冲爆掉报错，把回放速率再调低，或先按空格暂停 rosbag 再单步。
- 整体行为验证保持 `rviz:=false`，用 `rostopic echo -n 1 /odometry_reg` 确认里程计持续输出。

---

## 六、常见坑汇总

| 现象 | 原因 / 解决 |
|---|---|
| attach 报 `ptrace: Operation not permitted` | WSL 里执行 `echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope`（重启后失效，可加进 sysctl 或每次执行） |
| 变量显示 `value optimized out` | 没用 Debug 编译，回到第一节 `catkin_make -DCMAKE_BUILD_TYPE=Debug` |
| 断点打不上（灰色空心圆） | 改过代码没重编，或断在模板/头文件的优化代码里；先重新编译确认 |
| Eigen 变量打不出值 | 没装 eigen printer，见 2.4；临时用 `p v.data()[i]` |
| attach 后节点卡住不动 | gdb 拦截了信号，`c` 继续；必要时 `handle SIG64 nostop noprint pass` |
| Debug 版实时性跟不上、队列报错 | 回放降速 `-r 0.1`，或 rosbag 空格暂停后单步 |
| 后台跑时看不到日志输出 | stdout 全缓冲问题，用 `rostopic echo` 验证或 `roslaunch ... --screen` |
| WSL 后台进程被杀 | `wsl -e cmd` 退出会带走子进程，长时间调试保持 wsl 终端存活 |

---

## 七、推荐工作流总结

1. **平时研究算法**：Release 编译 + 正常速回放，看整体轨迹和统计。
2. **发现问题**：记录 rosbag 时间点 → `catkin_make -DCMAKE_BUILD_TYPE=Debug`。
3. **定位问题**：`rosbag play -s <秒>` 跳到现场 + VSCode attach + 条件断点 + Eigen printer 看数值。
4. **验证规律**：`display` / `watch` 盯住关键变量，配合 rosbag 空格暂停单步。
5. **测性能**：切回 Release，用代码内已有耗时统计（Debug 的耗时无意义）。
