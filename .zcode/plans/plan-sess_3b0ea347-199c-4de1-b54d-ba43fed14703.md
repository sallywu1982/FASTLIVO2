生成一份《DEBUG_GUIDE.md》完全可操作的调试文档，写入两处：
- Windows 本地代码文件夹 `E:\StoneRecord\ws-Work\06_FAST-LIVO2学习\FAST-LIVO2\DEBUG_GUIDE.md`
- WSL 实际调试环境 `/root/catkin_ws/src/FAST-LIVO2/DEBUG_GUIDE.md`（用 wsl 命令复制，便于在 VSCode Remote 里直接看）

文档特点：按步骤编号、每步含可复制粘贴的完整命令、标注当前环境已完成的配置（gdb 已装、launch.json 已建、eigen printer 已注册、WSL 扩展已装）、每步附"预期结果/如何确认成功"，并覆盖排错。断点位置全部使用刚核实的真实函数名与行号：

内容结构：
1. **环境现状清单**：哪些已配好（gdb 9.2、/root/catkin_ws/.vscode/launch.json、/root/eigen_printers.py + ~/.gdbinit、ptrace_scope=0 及 WSL 重启后需重设的提醒）、哪些待做
2. **第 0 步 Debug 编译**：catkin_make -DCMAKE_BUILD_TYPE=Debug，如何用 `file` 命令验证带符号、如何切回 Release
3. **第 1 步 VSCode 连接 WSL**：绿色远程按钮 → 连接 WSL → 打开 /root/catkin_ws → "Install in WSL" 装 C/C++ 扩展
4. **第 2 步 启动调试会话**：roslaunch + rosbag -r 0.3 慢放 → F5 → pickProcess 选 fastlivo_mapping；确认 attach 成功的标志
5. **断点速查表（真实位置）**：
   - 入口/主状态机：main.cpp:3、LIVMapper::stateEstimationAndMapping (LIVMapper.cpp:267)
   - 回调：standard_pcl_cbk:703 / livox_pcl_cbk:726 / imu_cbk:769 / img_cbk:829、sync_packages:884
   - IMU：processImu:248、IMU_Processing.cpp
   - LIO/ESIKF：handleLIO:336 → StateEstimation (voxel_map.cpp:338)、BuildResidualListOMP:643、build_single_residual:713
   - VIO：handleVIO:281 → processFrame (vio.cpp:1787)、updateState:1521
   - 地图：BuildVoxelMap:532 / UpdateVoxelMap:609
   - 每项标注"看什么变量、判断什么问题"
6. **条件断点与变量查看**：Eigen printer 用法（p cov、p state.rotation）、条件断点写法、rosbag -s 跳时段配合
7. **命令行 GDB 替代方案**：gdb -p attach、launch-prefix、bt、core dump
8. **故障排查表**：ptrace 报错、value optimized out、断点空心、attach 后卡住、IMU 队列爆、stdout 缓冲等，均附具体命令