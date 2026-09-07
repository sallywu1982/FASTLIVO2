#!/bin/bash
# analyze_lio_log.sh — 分析 /tmp/lio_node.log 的 LIO 运行质量
L=/tmp/lio_node.log
echo "=== LiDAR 帧处理范围（bag 时间） ==="
grep -a "Get LiDAR" "$L" | head -1 | cut -c1-120
grep -a "Get LiDAR" "$L" | tail -1 | cut -c1-120
echo "共 $(grep -ac "Get LiDAR" "$L") 帧被处理"
echo "=== IMU/系统重置次数 ==="
echo "IMU Initializing: $(grep -ac "IMU Initializing" "$L")"
echo "Reset: $(grep -ac "Reset" "$L")"
echo "=== ERROR / WARN（排除 XmlRpc 噪声） ==="
echo "ERROR: $(grep -ac "ERROR" "$L")"
grep -ai "warn" "$L" | grep -av "XmlRpc" | head -5 | cut -c1-130
echo "=== NaN 检查 ==="
grep -ac "nan\|NaN" "$L"
echo "=== 最后 6 行 ==="
tail -6 "$L" | cut -c1-130
