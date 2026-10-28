#!/usr/bin/env bash
# 停止本项目的 ROS 2 节点（bringup / hover_test / 后续的检测与任务节点）。
# 单独写成脚本，避免 pkill 的模式串出现在调用它的命令行里（会把自己杀掉）。
set -uo pipefail

PATTERNS=(
  "mavros_node"
  "parameter_bridge"
  "sim_bringup.launch.py"
  "hover_test"
  "aruco_detector"
  "landing_controller"
  "waypoint_nav"
  "mission_manager"
  "verify_flow"
)

for pat in "${PATTERNS[@]}"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "[stop_ros] kill -f $pat"
    pkill -f "$pat" 2>/dev/null || true
  fi
done

sleep 1
echo "[stop_ros] 剩余相关进程："
ps -eo pid,args | grep -E "mavros_node|parameter_bridge|hover_test" | grep -v grep || echo "  (无)"
