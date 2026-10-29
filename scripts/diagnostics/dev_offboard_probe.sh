#!/usr/bin/env bash
# 重启 bringup 并跑一次 offboard_probe。
# 用法: dev_offboard_probe.sh [pos|vel] [目标值] [--yaw-valid]
# 注意：stop_ros.sh 里有 pkill -f "ros2 launch"，所以启动 bringup 的语句必须待在脚本里，
# 不能在调用它的命令行里出现（否则会把自己那条 shell 一起杀掉）。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
WS=$ROOT/ArUco_Drone_ws
MODE=${1:-pos}
TARGET=${2:-1.5}
shift 2 2>/dev/null || true
EXTRA="$*"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1
sleep 1

setsid nohup bash -c "source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; exec ros2 launch drone_bringup sim_bringup.launch.py" \
  > /tmp/bringup.log 2>&1 < /dev/null &

for i in $(seq 1 25); do
  sleep 1
  if grep -q "Got HEARTBEAT, connected" /tmp/bringup.log 2>/dev/null; then
    echo "[dev_offboard_probe] mavros 已连接（${i}s）"
    break
  fi
done

timeout 90 python3 "$ROOT/scripts/diagnostics/offboard_probe.py" "$MODE" "$TARGET" $EXTRA
echo "[dev_offboard_probe] 完成"
