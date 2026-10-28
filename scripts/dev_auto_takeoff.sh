#!/usr/bin/env bash
# 重启 bringup（若已起则复用），然后跑 dev_auto_takeoff.py。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
WS=$ROOT/ArUco_Drone_ws
MODE=${1:-AUTO.TAKEOFF}

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if ! pgrep -f "mavros_node" >/dev/null 2>&1; then
  setsid nohup bash -c "source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; exec ros2 launch drone_bringup sim_bringup.launch.py" \
    > /tmp/bringup.log 2>&1 < /dev/null &
  for i in $(seq 1 25); do
    sleep 1
    if grep -q "Got HEARTBEAT, connected" /tmp/bringup.log 2>/dev/null; then
      echo "[dev_auto_takeoff] mavros 已连接（${i}s）"
      break
    fi
  done
else
  echo "[dev_auto_takeoff] mavros 已在运行"
fi

timeout 90 python3 "$ROOT/scripts/dev_auto_takeoff.py" "$MODE"
echo "[dev_auto_takeoff] 完成"
