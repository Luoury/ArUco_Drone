#!/usr/bin/env bash
# F2 流程一键跑：起 SITL（带健康检查）→ 起 flow_f2.launch.py → 跑 verify_flow --flow F2。
# 用法：bash run_f2.sh
#
# 注意：本脚本里不要用 set -u（source /opt/ros/humble/setup.bash 在 set -u 下会
# 报 AMENT_TRACE_SETUP_FILES: unbound variable 直接退出）。
set -eo pipefail

ROOT="$HOME/ArUco_Drone"
WS="$ROOT/ArUco_Drone_ws"
LOG=/tmp/f2_bringup.log
LAUNCH_WAIT="${1:-60}"

echo "=== [1/3] 重启 SITL（含 IMU 健康检查）==="
bash "$ROOT/scripts/start_until_healthy.sh" aruco 3

echo "=== [2/3] 起 flow_f2.launch.py ==="
bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
: > "$LOG"
setsid nohup ros2 launch drone_bringup flow_f2.launch.py > "$LOG" 2>&1 < /dev/null &
lp=$!
echo "[f2] launch pid=$lp 日志=$LOG"

waited=0
while [ "$waited" -lt "$LAUNCH_WAIT" ]; do
  if grep -q "Got HEARTBEAT, connected" "$LOG" 2>/dev/null; then
    echo "[f2] mavros 已连接（${waited}s）"
    break
  fi
  sleep 1
  waited=$((waited + 1))
done
if [ "$waited" -ge "$LAUNCH_WAIT" ]; then
  echo "[f2] 失败：${LAUNCH_WAIT}s 内 mavros 未连接，日志尾部："
  tail -30 "$LOG"
  exit 1
fi

# 三个节点都要在：mission_manager / landing_controller / aruco_detector
for want in "mission_manager 就绪" "降落控制器就绪" "发布 /aruco/detection"; do
  waited=0
  while [ "$waited" -lt 40 ]; do
    if grep -q "$want" "$LOG" 2>/dev/null; then
      echo "[f2] 就绪：$want（${waited}s）"
      break
    fi
    sleep 1
    waited=$((waited + 1))
  done
  if [ "$waited" -ge 40 ]; then
    echo "[f2] 失败：等不到「$want」，日志尾部："
    tail -40 "$LOG"
    exit 1
  fi
done

echo "=== [3/3] 跑 verify_flow F2 ==="
set +e
ros2 run drone_tools verify_flow --flow F2 \
  --ros-args --params-file "$WS/install/drone_bringup/share/drone_bringup/config/flows.yaml"
code=$?
set -e
echo "[f2] verify_flow exit=$code"

echo "=== 收尾：停 ROS 与 SITL ==="
bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true
echo "[f2] 完成"
exit "$code"
