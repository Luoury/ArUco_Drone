#!/usr/bin/env bash
# M0 最后一次验收尝试：起 bringup -> 等 mavros 连上 -> hover_test 起飞悬停 30 s。
# 之所以单独成脚本：命令行里出现 "ros2 launch" 字样时，stop_ros.sh 里的 pkill -f 会把调用它的
# shell 一起杀掉（本会话已踩过两次）。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
WS=$ROOT/ArUco_Drone_ws
LOG=$ROOT/logs/hover_test.log

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1
sleep 1

echo "[dev_m0_hover] 启动 bringup"
setsid nohup bash -c "source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; exec ros2 launch drone_bringup sim_bringup.launch.py" \
  > /tmp/bringup.log 2>&1 < /dev/null &

for i in $(seq 1 20); do
  sleep 1
  if grep -q "Got HEARTBEAT, connected" /tmp/bringup.log 2>/dev/null; then
    echo "[dev_m0_hover] mavros 已连接（等待 ${i}s）"
    break
  fi
done

tail -4 /tmp/bringup.log

echo "=== 1) /mavros/state ==="
timeout 20 ros2 topic echo /mavros/state --once --no-daemon 2>&1 | head -8

echo "=== 2) /mavros/local_position/pose 是否开始发 ==="
timeout 15 bash -c "ros2 topic echo /mavros/local_position/pose --once --no-daemon --qos-reliability best_effort 2>&1 | head -12"

echo "=== 3) hover_test ==="
timeout 180 ros2 run drone_control hover_test > "$LOG" 2>&1
RC=$?
echo "hover_test 退出码: $RC"
echo "--- hover_test 日志 ---"
cat "$LOG"
echo "[dev_m0_hover] 完成"
