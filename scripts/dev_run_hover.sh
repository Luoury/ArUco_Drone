#!/usr/bin/env bash
# 在已经健康的 SITL 上跑一次 hover_test，并把完整输出打出来。
# 前置：SITL 已由 start_until_healthy.sh 起好（IMU 有发布者）。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
WS=$ROOT/ArUco_Drone_ws

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1
sleep 1

setsid nohup bash -c "source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; exec ros2 launch drone_bringup sim_bringup.launch.py" \
  > /tmp/bringup.log 2>&1 < /dev/null &

for i in $(seq 1 30); do
  sleep 1
  if grep -q "Got HEARTBEAT, connected" /tmp/bringup.log 2>/dev/null; then
    echo "[run_hover] mavros 已连接（${i}s）"
    break
  fi
done

# 等 mavros 真的开始发位姿（PX4 的 LOCAL_POSITION_NED 有时要等 EKF 收敛）
for i in $(seq 1 30); do
  if timeout 6 ros2 topic echo /mavros/local_position/pose --once \
       --qos-reliability best_effort --no-daemon >/tmp/pose_once.txt 2>&1; then
    echo "[run_hover] 位姿已就绪（${i}s）"
    break
  fi
  sleep 1
done

echo "===== hover_test ====="
timeout 180 ros2 run drone_control hover_test 2>&1
echo "hover_test exit=$?"
