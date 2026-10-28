#!/usr/bin/env bash
# M1 联调：起 SITL + bringup + aruco_detector，然后在 2 m 悬停时验证检测精度。
# 变量与 pkill 模式都只写在脚本里（命令行里出现会被中间层吃掉/自杀）。
set -o pipefail
set +u

ROOT="$HOME/ArUco_Drone"
WS="$ROOT/ArUco_Drone_ws"
LOGS="$ROOT/logs"
mkdir -p "$LOGS"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[m1] 1/6 清理旧进程"
bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true

echo "[m1] 2/6 启动 SITL（带 IMU 健康检查）"
if ! bash "$ROOT/scripts/start_until_healthy.sh" aruco 3 2>&1 | tail -4; then
  echo "[m1] SITL 起不来，退出"
  exit 1
fi

echo "[m1] 3/6 启动 bringup（/clock + 相机 + mavros）"
setsid nohup ros2 launch drone_bringup sim_bringup.launch.py > "$LOGS/m1_bringup.log" 2>&1 < /dev/null &

for i in $(seq 1 40); do
  if grep -q "Got HEARTBEAT, connected" "$LOGS/m1_bringup.log" 2>/dev/null; then
    echo "[m1] mavros 已连接（${i}s）"
    break
  fi
  sleep 1
done

echo "[m1] 4/6 启动 aruco_detector"
POSE_TOPIC="${ARUCO_POSE_TOPIC:-/mavros/local_position/pose}"
setsid nohup ros2 launch drone_bringup aruco_detect.launch.py \
  drone_pose_topic:="$POSE_TOPIC" > "$LOGS/m1_aruco.log" 2>&1 < /dev/null &
for i in $(seq 1 30); do
  if grep -q "aruco_detector 启动" "$LOGS/m1_aruco.log" 2>/dev/null; then
    echo "[m1] 检测节点就绪（${i}s）"
    break
  fi
  sleep 1
done

echo "[m1] 5/6 起飞到 2 m 并悬停（后台）"
setsid nohup ros2 run drone_control hover_test --ros-args -p hover_altitude:=2.0 -p hover_duration:=35.0 \
  > "$LOGS/m1_hover.log" 2>&1 < /dev/null &

for i in $(seq 1 60); do
  if grep -q "OFFBOARD 已接管" "$LOGS/m1_hover.log" 2>/dev/null; then
    echo "[m1] 已交接 OFFBOARD（${i}s），开始采样检测结果"
    break
  fi
  sleep 1
done
sleep 4

echo "[m1] 6/6 校验检测精度"
STEP6="${M1_STEP6:-$ROOT/scripts/m1_check_detection.py}"
python3 "$STEP6" 15
rc=$?
echo "[m1] 检测校验返回 $rc（0=PASS）"
tail -3 "$LOGS/m1_hover.log"
exit $rc
