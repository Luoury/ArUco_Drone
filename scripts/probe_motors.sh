#!/usr/bin/env bash
# 诊断：hover_test 起飞失败时，PX4 到底有没有把电机指令发给 Gazebo。
# 采样 gz 侧的 /<model>/command/motor_speed 与模型里程计，判断是"控制没生效"还是"推力没起来"。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
source /opt/ros/humble/setup.bash
source "$ROOT/ArUco_Drone_ws/install/setup.bash"

MOTOR_TOPIC=/x500_mono_cam_down_0/command/motor_speed
ODOM_TOPIC=/model/x500_mono_cam_down_0/odometry_with_covariance

setsid nohup bash -c "source /opt/ros/humble/setup.bash; source '$ROOT/ArUco_Drone_ws/install/setup.bash'; exec ros2 run drone_control hover_test" \
  > /tmp/probe_hover.log 2>&1 < /dev/null &

sleep 8
echo "=== hover_test 日志（此刻）==="
tail -6 /tmp/probe_hover.log

echo "=== gz 电机指令采样（2 帧）==="
timeout 12 gz topic -e -t "$MOTOR_TOPIC" -n 2 2>&1 | head -30

echo "=== gz 模型里程计采样（1 帧）==="
timeout 12 gz topic -e -t "$ODOM_TOPIC" -n 1 2>&1 | grep -A6 "position" | head -10

echo "=== hover_test 日志（8 s 后）==="
tail -6 /tmp/probe_hover.log
echo "[probe_motors] 完成（hover_test 仍在跑，稍后用 stop_ros.sh 清场）"
