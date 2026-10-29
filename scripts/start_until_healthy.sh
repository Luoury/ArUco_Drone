#!/usr/bin/env bash
# 启动 SITL 并校验 IMU 是否真的在发数据；坏状态就重启重试。
# 背景：本机 Gazebo 偶发"模型起来了但没有传感器发布者"，PX4 会因此 preflight 失败、mavros 收不到位姿。
set -o pipefail

ROOT=/home/luosury/ArUco_Drone
WORLD=${1:-aruco}
ATTEMPTS=${2:-3}
IMU_TOPIC=/world/$WORLD/model/x500_mono_cam_down_0/link/base_link/sensor/imu_sensor/imu

for i in $(seq 1 "$ATTEMPTS"); do
  echo "[healthy] 第 $i 次启动 SITL"
  bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1
  sleep 3
  bash "$ROOT/scripts/sim.sh" start "$WORLD" >/dev/null 2>&1

  ok=0
  for j in $(seq 1 20); do
    sleep 1
    if timeout 8 gz topic -i -t "$IMU_TOPIC" 2>/dev/null | grep -q "Publishers"; then
      ok=1
      break
    fi
  done

  if [ "$ok" = "1" ]; then
    echo "[healthy] 第 $i 次启动成功：IMU 有发布者"
    bash "$ROOT/scripts/sim.sh" status
    exit 0
  fi
  echo "[healthy] 第 $i 次启动后 IMU 仍无发布者，重试"
done

echo "[healthy] 连续 $ATTEMPTS 次都没拿到健康的 IMU，请检查 gz 日志"
exit 1
