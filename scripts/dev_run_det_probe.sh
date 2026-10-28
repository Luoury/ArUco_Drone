#!/usr/bin/env bash
# 复现"检测到的码位置离无人机很远"：起 SITL + flow_f2，跑 det_probe 打印原始数据。
set -o pipefail
set +u

WS="$HOME/ArUco_Drone/ArUco_Drone_ws"
SCRIPTS="$HOME/ArUco_Drone/scripts"
LOG=/tmp/det_probe.log

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

bash "$SCRIPTS/start_until_healthy.sh" aruco 3 | tail -4
bash "$SCRIPTS/stop_ros.sh" >/dev/null 2>&1

setsid nohup ros2 launch drone_bringup flow_f2.launch.py \
	> "${LOG}_bringup.log" 2>&1 < /dev/null &
echo "[probe] launch pid=$!"

for i in $(seq 1 60); do
	if grep -q "Got HEARTBEAT, connected" "${LOG}_bringup.log" 2>/dev/null; then
		echo "[probe] mavros 已连接（${i}s）"
		break
	fi
	sleep 1
done
sleep 4

python3 "$SCRIPTS/dev_det_probe.py"
