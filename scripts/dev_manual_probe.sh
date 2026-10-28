#!/usr/bin/env bash
# 复现"MANUAL 接管后掉高"：起 SITL + flow_f2，然后请求 MANUAL，同时记录真实与估计高度。
set -o pipefail
set +u

WS="$HOME/ArUco_Drone/ArUco_Drone_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
SCRIPTS="$HOME/ArUco_Drone/scripts"
LOG_PREFIX=/tmp/manual_probe

bash "$SCRIPTS/start_until_healthy.sh" aruco 3 | tail -5
bash "$SCRIPTS/stop_ros.sh" >/dev/null 2>&1

setsid nohup ros2 launch drone_bringup flow_f2.launch.py \
	> "${LOG_PREFIX}_bringup.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "[probe] launch pid=$LAUNCH_PID"

for i in $(seq 1 60); do
	if grep -q "Got HEARTBEAT, connected" "${LOG_PREFIX}_bringup.log" 2>/dev/null; then
		echo "[probe] mavros 已连接（${i}s）"
		break
	fi
	sleep 1
done
sleep 3

python3 - <<'PYEOF' >/dev/null 2>&1
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
n = Node('probe_mode_pub')
pub = n.create_publisher(String, '/mission/mode', 10)
# 等发现完成再发：立刻 publish 会因为还没建立匹配而全部丢弃
end = time.monotonic() + 1.0
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
# 连发 3 s（mission_manager 只在收到消息时置 pending，多发几次确保命中）
end = time.monotonic() + 3.0
while time.monotonic() < end:
    pub.publish(String(data='MANUAL'))
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.05)
rclpy.shutdown()
PYEOF
echo "[probe] 已请求 MANUAL，开始记录 20 s"
python3 "$SCRIPTS/trace_alt.py" 20 > "${LOG_PREFIX}_trace.csv" 2>&1
echo "[probe] 记录结束，采样：$(wc -l < "${LOG_PREFIX}_trace.csv") 行"
tail -25 "${LOG_PREFIX}_trace.csv"
