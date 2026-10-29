#!/usr/bin/env bash
# 多工位巡回演示：一条命令跑完"基站 → 3 个工位 → 回基站"，并录成 mp4。
#
#   bash scripts/demo_tour.sh [录像秒数]     默认 280
#
# 流程：清场 → 内存预检 → 起 SITL（含 IMU 健康检查）→ 摆 3 个工位码 + 斜俯视机位
#      → 起 mission/detector/landing（start_overhead:=true）→ 起录制 → 发 TOUR
#      → 盯着 /mission/status 直到 TOUR_DONE → 收尾。
#
# 内存保护（本机 WSL 只有 7.6 GB，曾经跑挂过一次）：
#   - 起之前先把所有 gz/px4/ros 进程清干净（hard clean）；
#   - 可用内存 < MIN_AVAIL 就直接放弃，不冒险；
#   - 全程只允许一个 SITL、一个录制器。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ArUco_Drone_ws"
REC_SECONDS="${1:-280}"
MIN_AVAIL="${MIN_AVAIL:-3500}"      # MB，低于这个就不起（7.6 GB 机器留足余量）
LOG=/tmp/tour_run.log

say() { echo "[tour] $*"; }

cleanup_on_error() {
  say "出错收尾：停 ROS 与仿真"
  bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
  bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true
}

say "① 清场"
bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true
pkill -f "lib/drone_tools/record_video" >/dev/null 2>&1 || true
pkill -f "tour_watch" >/dev/null 2>&1 || true
sleep 2

avail=$(free -m | awk '/^Mem:/ {print $7}')
say "② 内存预检：可用 ${avail} MB（要求 ≥ ${MIN_AVAIL} MB）"
if [ "${avail:-0}" -lt "$MIN_AVAIL" ]; then
  say "可用内存不足，放弃本次演示。先手工关掉占内存的程序再试。"
  exit 1
fi

say "③ 起 SITL"
if ! bash "$ROOT/scripts/start_until_healthy.sh" aruco 2; then
  cleanup_on_error
  exit 1
fi

say "④ 摆放 3 个工位码 + 斜俯视机位"
if ! bash "$ROOT/scripts/spawn_course.sh" aruco; then
  cleanup_on_error
  exit 1
fi

say "⑤ 起任务链（含演示机位桥接）"
cd "$WS" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
: > "$LOG"
setsid nohup ros2 launch drone_bringup flow_tour.launch.py \
  start_overhead:=true overhead_model:=demo_cam \
  >"$LOG" 2>&1 </dev/null &

for _ in $(seq 1 90); do
  sleep 1
  grep -q "Got HEARTBEAT, connected" "$LOG" 2>/dev/null && break
done
if ! grep -q "Got HEARTBEAT, connected" "$LOG"; then
  say "MAVROS 15 s 内没连上，日志尾部："
  tail -n 20 "$LOG"
  cleanup_on_error
  exit 1
fi
say "    MAVROS 已连接"
for _ in $(seq 1 60); do
  sleep 1
  grep -q "mission_manager 就绪" "$LOG" 2>/dev/null && break
done
for _ in $(seq 1 40); do
  sleep 1
  grep -q "发布 /aruco/detection" "$LOG" 2>/dev/null && break
done
say "    任务链就绪"

say "⑥ 开始录制（${REC_SECONDS} s）"
STAMP=$(date +%H%M%S)
VIDEO="$ROOT/logs/videos/tour_${STAMP}.mp4"
setsid nohup ros2 run drone_tools record_video --ros-args \
  -p output:="$VIDEO" -p duration:="${REC_SECONDS}.0" -p title:="ArUco 多工位巡回" \
  >/tmp/tour_record.log 2>&1 </dev/null &
sleep 4

say "⑦ 发 TOUR 模式请求"
for _ in 1 2 3; do
  ros2 topic pub --once /mission/mode std_msgs/msg/String "{data: TOUR}" >/dev/null 2>&1
  sleep 1
done

say "⑧ 盯着状态机（超时 320 s）"
python3 "$ROOT/scripts/tour_watch.py" 320
watch_rc=$?

say "⑨ 收尾：停录制与任务链"
pkill -f "lib/drone_tools/record_video" >/dev/null 2>&1 || true
sleep 2
bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true

say "录像：$VIDEO"
if [ -f "$VIDEO" ]; then
  say "大小：$(du -h "$VIDEO" | cut -f1)"
else
  say "没有生成录像文件，看 /tmp/tour_record.log"
fi
say "仿真仍在运行；要停：bash scripts/sim.sh stop"
[ "$watch_rc" -eq 0 ] && say "结果：TOUR 完成（退出码 0）" || say "结果：未完成（退出码 ${watch_rc}）"
exit "$watch_rc"
