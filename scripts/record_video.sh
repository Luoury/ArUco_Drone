#!/usr/bin/env bash
# 一条命令录一段演示视频（不依赖任何 GUI 窗口，所以不会卡）。
#
#   bash scripts/record_video.sh [流程=F2] [秒数=70] [auto|manual]
#
#   auto   —— 录制期间自动跑一遍该流程的验收（会真的起飞、降落）
#   manual —— 只起仿真与录制，你自己另外开终端用 teleop 飞
#
# 产物：logs/videos/<流程>_<时分秒>.mp4（1280x720@30，带 HUD 与机载检测小窗）
#
# 为什么不用录屏：WSLg 的窗口要经远程桌面协议贴回 Windows，Gazebo 连续刷新
# 的画面很容易 stalled（窗口里 3D 视口不动、右键菜单还在动），录屏就废了。
# 这里改成"相机在仿真里，帧从 ROS 拿"，帧率恒定、可重复、还能带 HUD。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLOW="${1:-F2}"
DURATION="${2:-70}"
MODE="${3:-auto}"
WORLD=aruco

VIDEO_DIR="$ROOT/logs/videos"
mkdir -p "$VIDEO_DIR"
STAMP="$(date +%H%M%S)"
OUT="$VIDEO_DIR/${FLOW}_${STAMP}.mp4"
LAUNCH_LOG="/tmp/record_bringup.log"
REC_LOG="/tmp/record_node.log"
FLOW_LOG="/tmp/record_flow.log"

flow_launch() {
  case "$(echo "$FLOW" | tr 'a-z' 'A-Z')" in
    F1) echo flow_f1.launch.py ;;
    F2) echo flow_f2.launch.py ;;
    F3) echo flow_f3.launch.py ;;
    F4) echo flow_f4.launch.py ;;
    F5) echo flow_f5.launch.py ;;
    *) echo "" ;;
  esac
}

LAUNCH_FILE="$(flow_launch)"
if [ -z "$LAUNCH_FILE" ]; then
  echo "[record] 流程名只能是 F1/F2/F3/F4/F5，收到：$FLOW" >&2
  exit 2
fi

echo "[record] 流程=$FLOW 时长=${DURATION}s 模式=$MODE 输出=$OUT"

echo "[record] ① 清理旧进程"
bash "$ROOT/scripts/stop_ros.sh"
bash "$ROOT/scripts/sim.sh" stop

echo "[record] ② 起 SITL（含 IMU 健康检查，约 60~90 s）"
bash "$ROOT/scripts/start_until_healthy.sh" "$WORLD" 3 || exit 1

echo "[record] ③ 挂俯视机位"
bash "$ROOT/scripts/spawn_overhead_cam.sh" "$WORLD" || exit 1

echo "[record] ④ 起流程（带俯视机位桥接）"
cd "$ROOT/ArUco_Drone_ws" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
setsid nohup ros2 launch drone_bringup "$LAUNCH_FILE" start_overhead:=true \
  >"$LAUNCH_LOG" 2>&1 </dev/null &

for i in $(seq 1 60); do
  if grep -q "mission_manager 就绪" "$LAUNCH_LOG" && grep -q "俯视机位" "$LAUNCH_LOG"; then
    break
  fi
  sleep 2
done
grep -q "mission_manager 就绪" "$LAUNCH_LOG" || { echo "[record] 流程没起来，看 $LAUNCH_LOG" >&2; exit 1; }
if ! grep -q "俯视机位" "$LAUNCH_LOG"; then
  echo "[record] 警告：桥接没打出俯视机位日志（录制可能没有画面）" >&2
fi

echo "[record] ⑤ 开始录制"
REC_SECONDS=$((DURATION + 15))
setsid nohup ros2 run drone_tools record_video --ros-args \
  -p output:="$OUT" -p fps:=25.0 -p duration:="${REC_SECONDS}.0" -p title:="ArUco landing $FLOW" \
  >"$REC_LOG" 2>&1 </dev/null &
sleep 4

if [ "$MODE" = "auto" ]; then
  echo "[record] ⑥ 自动跑 $FLOW 流程"
  ros2 run drone_tools verify_flow --flow "$FLOW" \
    --ros-args --params-file install/drone_bringup/share/drone_bringup/config/flows.yaml \
    >"$FLOW_LOG" 2>&1
  echo "[record] 流程退出码 $?（0 = PASS）"
else
  echo "[record] ⑥ manual 模式：$DURATION 秒内你可以自己用 teleop 飞"
fi

# 注意：匹配串必须比脚本名更具体，否则 pkill -f 会把正在跑本脚本的 shell 一起杀掉
echo "[record] ⑦ 等录制收尾（最多 ${REC_SECONDS}s）"
for i in $(seq 1 "$REC_SECONDS"); do
  pgrep -f "lib/drone_tools/record_video" >/dev/null || break
  sleep 1
done
pgrep -f "lib/drone_tools/record_video" >/dev/null && pkill -f "lib/drone_tools/record_video" 2>/dev/null

echo "[record] ⑧ 收尾"
bash "$ROOT/scripts/stop_ros.sh"

if [ -f "$OUT" ]; then
  echo "[record] 视频已生成：$OUT（$(du -h "$OUT" | cut -f1)）"
  grep -E "录制结束|分辨率|没有收到图像帧" "$REC_LOG" | tail -5
else
  echo "[record] 失败：没有生成视频，检查 $REC_LOG 与 $LAUNCH_LOG" >&2
  tail -20 "$REC_LOG" >&2
  exit 1
fi

if [ -f "$FLOW_LOG" ]; then
  grep -E "RESULT|elapsed|final_" "$FLOW_LOG" | tail -6
fi

echo "[record] 仿真仍在运行；要停掉：bash scripts/sim.sh stop"
