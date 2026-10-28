#!/usr/bin/env bash
# 用最小世界测试 gz sim 相机渲染的内存增长，定位 RSS 线性上涨的触发条件。
#
# 用法：bash camera_leak_test.sh [engine] [duration] [variant]
#   variant: v1 静止相机(无阴影/visualize=false) | v2 加阴影+visualize=true
#            v3 相机挂在自由下落的动态 link 上 | v4 真实 aruco 世界+真实机体(不启 PX4)
set -uo pipefail

ENGINE="${1:-ogre}"
DURATION="${2:-60}"
VARIANT="${3:-v1}"
STAGE=/mnt/c/Users/Administrator/AppData/Local/Temp/aruco_stage

case "${VARIANT}" in
  v1) SDF="${STAGE}/test/cam_leak.sdf";  PATTERN="cam_leak.sdf" ;;
  v2) SDF="${STAGE}/test/cam_leak2.sdf"; PATTERN="cam_leak2.sdf" ;;
  v3) SDF="${STAGE}/test/cam_leak3.sdf"; PATTERN="cam_leak3.sdf" ;;
  v4) SDF="${STAGE}/test/cam_leak4.sdf"; PATTERN="cam_leak4.sdf" ;;
  *) echo "camera_leak_test: 未知 variant ${VARIANT}"; exit 1 ;;
esac

if [ ! -f "${SDF}" ]; then
  echo "camera_leak_test: 缺少 ${SDF}"
  exit 1
fi

# v4 需要解析 model://arucotag 与 model://x500_mono_cam_down
PX4_GZ=/home/luosury/PX4-Autopilot/Tools/simulation/gz
export GZ_SIM_RESOURCE_PATH="${PX4_GZ}/models:${PX4_GZ}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "camera_leak_test: variant=${VARIANT} engine=${ENGINE} duration=${DURATION}s"

setsid nohup gz sim -r -s --render-engine "${ENGINE}" "${SDF}" \
  >/tmp/cam_leak.log 2>&1 </dev/null &
sleep 12

# 取 RSS 最大的那个匹配进程：gz sim 会 fork 出子进程，父进程的 RSS 很小，
# 只看第一个匹配会误判（本会话踩过：把 71 MB 的 launcher 当成渲染进程）。
BIGGEST=0
for p in $(pgrep -f "${PATTERN}" || true); do
  rss="$(awk '/^VmRSS:/{print $2}' "/proc/${p}/status" 2>/dev/null || echo 0)"
  if [ "${rss}" -gt "${BIGGEST}" ]; then
    BIGGEST=${rss}
    PID=${p}
  fi
done
PID="${PID:-}"
if [ -z "${PID}" ]; then
  echo "camera_leak_test: gz 没起来，日志："
  tail -n 20 /tmp/cam_leak.log
  exit 1
fi

CAM_TOPIC="$(gz topic -l 2>/dev/null | grep 'sensor/camera/image' | head -1 || true)"
echo "camera_leak_test: gz pid=${PID} cam_topic=${CAM_TOPIC:-none}"
if [ -n "${CAM_TOPIC}" ]; then
  gz topic -i -t "${CAM_TOPIC}" 2>/dev/null | head -6
fi

FIRST=0
FIRST_T=0
ELAPSED=0
while [ "${ELAPSED}" -lt "${DURATION}" ]; do
  RSS_KB="$(awk '/^VmRSS:/{print $2}' "/proc/${PID}/status" 2>/dev/null || echo 0)"
  RSS_MB=$((RSS_KB / 1024))
  [ "${FIRST}" -eq 0 ] && FIRST=${RSS_MB} && FIRST_T=${ELAPSED}
  echo "  t=${ELAPSED}s rss=${RSS_MB}MB"
  sleep 10
  ELAPSED=$((ELAPSED + 10))
done

RSS_KB="$(awk '/^VmRSS:/{print $2}' "/proc/${PID}/status" 2>/dev/null || echo 0)"
LAST=$((RSS_KB / 1024))
SPAN=$((ELAPSED - FIRST_T))
[ "${SPAN}" -gt 0 ] && echo "camera_leak_test: ${FIRST}MB -> ${LAST}MB / ${SPAN}s = $(((LAST - FIRST) / SPAN))MB/s"

kill -9 "${PID}" 2>/dev/null
echo "camera_leak_test: 已停止 gz pid=${PID}"
