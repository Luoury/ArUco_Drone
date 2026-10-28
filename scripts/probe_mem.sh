#!/usr/bin/env bash
# 采样 gz sim 的 RSS 增长，判断相机渲染是否在泄漏内存。
# 用法：bash probe_mem.sh [采样次数] [间隔秒]
set -uo pipefail

TIMES="${1:-4}"
INTERVAL="${2:-20}"

PID="$(pgrep -f 'gz sim' | head -1 || true)"
if [ -z "${PID}" ]; then
  echo "probe_mem: 没找到 gz sim 进程"
  exit 1
fi
ENGINE="$(tr '\0' ' ' <"/proc/${PID}/cmdline" | grep -o 'render-engine [a-z0-9]*' || echo 'render-engine ?')"
echo "probe_mem: gz pid=${PID} ${ENGINE}"

for i in $(seq 1 "${TIMES}"); do
  RSS_KB="$(awk '/^VmRSS:/{print $2}' "/proc/${PID}/status" 2>/dev/null || echo 0)"
  MEM="$(free -m | awk 'NR==2{printf "used=%sMB avail=%sMB", $3, $7}')"
  echo "  t=$((i * INTERVAL))s gz_rss=$((RSS_KB / 1024))MB ${MEM}"
  [ "${i}" -lt "${TIMES}" ] && sleep "${INTERVAL}"
done

echo "probe_mem: RTF / iterations"
timeout 8 gz topic -e -t /world/aruco/stats -n 1 2>/dev/null |
  grep -E "real_time_factor|iterations" || echo "  (取不到 stats)"
