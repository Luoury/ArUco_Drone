#!/usr/bin/env bash
# 启动/停止 PX4 + Gazebo Harmonic SITL。
#
# 为什么必须 headless + Ogre1：
#   WSL2 没有 /dev/dri，Ogre2 走软件渲染带 1280x960 相机时 RTF 会掉到 0.006
#   （比实时慢 180 倍），PX4 主循环被拖死、MAVROS 每 9 s 掉线一次。
#   换 Ogre1（PX4_GZ_SIM_RENDER_ENGINE=ogre）+ HEADLESS=1 后 RTF ≈ 0.98。
#
# 用法：
#   scripts/sim.sh start [world]     # 默认 aruco
#   scripts/sim.sh stop
#   scripts/sim.sh status
#   scripts/sim.sh restart [world]
set -uo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
ROOTFS="$BUILD_DIR/rootfs"
SIM_MODEL="${SIM_MODEL:-gz_x500_mono_cam_down}"
WORLD="${WORLD:-aruco}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre}"
LOG_DIR="${LOG_DIR:-$HOME/ArUco_Drone/logs}"
LOG_FILE="$LOG_DIR/px4_sitl.log"
PID_FILE="$LOG_DIR/px4_sitl.pid"
# gz 插件清单：PX4 默认清单里的 custom::GstCameraSystem 会让相机每帧泄漏
# 约 3.7 MB（gz sim RSS 以 ~105 MB/s 增长，90 s 撑爆 WSL）。
# 启动前用 scripts/fix_gz_plugins.sh 把清单换成仓库里的安全版本。
GZ_PLUGIN_FIX="${GZ_PLUGIN_FIX:-$(dirname "$0")/fix_gz_plugins.sh}"
# gz 模型：x500_mono_cam_down → x500 → x500_base 的 include(merge=true) 链会在 spawn 时
# 丢掉 base_link 上的 4 个传感器（PX4 于是报 Accel/Gyro/baro/compass missing，无法 arm）。
# 启动前用 scripts/fix_gz_model.sh 把模型展平并让 PX4 用展平版。
GZ_MODEL_FIX="${GZ_MODEL_FIX:-$(dirname "$0")/fix_gz_model.sh}"
# 仓库自带的 gz 模型（overhead_cam / demo_cam / arucotag_1..3）：加进 GZ_SIM_RESOURCE_PATH，
# 这样 model.sdf 里的 model://<名字>/纹理.png 能解析，也能用 sdf_filename: "model://xxx"。
OWN_MODELS="${OWN_MODELS:-$(cd "$(dirname "$0")/.." && pwd)/ArUco_Drone_ws/src/drone_bringup/models}"

usage() {
  sed -n '2,20p' "$0"
  exit 1
}

px4_pid() {
  if [ -f "$PID_FILE" ]; then
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    # 只认 cmdline 里确实有 bin/px4 的进程：WSL 重启后 pid 会被复用，
    # 单靠 kill -0 会把别的进程误判成 PX4（已实测把 ros_gz_bridge 当成 PX4）。
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null &&
      tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | grep -q 'bin/px4'; then
      echo "$pid"
      return 0
    fi
  fi
  pgrep -n -f "bin/px4 -d" || true
}

start() {
  mkdir -p "$LOG_DIR"
  if [ -f "$GZ_PLUGIN_FIX" ]; then
    bash "$GZ_PLUGIN_FIX" apply >/dev/null 2>&1 || true
  fi
  if [ -f "$GZ_MODEL_FIX" ]; then
    bash "$GZ_MODEL_FIX" apply >/dev/null 2>&1 || true
  fi
  local pid
  pid="$(px4_pid)"
  if [ -n "$pid" ]; then
    echo "[sim] PX4 已在运行 (pid=$pid)，先 stop 再 start"
    return 1
  fi
  # PX4 不在了但 gz 还活着（上次被外部终止留下的孤儿 gz 很常见）：
  # 这种状态下 PX4 会打印 "gazebo already running world" 并挂到一个没有无人机的旧世界上。
  if pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "[sim] 检测到没有 PX4 的孤儿 gz，先清理"
    hard_clean
  fi
  if [ ! -x "$BUILD_DIR/bin/px4" ]; then
    echo "[sim] 找不到 $BUILD_DIR/bin/px4，请先在 $PX4_DIR 执行 make px4_sitl" >&2
    return 1
  fi

  echo "[sim] 启动 PX4 SITL: model=$SIM_MODEL world=$WORLD engine=$RENDER_ENGINE"
  : > "$LOG_FILE"
  (
    cd "$ROOTFS" || exit 1
    # gz_env.sh 里有 $GZ_SIM_RESOURCE_PATH 这类可能未定义的变量，先关掉 nounset
    set +u
    # shellcheck disable=SC1091
    source ./gz_env.sh
    export GZ_SIM_RESOURCE_PATH="$OWN_MODELS:$GZ_SIM_RESOURCE_PATH"
    set -u
    exec env \
      HEADLESS=1 \
      PX4_GZ_SIM_RENDER_ENGINE="$RENDER_ENGINE" \
      PX4_SIM_MODEL="$SIM_MODEL" \
      PX4_GZ_WORLD="$WORLD" \
      GZ_IP=127.0.0.1 \
      setsid nohup ../bin/px4 -d
  ) >>"$LOG_FILE" 2>&1 </dev/null &

  local ready=0
  for _ in $(seq 1 150); do
    sleep 1
    if grep -qE "Startup script returned successfully|Ready for takeoff" "$LOG_FILE" 2>/dev/null; then
      ready=1
      break
    fi
    if ! pgrep -f "bin/px4 -d" >/dev/null 2>&1; then
      echo "[sim] PX4 进程已退出，日志尾部：" >&2
      tail -n 30 "$LOG_FILE" >&2
      return 1
    fi
  done

  pid="$(pgrep -n -f "bin/px4 -d" || true)"
  [ -n "$pid" ] && echo "$pid" > "$PID_FILE"

  if [ "$ready" -eq 1 ]; then
    echo "[sim] PX4 就绪 (pid=${pid:-?})，MAVLink 14580 / 日志 $LOG_FILE"
  else
    echo "[sim] 150 s 内未就绪，请查看 $LOG_FILE" >&2
    return 1
  fi
}

stop() {
  local pid
  pid="$(px4_pid)"
  if [ -n "$pid" ]; then
    echo "[sim] 结束 PX4 pid=$pid"
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$pid" 2>/dev/null || true
  else
    echo "[sim] 没有记录到 PX4 进程"
  fi
  rm -f "$PID_FILE"

  # 残留的 gz / px4 一并清掉。必须按 /proc 逐个匹配再 kill：
  # 用 pkill -f 时，只要调用方的命令行里出现同样的字面量，就会把调用方自己杀掉。
  hard_clean
  echo "[sim] 已停止"
}

# 扫描 /proc 找出残留的仿真进程并强杀，直到 gz 确实消失
hard_clean() {
  local p cmd n=0
  for p in /proc/[0-9]*; do
    cmd="$(tr '\0' ' ' <"$p/cmdline" 2>/dev/null)" || continue
    case "$cmd" in
      *"gz sim"*|*"bin/px4 -d"*)
        kill -9 "${p#/proc/}" 2>/dev/null && n=$((n + 1))
        ;;
    esac
  done
  if [ "$n" -gt 0 ]; then
    echo "[sim] 强杀 $n 个残留仿真进程"
  fi
  for _ in $(seq 1 15); do
    if ! pgrep -f "gz sim" >/dev/null 2>&1 && ! pgrep -f "bin/px4 -d" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
}

status() {
  local pid
  pid="$(px4_pid)"
  if [ -z "$pid" ]; then
    echo "[sim] PX4 未运行"
    return 1
  fi
  echo "[sim] PX4 运行中 pid=$pid"
  echo "[sim] 世界 RTF / 迭代数："
  timeout 8 gz topic -e -t "/world/$WORLD/stats" -n 1 2>/dev/null |
    grep -E "real_time_factor|iterations" || echo "  (取不到 /world/$WORLD/stats)"
  echo "[sim] 相机话题数：$(gz topic -l 2>/dev/null | grep -c 'sensor/camera/image' || true)"
}

cmd="${1:-}"
world_arg="${2:-}"
if [ -n "$world_arg" ]; then
  WORLD="$world_arg"
fi

case "$cmd" in
  start) start ;;
  stop) stop ;;
  restart) stop; start ;;
  status) status ;;
  *) usage ;;
esac
