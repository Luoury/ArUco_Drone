#!/usr/bin/env bash
# 手动测试 / 看画面的统一入口。
#
#   bash scripts/manual_demo.sh start     起仿真 + 桥接 + 检测 + 降落控制器 + 任务调度（可飞）
#   bash scripts/manual_demo.sh gui       打开 Gazebo 3D 窗口（看无人机、地面码、飞行）
#   bash scripts/manual_demo.sh view      打开图像窗口（/aruco/debug_image：角点框 + 坐标轴 + ID + 距离）
#   bash scripts/manual_demo.sh view raw  打开图像窗口看原始相机（/camera/image_raw）
#   bash scripts/manual_demo.sh teleop    键盘遥控（必须在自己开的交互终端里跑）
#   bash scripts/manual_demo.sh status   看 RTF / 相机帧 / 关键话题
#   bash scripts/manual_demo.sh stop     停掉仿真、所有节点和已打开的窗口
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ArUco_Drone_ws"
CMD="${1:-help}"
ARG="${2:-}"

setup_env() {
  # 不能开 set -u：source ROS 环境会引用未定义变量
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash" >/dev/null 2>&1
}

case "$CMD" in
  start)
    setup_env
    cd "$ROOT" || exit 1
    echo "[manual] 清理旧进程…"
    bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
    echo "[manual] 起 SITL（含 IMU 健康检查，约 60~90 s）…"
    bash "$ROOT/scripts/start_until_healthy.sh" aruco 3 || exit 1
    echo "[manual] 起 F2 链路（桥接 + MAVROS + 检测 + 降落控制器 + 任务调度）…"
    setsid nohup ros2 launch drone_bringup flow_f2.launch.py \
      > /tmp/manual_demo.log 2>&1 < /dev/null &
    for i in $(seq 1 60); do
      if grep -q "mission_manager 就绪" /tmp/manual_demo.log 2>/dev/null &&
         grep -q "发布 /aruco/detection" /tmp/manual_demo.log 2>/dev/null; then
        echo "[manual] 全部就绪（${i}s）"
        break
      fi
      sleep 1
    done
    echo
    echo "接下来在**另外的终端**里："
    echo "  bash scripts/manual_demo.sh gui      # 3D 世界"
    echo "  bash scripts/manual_demo.sh view     # 相机 + 检测叠加"
    echo "  bash scripts/manual_demo.sh teleop   # 键盘飞（按 h 看键位）"
    echo
    bash "$ROOT/scripts/sim.sh" status
    ;;

  gui)
    setup_env
    # GUI 是独立进程连到正在跑的 gz server；用 Ogre1（WSL2 无 GPU，Ogre2 会卡死）
    # shellcheck disable=SC1091
    source /home/luosury/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh >/dev/null 2>&1
    echo "[manual] 打开 Gazebo 3D 窗口（关掉窗口即可退出，不影响仿真）"
    exec gz sim -g --render-engine ogre
    ;;

  view)
    setup_env
    if [ "$ARG" = "raw" ]; then
      TOPIC=/camera/image_raw
    else
      TOPIC=/aruco/debug_image
    fi
    echo "[manual] 打开图像窗口：$TOPIC"
    exec ros2 run rqt_image_view rqt_image_view "$TOPIC"
    ;;

  teleop)
    setup_env
    cd "$ROOT" || exit 1
    if [ ! -t 0 ]; then
      echo "错误：键盘遥控需要交互式终端。请在 Windows 里打开一个 WSL 终端再运行本命令。" >&2
      exit 2
    fi
    exec ros2 run drone_teleop teleop_node --ros-args \
      --params-file "$WS/install/drone_bringup/share/drone_bringup/config/flows.yaml"
    ;;

  status)
    setup_env
    bash "$ROOT/scripts/sim.sh" status || true
    echo "--- 关键话题 ---"
    for t in /clock /camera/image_raw /aruco/detection /mavros/state /mission/status; do
      n=$(ros2 topic info "$t" --no-daemon 2>/dev/null | grep -c "Publisher count: [1-9]" || true)
      if [ "$n" -gt 0 ]; then echo "  $t: 有发布者"; else echo "  $t: 无发布者"; fi
    done
    ;;

  stop)
    echo "[manual] 停 ROS 节点…"
    bash "$ROOT/scripts/stop_ros.sh" >/dev/null 2>&1 || true
    echo "[manual] 关 GUI 窗口…"
    pkill -f "gz sim -g" >/dev/null 2>&1 || true
    pkill -f rqt_image_view >/dev/null 2>&1 || true
    echo "[manual] 停 SITL…"
    bash "$ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true
    echo "[manual] 已全部停止"
    ;;

  *)
    sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
esac
