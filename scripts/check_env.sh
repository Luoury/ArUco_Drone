#!/usr/bin/env bash
# 环境自检（需求书 §2）：一次性打印所有依赖版本与关键路径，便于排查"环境不对"。
# 注意：这里**不能**开 `set -u` —— source /opt/ros/humble/setup.bash 会引用
# 未定义的 AMENT_TRACE_SETUP_FILES，开了 -u 会直接退出（本脚本第一版就栽在这）。
set -o pipefail

echo "==== 系统 ===="
uname -a
echo "WSL: $(grep -m1 . /proc/version 2>/dev/null || true)"
[ -d /dev/dri ] && echo "/dev/dri: 存在（有 GPU 直通）" || echo "/dev/dri: 不存在（走软件渲染，必须用 Ogre1）"

echo "==== ROS 2 ===="
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  echo "ROS_DISTRO=$ROS_DISTRO  rclcpp=$(ros2 pkg prefix rclcpp 2>/dev/null || echo '缺失')"
else
  echo "未找到 /opt/ros/humble/setup.bash"
fi
# 运行时真正依赖的包（ros_gz_bridge 已弃用：apt 版是 Fortress ABI，与 Harmonic 的
# gz-transport13 不兼容，相机/时钟改由本仓库的 drone_bringup/gz_sensor_bridge 直连）
for pkg in mavros mavros_msgs mavros_extras rclcpp sensor_msgs rosgraph_msgs geometry_msgs tf2 tf2_geometry_msgs cv_bridge std_msgs std_srvs; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "  $pkg: 已安装"
  else
    echo "  $pkg: 缺失"
  fi
done

echo "==== Gazebo ===="
command -v gz >/dev/null 2>&1 && gz sim --versions 2>/dev/null | head -3 || echo "gz 未安装"
echo "  gz 路径: $(command -v gz || echo '无')"
for lib in gz-transport13 gz-msgs10; do
  if pkg-config --exists "$lib" 2>/dev/null; then
    echo "  $lib: $(pkg-config --modversion "$lib")"
  else
    echo "  $lib: 缺失（drone_bringup 的 gz_sensor_bridge 需要）"
  fi
done
if [ -f /usr/include/ignition/transport11/ignition/transport.hh ] ||    pkg-config --exists ignition-transport11 2>/dev/null; then
  echo "  警告: 系统里还有 Fortress(ignition) 版的传输库，别用 ros_gz_bridge（ABI 不匹配）"
fi

echo "==== PX4 ===="
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ -d "$PX4_DIR/.git" ]; then
  echo "  路径: $PX4_DIR"
  echo "  版本: $(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo '未知')"
  echo "  分支: $(git -C "$PX4_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '未知')"
  for f in build/px4_sitl_default/bin/px4 build/px4_sitl_default/rootfs/gz_env.sh; do
    [ -e "$PX4_DIR/$f" ] && echo "  $f: 存在" || echo "  $f: 缺失（需 make px4_sitl）"
  done
  if [ -n "${PX4_GZ_SIM_RENDER_ENGINE:-}" ]; then
    echo "  PX4_GZ_SIM_RENDER_ENGINE=$PX4_GZ_SIM_RENDER_ENGINE"
  else
    echo "  PX4_GZ_SIM_RENDER_ENGINE 未设置（脚本 scripts/sim.sh 会自动设成 ogre）"
  fi
else
  echo "  未找到 $PX4_DIR"
fi

echo "==== 工作区 ===="
for d in "$HOME/ArUco_Drone" "$HOME/ArUco_Drone/ArUco_Drone_ws/src"; do
  [ -d "$d" ] && echo "  $d: 存在" || echo "  $d: 缺失"
done
