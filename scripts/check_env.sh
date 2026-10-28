#!/usr/bin/env bash
# 环境自检（需求书 §2）：一次性打印所有依赖版本与关键路径，便于排查"环境不对"。
set -uo pipefail

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
for pkg in mavros mavros_msgs mavros_extras ros_gz_bridge ros_gz_sim ros_gz_image ros_gz_interfaces cv_bridge; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "  $pkg: 已安装"
  else
    echo "  $pkg: 缺失"
  fi
done

echo "==== Gazebo ===="
command -v gz >/dev/null 2>&1 && gz sim --versions 2>/dev/null | head -3 || echo "gz 未安装"
echo "  gz 路径: $(command -v gz || echo '无')"

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
