#!/usr/bin/env bash
# 把 PX4 默认的 gz 插件清单替换成本仓库的版本（去掉会泄漏内存的 custom 插件）。
#
# 为什么必须替换（实测数据，M0 记录）：
#   PX4 v1.17 的 server.config 默认加载 custom::GstCameraSystem，
#   它用正则 /world/(.+)/model/(.+)/link/(.+)/sensor/(.+)/image 订阅所有相机图像，
#   每帧把 1280x960x3 拷进 GStreamer pipeline 却释放不掉 —— gz sim 的 RSS
#   以 ~105 MB/s 线性增长（≈ 1280*960*3*30），90 s 就把 7.7 GB 的 WSL2 吃光、
#   整机卡死（本机因此崩过 4 次）。
#   去掉 GstCameraSystem + OpticalFlowSystem 后：gz RSS 恒定 470 MB，RTF 1.0006。
#
# 注意：不能只设 GZ_SIM_SERVER_CONFIG_PATH —— PX4 启动脚本
#   ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim 会自己 `source ./gz_env.sh`，
#   里面把 GZ_SIM_SERVER_CONFIG_PATH 又指回 PX4 的 server.config，覆盖掉外部设置。
#   所以只能直接替换那个文件（本脚本做的事，带备份、可还原）。
set -uo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
TARGET="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"
BACKUP="$TARGET.px4bak"
SOURCE="${SOURCE:-$HOME/ArUco_Drone/ArUco_Drone_ws/src/drone_bringup/config/gz_server.config}"

apply() {
  if [ ! -f "$SOURCE" ]; then
    echo "[gz-plugins] 缺少 $SOURCE" >&2
    return 1
  fi
  if [ ! -f "$TARGET" ]; then
    echo "[gz-plugins] 找不到 PX4 的 $TARGET" >&2
    return 1
  fi
  if grep -q "GstCameraSystem" "$TARGET"; then
    [ -f "$BACKUP" ] || cp "$TARGET" "$BACKUP"
    cp "$SOURCE" "$TARGET"
    echo "[gz-plugins] 已替换 PX4 插件清单（备份：$BACKUP）"
  else
    echo "[gz-plugins] PX4 插件清单已是安全版本"
  fi
}

restore() {
  if [ -f "$BACKUP" ]; then
    cp "$BACKUP" "$TARGET"
    rm -f "$BACKUP"
    echo "[gz-plugins] 已还原 PX4 默认清单"
  else
    echo "[gz-plugins] 没有备份文件，无需还原"
  fi
}

show() {
  grep -n "plugin filename" "$TARGET"
}

case "${1:-apply}" in
  apply) apply ;;
  restore) restore ;;
  show) show ;;
  *) echo "用法：bash fix_gz_plugins.sh [apply|restore|show]"; exit 1 ;;
esac
