#!/usr/bin/env bash
# 把 Windows 暂存目录里的源码同步进 WSL 的工作区。
# 之所以单独成脚本：在 wsl 命令行里用 $VAR 会被中间那层解析吃掉（本会话反复踩到），
# 变量只能写在脚本文件里才安全。
set -euo pipefail

STAGE=/mnt/c/Users/Administrator/AppData/Local/Temp/aruco_stage
ROOT="$HOME/ArUco_Drone"
WS="$ROOT/ArUco_Drone_ws"

mkdir -p "$ROOT/scripts" "$ROOT/logs" "$WS/src"

if [ -d "$STAGE/scripts" ]; then
  cp -f "$STAGE"/scripts/*.sh "$ROOT/scripts/"
  chmod +x "$ROOT"/scripts/*.sh
fi

if [ -d "$STAGE/src" ]; then
  cp -rf "$STAGE"/src/. "$WS/src/"
fi

if [ -d "$STAGE/test" ]; then
  mkdir -p "$ROOT/test"
  cp -f "$STAGE"/test/*.sdf "$ROOT/test/" 2>/dev/null || true
fi

echo "[sync] 完成 -> $WS/src"
ls -1 "$WS/src"
