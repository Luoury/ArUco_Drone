#!/usr/bin/env bash
# 清理相机泄漏测试留下的 gz 进程。
# 之所以单独成文件：如果把 "cam_leak" 这种模式直接写在 wsl 命令行里，
# pkill -f 会匹配到执行它的那层 bash 自身，把自己杀掉（本会话已踩过）。
set -uo pipefail
pkill -9 -f 'cam_leak' 2>/dev/null || true
sleep 1
if pgrep -f 'cam_leak' >/dev/null 2>&1; then
  echo "kill_test_gz: 仍有残留"
else
  echo "kill_test_gz: 已清理"
fi
