#!/usr/bin/env bash
# 把 Windows 暂存目录里的源码同步进 WSL 的工作区。
# 之所以单独成脚本：在 wsl 命令行里用 $VAR 会被中间那层解析吃掉（本会话反复踩到），
# 变量只能写在脚本文件里才安全。
#
# 两个血泪教训都体现在这个文件里：
#   1. 只覆盖"暂存更新"的文件（cp -u）。M1 阶段用旧暂存无条件 cp -rf，
#      把 M0 末期写的 takeoffTo 源码覆盖成了没有它的旧版本，只剩 install/ 里
#      的编译产物还留着符号。需要强制覆盖时用 SYNC_FORCE=1。
#   2. 判据不能用 mtime（`-nt` / `cp -u`）。/mnt/c 是 DrvFs，mtime 有秒级/时区误差，
#      实测改过的脚本"看起来更旧"，同步被静默跳过（M5 就这样白跑了一轮 F4）。
#      现在一律用 `cmp -s` 比内容：内容相同才跳过。
#   3. 脚本类文件必须"先写临时名再 mv"。因为本脚本自己也在被同步的目录里，
#      就地 cp 覆盖会一边执行一边被截断，bash 直接报
#      "unexpected EOF while looking for matching"，一个文件都没同步成。
#      mv 换的是目录项，正在执行的 bash 仍持旧 inode，安全。
set -euo pipefail

STAGE=/mnt/c/Users/Administrator/AppData/Local/Temp/aruco_stage
ROOT="$HOME/ArUco_Drone"
WS="$ROOT/ArUco_Drone_ws"

FORCE="${SYNC_FORCE:-0}"
if [ "$FORCE" = "1" ]; then
  echo "[sync] SYNC_FORCE=1：无条件覆盖"
fi

# $1 = 源文件，$2 = 目标目录：按需复制，且永不就地覆盖
sync_file() {
  local src="$1" dst_dir="$2" dst tmp
  dst="$dst_dir/$(basename "$src")"
  if [ "$FORCE" != "1" ] && [ -f "$dst" ] && cmp -s "$src" "$dst"; then
    return 0
  fi
  echo "[sync] 更新 $(basename "$src")" 
  tmp="$dst_dir/.sync.$$.tmp"
  cp -f "$src" "$tmp"
  mv -f "$tmp" "$dst"
}

mkdir -p "$ROOT/scripts" "$ROOT/logs" "$WS/src"

if [ -d "$STAGE/scripts" ]; then
  for f in "$STAGE"/scripts/*; do
    [ -f "$f" ] || continue
    sync_file "$f" "$ROOT/scripts"
  done
  chmod +x "$ROOT"/scripts/*.sh "$ROOT"/scripts/*.py 2>/dev/null || true
fi

if [ -d "$STAGE/src" ]; then
  if [ "$FORCE" = "1" ]; then
    cp -rf "$STAGE"/src/. "$WS/src/"
  else
    # --checksum：按内容而不是 mtime 判断，见文件头教训 2
    rsync -a --checksum "$STAGE/src/" "$WS/src/"
  fi
fi

if [ -d "$STAGE/docs" ]; then
  mkdir -p "$ROOT/docs/images"
  for f in "$STAGE"/docs/*.md; do
    [ -f "$f" ] || continue
    sync_file "$f" "$ROOT/docs"
  done
  for f in "$STAGE"/docs/images/*.png; do
    [ -f "$f" ] || continue
    sync_file "$f" "$ROOT/docs/images"
  done
fi

echo "[sync] 完成 -> $WS/src"
ls -1 "$WS/src"
