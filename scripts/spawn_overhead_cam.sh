#!/usr/bin/env bash
# 在正在运行的 gz 世界里挂一台"俯视机位"（录视频用）。
#   bash scripts/spawn_overhead_cam.sh [world]     默认 world = aruco
# 幂等：先删掉同名旧模型再创建。
#
# 两个实测坑（都踩过）：
#   1) `-r` 请求串里的字符串值必须用**双引号**：传单引号（如 name: 'overhead_cam'）
#      gz 的文本格式解析器不认，服务照样返回 true 但实体根本没建。
#   2) 判断有没有挂上，要看 gz 话题有没有发布者；
#      `gz model -m X -p` 在还没收到世界状态时会先打 "Requesting state for world ..."，
#      拿它做成功判据会误判成失败。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-aruco}"
MODEL=overhead_cam
MODEL_SDF="$ROOT/ArUco_Drone_ws/src/drone_bringup/models/overhead_cam/model.sdf"
TOPIC="/world/$WORLD/model/$MODEL/link/link/sensor/camera/image"

if [ ! -f "$MODEL_SDF" ]; then
  echo "[overhead] 找不到模型文件：$MODEL_SDF" >&2
  exit 1
fi

echo "[overhead] 清掉同名旧模型（没有就忽略）"
gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 -r "name: \"$MODEL\", type: 2" >/dev/null 2>&1 || true

echo "[overhead] 生成俯视机位（模型文件 $MODEL_SDF）"
RESP=$(gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 5000 \
  -r "sdf_filename: \"$MODEL_SDF\", name: \"$MODEL\", allow_renaming: false" 2>&1)
case "$RESP" in
  *"data: true"*) ;;
  *)
    echo "[overhead] 失败：创建返回 $RESP" >&2
    exit 1
    ;;
esac

for i in $(seq 1 30); do
  if gz topic -i -t "$TOPIC" 2>/dev/null | grep -q "Publishers"; then
    echo "[overhead] 就位：$TOPIC 已有发布者"
    exit 0
  fi
  sleep 1
done

echo "[overhead] 失败：$TOPIC 一直没有发布者" >&2
gz topic -l 2>/dev/null | grep -i "$MODEL" >&2 || true
exit 1
