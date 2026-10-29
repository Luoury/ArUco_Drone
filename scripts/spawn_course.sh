#!/usr/bin/env bash
# 摆放多工位演示场景：3 个额外 ArUco 码（id 1/2/3）+ 一台斜俯视演示机位。
#
#   bash scripts/spawn_course.sh [world]     默认 world = aruco
#
# 工位布局（基站是世界里本来就有的 id=0，在原点）：
#       C(0,2.5)     B(2.5,2.5)
#          ┌─────────┐
#          │         │
#      基地(0,0)    A(2.5,0)
#
# 幂等：先删同名旧实体再创建。
#
# 三个实测坑：
#   1) `gz service create` 返回的 `data: true` **不代表实体建出来了** —— 文件找不到、
#      SDF 解析失败时它同样返回 true。必须回读世界（scene/info 里的模型名）确认。
#   2) `gz model -m X -p` 对静态模型不打印 Pose 行，别拿它当存在判据。
#   3) create 时传 `pose` 会**覆盖** model.sdf 里的 <pose>。像 demo_cam 这种把位姿
#      写在文件里的模型，绝不能传 pose，否则会落到原点、镜头怼在无人机身上。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-aruco}"
MODELS="$ROOT/ArUco_Drone_ws/src/drone_bringup/models"

remove_model() {
  gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 2000 -r "name: \"$1\", type: 2" >/dev/null 2>&1 || true
}

# spawn_model <名字> <sdf 路径> [x y z]
# 不给坐标就用 model.sdf 里自带的位姿
spawn_model() {
  local name="$1" sdf="$2" x="${3:-}" y="${4:-}" z="${5:-}" req resp
  remove_model "$name"
  if [ ! -f "$sdf" ]; then
    echo "[course] 模型文件不存在：$sdf" >&2
    return 1
  fi
  if [ -n "$x" ]; then
    req="sdf_filename: \"$sdf\", name: \"$name\", allow_renaming: false, "
    req="${req}pose: {position: {x: $x, y: $y, z: $z}}"
  else
    req="sdf_filename: \"$sdf\", name: \"$name\", allow_renaming: false"
  fi
  resp=$(gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 -r "$req" 2>&1)
  case "$resp" in
    *"data: true"*) return 0 ;;
    *) echo "[course] $name 创建被拒：$resp" >&2; return 1 ;;
  esac
}

echo "[course] 清理旧的演示实体"
for n in aruco_1 aruco_2 aruco_3 demo_cam; do remove_model "$n"; done

echo "[course] 摆放 3 个 ArUco 工位 + 斜俯视机位"
spawn_model aruco_1 "$MODELS/arucotag_1/model.sdf" 2.5 0.0 0.001 || exit 1
spawn_model aruco_2 "$MODELS/arucotag_2/model.sdf" 2.5 2.5 0.001 || exit 1
spawn_model aruco_3 "$MODELS/arucotag_3/model.sdf" 0.0 2.5 0.001 || exit 1
spawn_model demo_cam "$MODELS/demo_cam/model.sdf" || exit 1
sleep 2

SCENE=$(gz service -s "/world/$WORLD/scene/info" --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene --timeout 15000 -r "" 2>/dev/null)
missing=0
for n in aruco_1 aruco_2 aruco_3 demo_cam; do
  if echo "$SCENE" | grep -q "name: \"$n\""; then
    echo "[course] 确认存在：$n"
  else
    echo "[course] 世界查不到：$n" >&2
    missing=1
  fi
done
[ "$missing" -eq 0 ] || exit 1

CAM_TOPIC="/world/$WORLD/model/demo_cam/link/link/sensor/camera/image"
for i in $(seq 1 30); do
  if gz topic -i -t "$CAM_TOPIC" 2>/dev/null | grep -q "Publishers"; then
    echo "[course] 机位就绪：$CAM_TOPIC"
    exit 0
  fi
  sleep 1
done

echo "[course] 失败：$CAM_TOPIC 一直没有发布者" >&2
exit 1
