#!/usr/bin/env bash
# 把 PX4 的无人机模型"展平"成单文件 SDF，并让 PX4 用展平版 spawn。
#
# 为什么需要：
#   x500_mono_cam_down → include(merge=true) x500 → include(merge=true) x500_base。
#   这条 include/merge 链在 `gz sim` 8.10 里 spawn 时会把 base_link 上的 4 个传感器
#   （imu_sensor / air_pressure_sensor / magnetometer_sensor / navsat_sensor）丢掉，
#   表现是 PX4 一直报 `Preflight Fail: Accel Sensor 0 missing / barometer 0 missing /
#   Found 0 compass`，`gz topic -i` 里 IMU 话题只有订阅者没有发布者（实测 scene/info
#   里 x500_mono_cam_down_0 只剩 camera 一个传感器）。
#   用 `gz sdf -p` 展平后单文件里 5 个传感器齐全，spawn 时不再走 merge，问题消失。
#
# 用法：fix_gz_model.sh apply|restore|show
set -uo pipefail

PX4_MODELS="$HOME/PX4-Autopilot/Tools/simulation/gz/models"
MODEL_NAME="x500_mono_cam_down"
OUT_ROOT="$HOME/ArUco_Drone/.cache/gz_models"
OUT_DIR="$OUT_ROOT/$MODEL_NAME"
GZ_ENV="$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
BAK="${GZ_ENV}.px4bak"

apply() {
  rm -rf "$OUT_ROOT"
  mkdir -p "$OUT_DIR"

  # PX4_GZ_MODELS 会被改成 $OUT_ROOT，而 GZ_SIM_RESOURCE_PATH 用的是同一个变量，
  # 所以要把 PX4 原有 models 目录里的其它模型（arucotag / mono_cam / ...）用软链挂进来。
  local entry base
  for entry in "$PX4_MODELS"/*; do
    [ -e "$entry" ] || continue
    base="$(basename "$entry")"
    [ "$base" = "$MODEL_NAME" ] && continue
    ln -sfn "$entry" "$OUT_ROOT/$base"
  done

  SDF_PATH="$PX4_MODELS" GZ_SIM_RESOURCE_PATH="$PX4_MODELS" \
    gz sdf -p "$PX4_MODELS/$MODEL_NAME/model.sdf" > "$OUT_DIR/model.sdf" 2>/dev/null

  local n
  n=$(grep -c "<sensor " "$OUT_DIR/model.sdf" 2>/dev/null || echo 0)
  if [ "$n" -lt 5 ]; then
    echo "[gz_model] 展平失败：只有 $n 个传感器，放弃覆盖"
    return 1
  fi

  cp -f "$PX4_MODELS/$MODEL_NAME/model.config" "$OUT_DIR/model.config" 2>/dev/null || true

  if [ ! -f "$BAK" ]; then
    cp -f "$GZ_ENV" "$BAK"
  fi
  python3 - "$GZ_ENV" "$OUT_ROOT" <<'PY'
import io, re, sys
path, root = sys.argv[1], sys.argv[2]
s = io.open(path, encoding="utf-8").read()
s2 = re.sub(r"^export PX4_GZ_MODELS=.*$", "export PX4_GZ_MODELS=" + root,
            s, count=1, flags=re.M)
if s2 == s:
    print("[gz_model] 没找到 PX4_GZ_MODELS 行")
    sys.exit(1)
io.open(path, "w", encoding="utf-8").write(s2)
PY

  if grep -q "^export PX4_GZ_MODELS=${OUT_ROOT}$" "$GZ_ENV"; then
    echo "[gz_model] gz_env.sh 已指向展平目录（幂等）"
  else
    echo "[gz_model] 警告：gz_env.sh 里没找到可改写的 PX4_GZ_MODELS 行"
  fi
  echo "[gz_model] 已用展平模型：$OUT_DIR/model.sdf（传感器 $n 个）"
  echo "[gz_model] PX4_GZ_MODELS -> $OUT_ROOT（其余 $(( $(ls -1 "$OUT_ROOT" | wc -l) - 1 )) 个模型为软链）"
}

restore() {
  if [ -f "$BAK" ]; then
    cp -f "$BAK" "$GZ_ENV"
    echo "[gz_model] 已恢复原始 gz_env.sh"
  else
    echo "[gz_model] 没有备份，无需恢复"
  fi
}

show() {
  echo "[gz_model] GZ_ENV=$GZ_ENV"
  grep -n "PX4_GZ_MODELS" "$GZ_ENV" 2>/dev/null || true
  if [ -f "$OUT_DIR/model.sdf" ]; then
    echo "[gz_model] 展平文件 $(wc -l < "$OUT_DIR/model.sdf") 行，传感器 $(grep -c '<sensor ' "$OUT_DIR/model.sdf") 个"
  else
    echo "[gz_model] 还没有展平文件"
  fi
}

case "${1:-show}" in
  apply) apply ;;
  restore) restore ;;
  show) show ;;
  *) echo "用法: $0 apply|restore|show"; exit 2 ;;
esac
