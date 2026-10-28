#!/usr/bin/env bash
# 逐个变体验证：PX4 用 sdf 字符串(版本 1.6 + include) 创建模型时会不会丢传感器。
# 三个变体依次累加进同一个空世界，每次读 scene/info 数 imu_sensor 个数（累加值）。
set -o pipefail

PX4_MODELS=/home/luosury/PX4-Autopilot/Tools/simulation/gz/models
MODEL_NAME=x500_mono_cam_down
MODEL_FILE="$PX4_MODELS/$MODEL_NAME/model.sdf"
SRV_CFG=$HOME/ArUco_Drone/ArUco_Drone_ws/src/drone_bringup/config/gz_server.config
WORLD_FILE=/tmp/sv_world.sdf
WORLD_NAME=svtest
GZLOG=/tmp/sv_gz.log
SCENE=/tmp/sv_scene.txt

export GZ_SIM_RESOURCE_PATH="$PX4_MODELS"
if [ -f "$SRV_CFG" ]; then export GZ_SIM_SERVER_CONFIG_PATH="$SRV_CFG"; fi

cat >"$WORLD_FILE" <<EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="$WORLD_NAME">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
  </world>
</sdf>
EOF

gz sim -r -s "$WORLD_FILE" --render-engine ogre >"$GZLOG" 2>&1 &
GZPID=$!
sleep 10

count_sensors() {
  gz service -s "/world/$WORLD_NAME/scene/info" \
    --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 15000 -r "" >"$SCENE" 2>/dev/null
  printf 'imu=%s baro=%s mag=%s cam=%s\n' \
    "$(grep -c imu_sensor "$SCENE")" \
    "$(grep -c air_pressure_sensor "$SCENE")" \
    "$(grep -c magnetometer_sensor "$SCENE")" \
    "$(grep -c 'type: \"camera\"' "$SCENE")"
}

spawn_sdf_string() {
  local ver="$1"
  local name="$2"
  local sdf_str="<sdf version=\\\"$ver\\\"> <include> <uri>file://$MODEL_FILE</uri> </include> </sdf>"
  gz service -s "/world/$WORLD_NAME/create" \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 15000 \
    -r "name: \"$name\", allow_renaming: false, sdf: '$sdf_str'" >/tmp/sv_create.txt 2>&1
  echo "create($name) -> $(tr -d '\n' </tmp/sv_create.txt)"
}

echo "=== 变体1: sdf 字符串 version=1.6 + include(PX4 的实际做法) ==="
spawn_sdf_string 1.6 v16
sleep 4
count_sensors

echo "=== 变体2: sdf 字符串 version=1.9 + include ==="
spawn_sdf_string 1.9 v19
sleep 4
count_sensors
echo "（变体2 的 imu 计数应比变体1 多 1 才算正常）"

echo "=== 变体3: 直接用 sdf_filename 指向 model.sdf ==="
gz service -s "/world/$WORLD_NAME/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 15000 \
  -r "name: \"vf\", allow_renaming: false, sdf_filename: \"$MODEL_FILE\"" >/tmp/sv_create.txt 2>&1
echo "create(vf) -> $(tr -d '\n' </tmp/sv_create.txt)"
sleep 4
count_sensors

echo "--- gz 日志错误行 ---"
grep -iE "error|unable|fail" "$GZLOG" | head -6

kill -9 "$GZPID" 2>/dev/null
wait "$GZPID" 2>/dev/null
echo "[probe_spawn_variants] 完成"
