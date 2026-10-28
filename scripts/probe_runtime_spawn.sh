#!/usr/bin/env bash
# 验证假设：gz sim 在"运行时用 EntityFactory 服务创建模型"时，会不会丢掉 merge 进来的 <sensor>。
# 做法：起一个空世界 -> 用 /world/<w>/create 服务把模型塞进去 -> 读 scene/info 数传感器。
set -o pipefail

MODEL=${1:-x500_mono_cam_down}
PX4_MODELS=/home/luosury/PX4-Autopilot/Tools/simulation/gz/models
MODEL_FILE="$PX4_MODELS/$MODEL/model.sdf"
SRV_CFG=$HOME/ArUco_Drone/ArUco_Drone_ws/src/drone_bringup/config/gz_server.config
WORLD_FILE=/tmp/rs_world.sdf
WORLD_NAME=rstest
GZLOG=/tmp/rs_gz.log
SCENE=/tmp/rs_scene.txt

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

REQ="name: \"${MODEL}_0\", sdf_filename: \"$MODEL_FILE\", allow_renaming: false"
gz service -s "/world/$WORLD_NAME/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 15000 -r "$REQ" > /tmp/rs_create.txt 2>&1
echo "--- create 返回 ---"
cat /tmp/rs_create.txt
sleep 4

gz service -s "/world/$WORLD_NAME/scene/info" \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 15000 -r "" \
  >"$SCENE" 2>&1

echo "===== 运行时 create 的模型: $MODEL ====="
echo "imu_sensor        = $(grep -c imu_sensor "$SCENE")"
echo "air_pressure      = $(grep -c air_pressure_sensor "$SCENE")"
echo "magnetometer      = $(grep -c magnetometer_sensor "$SCENE")"
echo "navsat            = $(grep -c navsat_sensor "$SCENE")"
echo "camera sensor     = $(grep -c 'type: \"camera\"' "$SCENE")"
echo "--- 场景里的 name 列表（前 25）---"
grep -oE 'name: "[A-Za-z0-9_]+"' "$SCENE" | sed -n '1,25p' | tr '\n' ' '
echo
echo "--- gz 日志错误行 ---"
grep -iE "error|unable|not found|fail" "$GZLOG" | head -5

kill -9 "$GZPID" 2>/dev/null
wait "$GZPID" 2>/dev/null
echo "[probe_runtime_spawn] 完成"
