#!/usr/bin/env bash
# 探查任意 PX4 gz 模型被 gz sim 加载后，场景里到底有没有 base_link 的传感器。
# 用法: probe_sensors.sh [模型名]  默认 x500_mono_cam_down
# 结论依据: /world/<world>/scene/info 的 Scene 消息里会列出每个 link 下的 <sensor>。
set -o pipefail

MODEL=${1:-x500_mono_cam_down}
PX4_MODELS=/home/luosury/PX4-Autopilot/Tools/simulation/gz/models
SRV_CFG=$HOME/ArUco_Drone/ArUco_Drone_ws/src/drone_bringup/config/gz_server.config
WORLD_FILE=/tmp/sp_world.sdf
SCENE=/tmp/sp_scene.txt
GZLOG=/tmp/sp_gz.log
WORLD_NAME=sptest

export GZ_SIM_RESOURCE_PATH="$PX4_MODELS"
if [ -f "$SRV_CFG" ]; then
  export GZ_SIM_SERVER_CONFIG_PATH="$SRV_CFG"
fi

cat >"$WORLD_FILE" <<EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="$WORLD_NAME">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <include>
      <uri>model://$MODEL</uri>
    </include>
  </world>
</sdf>
EOF

gz sim -r -s "$WORLD_FILE" --render-engine ogre >"$GZLOG" 2>&1 &
GZPID=$!
sleep 12

gz service -s "/world/$WORLD_NAME/scene/info" \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 15000 -r "" \
  >"$SCENE" 2>&1

echo "===== 模型: $MODEL ====="
echo "imu_sensor        = $(grep -c imu_sensor "$SCENE")"
echo "air_pressure      = $(grep -c air_pressure_sensor "$SCENE")"
echo "magnetometer      = $(grep -c magnetometer_sensor "$SCENE")"
echo "navsat            = $(grep -c navsat_sensor "$SCENE")"
echo "camera sensor     = $(grep -c 'type: "camera"' "$SCENE")"
echo "--- 场景里出现的 link 名 ---"
grep -oE 'name: "[A-Za-z0-9_]+"' "$SCENE" | sed -n '1,25p' | tr '\n' ' '
echo
echo "--- gz 日志中的错误行 ---"
grep -iE "error|unable|not found|fail" "$GZLOG" | head -5

kill -9 "$GZPID" 2>/dev/null
wait "$GZPID" 2>/dev/null
echo "[probe_sensors] 完成"
