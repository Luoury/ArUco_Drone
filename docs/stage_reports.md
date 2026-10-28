# 阶段成果报告（M0 / M1 / M2）

> 本文按里程碑逐阶段记录"做了什么、实测到什么、踩了哪些坑"。
> 数据全部来自本机真实终端输出（WSL2 Ubuntu 22.04 + PX4 v1.17 SITL + Gazebo Harmonic + ROS 2 Humble），
> 未做任何美化。后续 M3~M6 完成后在文末继续追加。
>
> 汇总这份文件即可直接转 PDF 交作业。

---

## 0. 项目概览

| 项目 | 内容 |
| --- | --- |
| 目标 | 四旋翼在 Gazebo 中识别地面 ArUco 码并全自主精准降落 |
| 仿真栈 | PX4 v1.17.0-alpha1-338 SITL + Gazebo Harmonic (gz sim 8.10.0) |
| 中间件 | MAVROS（`fcu_url:=udp://:14540@127.0.0.1:14580`） |
| 视觉 | OpenCV 4.5.4 `cv::aruco` + `SOLVEPNP_IPPE_SQUARE` |
| 工作区 | `~/ArUco_Drone/ArUco_Drone_ws`（ROS 2 工作区）+ `~/ArUco_Drone/scripts`（运维脚本） |
| 代码仓库 | `github.com/Luoury/ArUco_Drone`（private） |

### 包职责

| 包 | 语言 | 职责 |
| --- | --- | --- |
| `drone_interfaces` | msg/srv | `MissionStatus.msg`、`SetMode.srv` |
| `drone_bringup` | C++/launch/config | 仿真桥接节点 `gz_sensor_bridge`、全部 launch 与参数 yaml、gz 插件与模型修复 |
| `drone_control` | C++ 库 | 设定点发布、ENU→NED、OFFBOARD 前置、起飞/悬停/降落/航点原语 |
| `drone_aruco` | C++ | `aruco_detector`：检测 + PnP + 位姿融合 + debug 图 |
| `drone_mission` | C++ | `mission_manager`：模式调度、遥控指令唯一出口、状态发布 |
| `drone_teleop` | Python | 键盘遥控 → `/teleop/cmd`、`/mission/mode` |
| `drone_tools` | Python | `verify_flow`：流程自动验收 |

### 关键话题契约

| 话题 | 类型 | 方向 |
| --- | --- | --- |
| `/camera/image_raw`、`/camera/camera_info` | `sensor_msgs` | gz → ROS（自建桥接） |
| `/clock` | `rosgraph_msgs/Clock` | gz → ROS |
| `/mavros/state`、`/mavros/local_position/pose` | `mavros_msgs` / `geometry_msgs` | MAVROS |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 唯一控制出口 |
| `/aruco/detection`、`/aruco/debug_image` | `geometry_msgs/PoseStamped`、`sensor_msgs/Image` | 检测节点 |
| `/teleop/cmd` | `geometry_msgs/Twist`（body 系） | 遥控 → mission |
| `/mission/mode`、`/mission/status`、`/mission/set_mode` | `String` / `MissionStatus` / `SetMode` | 任务调度 |

---

## 1. M0 — 环境打通（PX4 + Gazebo + MAVROS + 相机 + OFFBOARD 悬停）

### 1.1 交付物

- `gz_sensor_bridge`：**自己写的** gz→ROS 桥接节点（原因见 1.4 坑 A），桥 `/clock`、相机图像、相机内参。
- `drone_control` 库 + `hover_test` 可执行：ENU/NED、type_mask、OFFBOARD 前置顺序、起飞→悬停→降落。
- `scripts/sim.sh`（起停 SITL）、`fix_gz_plugins.sh`、`fix_gz_model.sh`、`start_until_healthy.sh`、`probe_mem.sh` 等运维脚本。

### 1.2 验收结果（真实输出）

```
[healthy] 第 1 次启动成功：IMU 有发布者
[sim] PX4 运行中 pid=26089
iterations: 2259
real_time_factor: 0.98463162346308231
① AUTO.TAKEOFF 起飞（目标高度 1.50 m）
arming(true) → success=true
② 已离地 1.37 m
③ 预发位置设定点 2000 ms @ 50 Hz
SetMode(OFFBOARD) → mode_sent=true
④ OFFBOARD 已接管，armed=true
爬升结果：到达目标高度，当前 ENU (-0.01, -0.04, 1.65)
hovers_samples = 1469
final_altitude = 1.486 m（目标 1.500 m）
mean_altitude = 1.512 m
max_altitude_error = 0.146 m（容差 0.150 m）
max_horizontal_drift = 0.045 m（容差 0.300 m）
max_vertical_speed = 0.303 m/s
hover_test RESULT: PASS
降落结果：已上锁，落点 ENU (-0.01, -0.01, 0.04)，armed=false
```

相机与 MAVROS：

```
state.connected = True
state.mode = OFFBOARD
camera.frames = 118 (19.6 Hz)        # Python 探针上限，桥接自报 29.8 Hz
camera.size = (1280, 960) encoding=rgb8
camera.K = [539.936, 0, 640, 0, 539.936, 480, 0, 0, 1]
EVIDENCE RESULT: PASS
```

### 1.3 里程碑判据对照

| 判据 | 结果 |
| --- | --- |
| PX4 SITL 起飞成功 | ✅ AUTO.TAKEOFF → 1.5 m |
| `/camera/image_raw` ≈30 Hz | ✅ 28.6~29.8 Hz |
| `/mavros/state` connected | ✅ true |
| 高度随起飞变化 | ✅ 0.04 → 1.65 m |
| `hover_test` OFFBOARD 悬停 1.5 m / 30 s | ✅ PASS（最大高度误差 0.146 m 容差 0.15 m） |

### 1.4 踩坑与修复（M0 的四道坎）

**坑 A：官方 `ros_gz_bridge` 与 Harmonic 的 ABI 不匹配**
`ros-humble-ros-gz-bridge` 0.244.20 链接 `libignition-transport11`（Fortress ABI），而 `gz sim` 8.10 用 `libgz-transport13`（Harmonic ABI）。桥接进程能挂上 ROS 发布者，但 gz 侧订阅者落进另一个发现域，**永远收不到数据**。铁证：`gz topic -i -t /clock` 输出 `No subscribers on topic [/clock]`。
→ 用 `gz-transport13` + `gz-msgs10` 自写 `gz_sensor_bridge`（~200 行），实测 `/camera/image_raw` 29.6 Hz、`/clock` 正常。

**坑 B：PX4 的 GstCameraSystem 插件内存泄漏**
gz sim 的 RSS 以 **105 MB/s** 线性暴涨（= 1280×960×3 B × 30 Hz，每帧泄漏一个图像缓冲），80 s 就吃掉 7 GB，导致 WSL2 整体卡死。对照实验：不启 PX4 时 RSS 恒定 229 MB。
→ 根因是 `server.config` 默认加载的 `custom::GstCameraSystem` 订阅所有相机图像塞进 GStreamer pipeline 不释放。自写 `fix_gz_plugins.sh` 用精简清单覆盖 PX4 的 `server.config`（去掉 GstCamera 与 OpticalFlow，写死 `render_engine=ogre`）→ **RSS 恒定 470 MB**。

**坑 C：WSL2 里 Ogre2 软件渲染慢 180 倍**
默认渲染引擎下 `/world/aruco/stats` 的 `real_time_factor` 只有 **0.0056**，PX4 主循环被拖死、mavros 每 9 s 掉线一次。换 Ogre1 后 RTF **0.9816**。

**坑 D：OFFBOARD 无法从地面起飞**
位置/速度设定点、两种 arm 顺序都试过，电机始终停在怠速 151 rad/s（x500 悬停需 ~657 rad/s）。`AUTO.TAKEOFF` 则正常。
→ `FlightControl::takeoffTo()`：`AUTO.TAKEOFF` 起飞 → 离地后按 §5.3 顺序交 OFFBOARD。

**坑 E（坐标契约）**：`/mavros/setpoint_raw/local` 的 position/velocity **就是 ENU**，mavros 内部转 NED。M0 初期我们手工又转了一次（`z=-enu.z`），目标变成"地下 1.5 m"，实测从 1.28 m 扎到 -0.09 m。改回直接发 ENU 后收敛到 1.41 m。
> 已把这条写进需求书 §6.4 的坑列表。

---

## 2. M1 — ArUco 视觉检测

### 2.1 码的真值（实测标定，不是猜的）

| 项 | 值 | 依据 |
| --- | --- | --- |
| 字典 | `DICT_4X4_50` | 用 `cv2.aruco.generateImageMarker()` 反查，4X4 家族 id=0 位图一致 |
| ID | `0` | 同上，离线脚本 `m1_detect_offline.py` 输出 PASS |
| `marker_size` | **0.487 m** | 名义平面 0.5 m；多轮在线标定隐含 0.4869~0.4952，取 0.487 |
| 相机内参 | fx=fy=539.936, cx=640, cy=480 | `/camera/camera_info` 实测（`horizontal_fov=1.74`, 1280×960） |

### 2.2 交付物

- `aruco_detector`（C++）：订阅 `/camera/image_raw`，`cv::aruco` 检测 + `SOLVEPNP_IPPE_SQUARE` 解算，融合无人机全姿态到位姿，发布 `/aruco/detection`（`frame_id: map`）与 `/aruco/debug_image`。
- 参数全在 `drone_bringup/config/aruco.yaml`，代码零魔法数。
- 诊断脚本：`m1_detect_offline.py`（离线识别）、`m1_check_detection.py`（在线精度，先等稳态悬停）、`m1_detector_accuracy.py`（把检测误差与 EKF 误差分离）、`m1_save_debug.py`（存 debug 图）。

### 2.3 验收结果（真实输出）

离线：

```
[M1] dictionary = DICT_4X4_50
[M1] marker_id = 0
[M1] 检测到角点 353.9 px
[M1] RESULT: PASS（期望 id=0）
```

在线（悬停 2.00 m，`marker_size=0.487`）：

```
无人机 ENU = (-0.040, -0.004, 2.004)
detection 均值 = (-0.004, -0.016, -0.019)
三维误差中位 0.061 m / 最大 0.098
水平中位 0.056 / 最大 0.095
垂直中位 -0.020 / 范围 -0.059 ~ +0.005
隐含 marker_size 0.4952
```

误差来源分解（`m1_detector_accuracy.py`）：

```
EKF 相对 gz 真值的偏置（中位）= (+0.0087, +0.0370, +0.0447)
该偏置在 15 s 采样窗内的漂移幅度 = (+0.0629, +0.0359, +0.0373)
原始误差：三维中位 0.0659 / 水平 0.0650 / 垂直 -0.0034
```

结论：**垂直误差已被 0.487 压到 -0.02 m**；残余水平误差与 PX4 EKF 相对 gz 真值的随机游走同量级（15 s 内就漂 6 cm），扣掉常数偏置反而更差（0.0724 m）—— 说明这不是安装偏差，按单帧补偿没有意义，后续必须靠时序滤波（M3 的降落控制器里做）。

debug 图证据：`docs/images/m1_debug_image.png`（1280×960，画面含 `id=0 z_cam=2.07m frame=371` 绿字、绿色角点框、RGB 坐标轴；与真值 2.06 m 对照）。

### 2.4 踩坑与修复

**坑 F：launch 默认值悄悄覆盖 yaml**
`aruco_detect.launch.py` 里 `DeclareLaunchArgument('marker_size', default_value='0.50')` 又传给 Node，导致标定好的 yaml 值不生效 —— "改了没反应"的元凶。改为 launch 只暴露 `use_sim_time / publish_debug / marker_id / drone_pose_topic`，字典与 marker_size 一律以 yaml 为准。

**坑 G：时基不一致导致位姿配对错帧**
曾按"图像时间戳挑最近位姿"，误差从 0.03 恶化到 0.13。原因：`/mavros/local_position/pose` 的 header 是**墙钟时间**（~1.79e9 s），gz 图像是**仿真时间**（几百秒），差 1.79e9 s，按戳配对必然挑到缓冲里最旧那帧（爬升阶段）。改为取最新位姿 + mutex。

**坑 H：只用偏航角做旋转不够**
悬停时 1~3° 倾斜经 2 m 放大成厘米级误差。改为从四元数取完整 `R_map_body`，三维误差 0.146 → 0.053 m。

**坑 I：`merge="true"` 会丢掉 link 上的传感器**
`x500_mono_cam_down` → `x500` → `x500_base` 的 include 链里，`gz sdf -p` 展平后 5 个传感器齐全（air_pressure/imu/magnetometer/navsat/camera），但**运行时 spawn 只留下相机**，PX4 因此报 `Preflight Fail: Accel Sensor 0 missing` / `Gyro` / `barometer`，无法 arm。
→ `fix_gz_model.sh`：把展平后的 SDF 写到 `.cache/gz_models/`，其余 36 个模型做软链，改写 `gz_env.sh` 的 `PX4_GZ_MODELS` 指向它。修复后每次启动传感器都在。

---

## 3. M2 — 飞控封装 + 键盘 + F1 流程

### 3.1 交付物

- `drone_control` 库补齐：`takeoffTo / takeoff / land / hover / gotoPosition / velocity / yaw / publishVelocitySetpoint`。
- `drone_mission/mission_manager`（C++）：唯一控制出口；模式 `MANUAL / HOVER / LAND`（`ARUCO_LAND / WAYPOINT / AUTO` 留到 M3~M5）；body→ENU 转换；松键 300 ms 清零；贴地强制降落；10 Hz 发 `/mission/status`。
- `drone_teleop/teleop_node`（Python）：`w/s/a/d/r/f/q/e` + `Space/1/2/3/4/0/ESC/h`，termios raw 模式，20 Hz 发 `/teleop/cmd`。
- `drone_tools/verify_flow`（Python）：F1 自动验收，走与键盘完全相同的通路。
- `flow_f1.launch.py`：`sim_bringup` + `mission_manager`（`start_teleop:=true` 才起键盘）。

### 3.2 F1 验收结果（真实输出，一次跑通）

```
=== [1/3] 重启 SITL（含 IMU 健康检查）===
[healthy] 第 1 次启动成功：IMU 有发布者
[sim] PX4 运行中 pid=98285
iterations: 2367
real_time_factor: 0.92995854477297035
=== [2/3] 起 flow_f1.launch.py ===
[f1] mavros 已连接（4s）
[f1] mission_manager 就绪（0s）
=== [3/3] 跑 verify_flow F1 ===
[FLOW F1] mission_manager 在线（0.1 s），当前 mode = IDLE
[FLOW F1] takeoff_altitude = 2.06 m（用时 9.2 s）
[FLOW F1] 遥控：前进 2 s / 右移 2 s / 后退 2 s / 左移 2 s / 上升 1 s / 偏航 1 s
[FLOW F1] 飞行结束 altitude = 0.98 m，飞行中最低 0.98 m
[FLOW F1] landed = yes（6.6 s），final_altitude = -0.00 m
[FLOW F1] takeoff_altitude = 2.06 m（判据 > 1.20）→ ok
[FLOW F1] final_altitude = -0.00 m（判据 < 0.20）→ ok
[FLOW F1] crash_free = 1.00 m（判据 == 1.00）→ ok
[FLOW F1] RESULT: PASS
[f1] verify_flow exit=0
```

### 3.3 判据对照

| F1 判据 | 要求 | 实测 | 结论 |
| --- | --- | --- | --- |
| 起飞高度 | > 1.2 m | 2.06 m | ✅ |
| 最终高度 | < 0.2 m | -0.00 m | ✅ |
| 全程无 crash | 高度不低于 0.5 m | 最低 0.98 m | ✅ |

### 3.4 观察到的现象（要在 M3 处理）

- 水平机动时高度从 2.06 m 掉到 0.98 m：OFFBOARD **速度**模式下 `vz=0` 只代表"零爬升率"，机体倾斜加速时会有掉高，没有高度保持。F3 的航点飞行会改用**位置**设定点（位置控制自带高度保持）。

### 3.5 踩坑与修复

**坑 J：旧暂存覆盖了新源码**
`sync.sh` 原来无条件 `cp -rf`，M1 阶段某次用 14:22 的旧暂存把 M0 末期写的 `takeoffTo` 源码覆盖成没有它的版本（只剩 `install/` 里还能看到符号）。→ 改为只覆盖"更新"的文件（`cp -u`），并加 `SYNC_FORCE=1` 显式强制。

**坑 K：脚本同步自己导致解析失败**
`sync.sh` 自己也在被同步目录里，就地 `cp` 覆盖等于"边执行边截断"，bash 直接报 `unexpected EOF while looking for matching '"'`，一个文件都没同步成。→ 改"先写临时名再 `mv`"（换目录项，正在执行的 bash 仍持旧 inode）。

**坑 L：`ament_target_dependencies(drone_control)` 找不到目标**
`drone_control` 把库和 `hover_test` 装在同一个 export set 里，展开成 `drone_control::hover_test`，CMake 报 `links to target drone_control::hover_test but the target was not found`。→ 显式 `target_link_libraries(... drone_control::drone_control_lib)`。

**坑 M：Python 包 `ros2 run` 报 No executable found**
少了 `setup.cfg` 里的 `[develop] script_dir=$base/lib/<pkg>` / `[install] install_scripts=$base/lib/<pkg>`，脚本被装到 `bin/` 而不是 `lib/<pkg>/`。→ 补 `setup.cfg` 后正常。

**坑 N：arm 被"没有 GCS 连接"拦住**
`WARN [commander] Arming denied` + `Preflight Fail: No connection to the GCS`。源码 `rcAndDataLinkCheck.cpp:75-97` 里 `gcs_connection_required = (_param_nav_dll_act.get() > 0)`。→ `NAV_DLL_ACT=0`（用 `/mavros/param/set`，类型是 `ParamSetV2`，字段是 `rcl_interfaces/ParameterValue`）。该值已持久化进 `parameters.bson`，重装仿真环境后要重设。

---

## 4. 当前状态与下一步

| 里程碑 | 状态 |
| --- | --- |
| M0 环境打通 | ✅ 完成（悬停 30 s PASS） |
| M1 视觉检测 | ✅ 完成（id/字典/尺寸标定完成，精度受 EKF 限制已说明） |
| M2 飞控封装 + 键盘 + F1 | ✅ 完成（F1 PASS） |
| M3 视觉伺服降落 + F2 | ⏳ 下一步 |
| M4 航点导航 + F3 / M5 全自主 + F4 / M6 异常处理 + F5 | ⏳ 未开始 |

代码提交：`712d516`(M0) → `7e62583`(需求书修正) → `8b52c1e`(M1) → M2 见 `git log`。
