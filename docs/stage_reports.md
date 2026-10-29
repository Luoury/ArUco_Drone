# 阶段成果报告（M0 / M1 / M2 / M3 / M4）

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

## 4. M3 — 视觉伺服降落 + F2 流程

### 4.1 交付物

- `drone_aruco/landing_controller`：§5.3 状态机 `IDLE → SEARCH → ALIGN → DESCEND → FINAL → TOUCHDOWN`，以 50 Hz 发 `/mavros/setpoint_raw/local`；服务 `/aruco/landing/start|stop`，10 Hz 发布 `/aruco/landing/state`。
- `aruco_detector` 增加 EMA 位置平滑（`smooth_alpha: 0.3`），抑制逐帧 PnP 噪声。
- `mission_manager` 增加 `ARUCO_LAND` 委托：§5.5 的"先置 HOVER、等 0.5 s、再交给新控制器"，交接期间**本节点一个字都不发**，避免双发布者抢话题。
- `flow_f2.launch.py`（`sim_bringup` + `aruco_detect` + `landing_controller` + `mission_manager`）、`drone_tools/verify_flow --flow F2`、`scripts/dev_run_f2.sh`。
- MANUAL 模式从"速度设定点"改成"速度积分成位置目标"：位置环自带定高，顺手修掉了 M2 遗留的掉高问题。

### 4.2 F2 验收结果（真实输出）

```
[healthy] 第 1 次启动成功：IMU 有发布者
[sim] PX4 运行中 pid=137214   iterations: 2211   real_time_factor: 0.99
[f2] mavros 已连接（4s）
[f2] 就绪：mission_manager 就绪（0s）
[f2] 就绪：降落控制器就绪（0s）
[f2] 就绪：发布 /aruco/detection（0s）
[FLOW F2] takeoff_altitude = 1.71 m（用时 9.6 s）
[FLOW F2] 遥控：前飞 2.0 s（离开码上方，模拟人工操作）
[FLOW F2] 接管前水平误差 = 0.81 m
[FLOW F2] 请求模式 ARUCO_LAND（等价于按下 '2'）
[FLOW F2] marker_detected = yes（收到 478 帧检测，最后一帧水平误差 0.01 m）
[FLOW F2] landing_states = IDLE → ALIGN → DESCEND → FINAL
[FLOW F2] landed = yes（18.4 s），final_altitude = -0.01 m，final_lateral = 0.01 m
[FLOW F2] marker_detected = yes（判据 == yes）→ ok
[FLOW F2] final_lateral = 0.01 m（判据 <= 0.25）→ ok
[FLOW F2] final_altitude = -0.01 m（判据 <= 0.25）→ ok
[FLOW F2] elapsed = 31.06 s（判据 <= 150.00）→ ok
[FLOW F2] RESULT: PASS
```

### 4.3 判据对照

| F2 判据 | 要求 | 实测 | 结论 |
| --- | --- | --- | --- |
| 检测到码 | 必须 | 478 帧 | ✅ |
| 最终水平误差 | ≤ 0.25 m | 0.01 m | ✅ |
| 最终高度 | ≤ 0.25 m | -0.01 m | ✅ |
| 总耗时 | ≤ 150 s | 31.1 s | ✅ |

回归：F1 在改造 MANUAL 后重跑仍然 PASS，且飞行中最低高度 1.84 m（改造前会掉到 0.98 m）。

### 4.4 三个决定性的坑

**坑 O：ENU 双重转换回归（最惊险）**
`publishPositionSetpoint` 里又出现了 `msg.position.z = -enu.z`。mavros 收到的就是 ENU，再转一次等于命令"降到地下 1.4 m"，机体 0.3 s 内从 1.4 m 砸到 0.1 m。用 `scripts/trace_alt.py` 同时录 mavros 估计高度与 gz 真值，才确认是"真掉"而不是"读数跳变"。→ 四个 publish 函数一律直接发 ENU。

**坑 P：相机光学系→机体系的 x 轴符号错**
`kOptToFlu` 第一行写成 `{0, 1, 0}`，导致检测出的码位置在无人机**另一侧**（实测真实偏移 `(-1.09, -0.91)` vs 估计 `(+1.08, -0.87)`），降落控制器于是越飞越远。M1 是悬停在码正上方标定的，x/y 偏移都接近 0，**符号错了也完全看不出来**。→ 改成 `{0,-1,0, -1,0,0, 0,0,-1}`（行列式 +1），并新增 `dev_run_det_probe.sh` 专门做"飞离码上空"的偏移校验，修正后估计偏移 `(-1.02, -0.90)` vs 真实 `(-1.01, -0.92)`，误差约 2 cm。

**坑 Q：`align_abort_tol` 太小导致 ALIGN ↔ SEARCH 死循环**
0.60 m 的"发散保护"阈值比"飞离码上空"后的正常误差（1~2 m）还小，一进 ALIGN 就被判发散。→ 提到 2.50 m，明确它只兜解算发散；远距离接近交给 ALIGN 的比例速度（`kp_xy * err`，限幅 `max_vel_xy`）。

另外还踩到：`stop_ros.sh` 的模式串出现在调用命令行里会把自己 kill 掉（"无输出、exit 0"）；Python 在 Windows 上写 `.sh` 会带 CRLF（`$'
': command not found`）；`ros2 topic pub --once` 刚建发布者就发会丢（要先空转完成 DDS 匹配）。

---

## 5. M4 — 航点导航 + F3 流程

### 5.1 交付物

| 文件 | 作用 |
| --- | --- |
| `drone_nav/src/waypoint_nav.cpp` | 航点导航节点：`/waypoint/start`、`/waypoint/stop`（Trigger），10 Hz 发布 `/waypoint/state` 与 `/waypoint/progress` |
| `drone_bringup/config/waypoints.yaml` | 航点真值（`[3,0,2] → [3,3,2] → [0,0,2]`），每个点带注释说明设计意图 |
| `drone_bringup/launch/flow_f3.launch.py` | `sim_bringup` + `waypoint_nav` + `mission_manager(flow=F3)` |
| `drone_mission`（改） | 新增 `WAYPOINT` 模式委托：§5.5 先悬停 0.5 s 再 `/waypoint/start`，期间本节点**不发任何 setpoint** |
| `drone_tools/drone_tools/verify_flow.py`（改） | 新增 `run_f3()`：一条模式指令跑完全程，逐点校验稳态误差 |
| `scripts/dev_run_f3.sh` | 一键：起 SITL（含 IMU 健康检查）→ 起 F3 launch → 跑验收 → 收尾 |

航点状态机：`IDLE → TAKEOFF → GOTO ⇄ HOLD → DONE`。每个航点必须"误差 ≤ `settle_tol`(0.30 m) 且**连续稳定 2 s**"才算到位，稳态误差取稳定计时结束那一拍的实测值。

### 5.2 F3 验收结果（真实输出）

```
[FLOW F3] 开始：F3 纯自主航点（起飞 → 3 个航点 → 回码上方悬停 → 降落）
[FLOW F3] mission_manager 在线（0.0 s），当前 mode = IDLE
[FLOW F3] 请求模式 WAYPOINT（此后不再发任何遥控指令）
[FLOW F3] 航点阶段 = IDLE → GOTO → HOLD → GOTO → HOLD → GOTO → HOLD → DONE
[FLOW F3] 航点 1 稳态误差 = 0.158 m
[FLOW F3] 航点 2 稳态误差 = 0.075 m
[FLOW F3] 航点 3 稳态误差 = 0.102 m
[FLOW F3] landed = yes（7.8 s），final_altitude = -0.04 m，elapsed = 30.6 s
[FLOW F3] waypoint_1_error = 0.158 m（判据 <= 0.35）→ ok
[FLOW F3] waypoint_2_error = 0.075 m（判据 <= 0.35）→ ok
[FLOW F3] waypoint_3_error = 0.102 m（判据 <= 0.35）→ ok
[FLOW F3] elapsed = 30.60 s（判据 <= 180.00）→ ok
[FLOW F3] final_altitude = -0.04 m（判据 <= 0.20）→ ok
[FLOW F3] RESULT: PASS
```

节点侧日志（对应"到位 → 稳定计时 → 记录"三段）：

```
[WAYPOINT] IDLE -> TAKEOFF / TAKEOFF -> GOTO
[WAYPOINT] 航点 1/3 到位（误差 0.294 m），开始稳定计时
[WAYPOINT] 航点 1/3 稳态误差 0.158 m（稳定 2.0 s）
[WAYPOINT] 航点 2/3 稳态误差 0.075 m（稳定 2.0 s）
[WAYPOINT] 航点 3/3 稳态误差 0.102 m（稳定 2.0 s）
[WAYPOINT] 全部 3 个航点完成
```

### 5.3 判据对照（需求书 §5.6 F3）

| 判据 | 要求 | 实测 | 结论 |
| --- | --- | --- | --- |
| 每个航点稳态误差 | ≤ 0.35 m | 0.158 / 0.075 / 0.102 m | 通过 |
| 全程时间 | ≤ 180 s | 30.6 s | 通过 |
| 零人工输入 | 只发一条模式指令 | 只发了 `WAYPOINT` | 通过 |
| 落地 | 上锁 | `final_altitude = -0.04 m` | 通过 |

### 5.4 两个坑

**坑 R：稳态误差抓不到，"全部航点完成"却判 FAIL**
第一次跑 F3，状态机跑得漂亮（`GOTO→HOLD→DONE`、29.4 s 落地），但三个航点全报"未收到稳态数据"。原因：`waypoint_nav` 的 `dwell_` 在"稳定 2 s 达标"那一拍**同时**被清零并推进下一个航点，10 Hz 的 progress 里永远看不到 `dwell >= 2.0` 的样本，验收脚本自然一次也没采到。
→ 修法不是放宽判据，而是让 `waypoint_nav` 在完成那一拍把 `settled_index_ / settled_error_` 记进 `/waypoint/progress` 的 `[4] / [5]` 两个新元素，验收脚本按"完成的航点号 → 稳态误差"取值。**这是典型的"测量口而不是被测系统"的问题**：如果当时把判据改成"取 dwell 最大时的误差"，就会把一个不准确的测量当成结论。

**坑 S：字符串替换 `if (...) {` 时把开括号吃掉**
给 `mission_manager` 加 `WAYPOINT` 分支时，用脚本替换原来的 `if (!callLanding(...)) {`，替换串漏了行尾的 `{`，编译报 `expected unqualified-id before 'if'` + `expected ';' after class definition`（报错行号全是下游，根因在 299 行）。→ 教训：**改带括号的行，替换串必须把 `{` 写全**；另外这类"括号错位"的报错会连累一堆 `not declared in this scope`，别被下游错误带跑偏。

---

## 6. M5 — 全自主链路（AUTO）+ F4 流程

### 6.1 交付物

| 文件 | 说明 |
| --- | --- |
| `drone_mission/src/mission_manager.cpp` | 新增 `AUTO` 两阶段委托：`WAYPOINT`（阶段 1）→ `LAND`（阶段 2），全程本节点不发任何 setpoint；`auto_timeout` 120 s 兜底 |
| `drone_bringup/launch/flow_f4.launch.py` | 一条命令拉起 sim_bringup + aruco_detect + waypoint_nav + landing_controller + mission_manager(F4) |
| `drone_tools/drone_tools/verify_flow.py` | `run_f4()`：只发一条 `AUTO` 模式请求，其余全靠系统自己跑；记录 `/mission/status.state` 变化序列 |
| `drone_bringup/config/flows.yaml` | `auto_timeout: 120.0`、`f4_lateral_max: 0.25`、`f4_altitude_max: 0.25`、`f4_flow_timeout: 180.0` |
| `scripts/dev_run_f4.sh` | 起 SITL → launch F4 → 等四个节点就绪 → 跑验收 → 收尾 |
| `scripts/m5_probe.py` | 临时探针：1 Hz 记录 `/mavros/setpoint_raw/local` 的 `type_mask / position.z / velocity.z` + 实际高度 |

AUTO 的调度顺序（严格按 `mission_manager` 的 `monitorAuto()`）：

```
AUTO 请求 → 未解锁则 AUTO.TAKEOFF 起 1.5 m → 悬停 0.5 s（§5.5 模式切换纪律）
        → /waypoint/start → 阶段 WAYPOINT（本节点沉默，waypoint_nav 独占设定点）
        → /waypoint/state == DONE → /waypoint/stop → 悬停 0.5 s
        → /aruco/landing/start → 阶段 LAND（本节点沉默，landing_controller 独占设定点）
        → /aruco/landing/state == DONE → mode = IDLE、state = AUTO_DONE
```

### 6.2 F4 验收结果（真实输出）

```
=== [3/3] 跑 verify_flow F4 ===
[FLOW F4] mission_manager 在线（0.1 s），当前 mode = IDLE
[FLOW F4] 请求模式 AUTO（这是全程唯一一条指令）
[FLOW F4] 航点阶段 → GOTO → HOLD → GOTO → HOLD → GOTO → HOLD → DONE → IDLE
[FLOW F4] 航点 1 稳态误差 = 0.190 m
[FLOW F4] 航点 2 稳态误差 = 0.127 m
[FLOW F4] 航点 3 稳态误差 = 0.116 m
[FLOW F4] landing 阶段 → ALIGN → DESCEND → FINAL → SEARCH → ALIGN → DESCEND → FINAL → DONE
[FLOW F4] 状态序列 = IDLE → AUTO_WAYPOINT → AUTO_LAND → AUTO_DONE
[FLOW F4] marker_detected = yes（收到 381 帧检测，最后一帧水平误差 0.04 m）
[FLOW F4] landed = yes（42.4 s），final_altitude = -0.03 m，elapsed = 42.5 s

[FLOW F4] marker_detected = yes（判据 == yes）→ ok
[FLOW F4] final_lateral = 0.04 m（判据 <= 0.25）→ ok
[FLOW F4] final_altitude = -0.03 m（判据 <= 0.25）→ ok
[FLOW F4] elapsed = 42.50 s（判据 <= 180.00）→ ok
[FLOW F4] RESULT: PASS
```

修复后 `m5_probe.py` 的同一段（landing 阶段，`4039` = 速度掩码，`4088` = 位置掩码）：

```
 30.3   0.880   mask=4039   vz=-0.300      ← DESCEND，实际下降
 31.3   0.588   mask=4039   vz=-0.300
 32.3   0.395   mask=4039   vz=-0.120      ← FINAL
 33.3   0.290   mask=4039   vz=-0.120
 34.4   0.184   mask=4039   vz=-0.120
 35.4   0.158   mask=4039   vz= 0.000      ← 贴地，TOUCHDOWN
 38.4   0.123   mask=4088   pz= 0.123      ← 交由 mission_manager 做最后的位置收尾
 39.4  -0.014   mask=4088   pz=-0.015
```

设定点不再来回打摆：位置设定点序列（`[0,0,2.0]`，最后一个航点）在交接窗口后彻底消失。

### 6.3 判据对照（需求书 §5.6 F4）

| 指标 | 判据 | 实测 | 结论 |
| --- | --- | --- | --- |
| 全程人工输入 | 零 | 1 条 `AUTO` 模式请求（由验收脚本发，等价于按 `4`） | ✅ |
| 最终水平误差 | ≤0.25 m | 0.04 m | ✅ |
| 最终高度 | ≤0.25 m | -0.03 m | ✅ |
| 全程耗时 | ≤180 s | 42.5 s | ✅ |
| 模式切换日志 | 清晰 | `AUTO_WAYPOINT → AUTO_LAND → AUTO_DONE` 三段齐全 | ✅ |

### 6.4 决定性的坑

**坑 T：两个节点同时抢 `/mavros/setpoint_raw/local`（本阶段最贵的一次排查）**
F4 前三次都以同样的方式失败：链路、检测、状态机全对，就是最后一段降不下来 —— DESCEND 从 2.5 m 到 0.5 m 花了 **108 s**（指令 -0.30 m/s，实际 0.014 m/s），最后顶到 `auto_timeout` 120 s 被强制 LAND，`state` 停在 `LANDED` 而不是 `AUTO_DONE`。

一开始怀疑是检测延迟、PnP 噪声、速度限幅，都不是。加了 `m5_probe.py` 直接听设定点，1 秒就看见真相：

```
 61.6   1.932   mask=4039  vz=-0.300     ← landing_controller 在降
 62.6   1.894   mask=4088  pz= 2.000     ← waypoint_nav 还在把它拉回 2 m
```

`4088 = 0b0000111111111000`（位置掩码）与 `4039 = 0b0000111111000111`（速度掩码）以 8 s 周期交替，高度就在 0.8~1.9 m 之间画正弦 —— 典型的两个发布者互相打架的极限环。

根因：`waypoint_nav` 在 `kStateDone` 分支里无条件 `publishPositionSetpoint(hold_)`，而 `hold_` 正是最后一个航点 `[0, 0, 2.0]`；mission_manager 这时已经把控制权交给了 `landing_controller`（发速度设定点）。F3 里完全看不出来，因为那时 mission_manager 进 HOVER 发的也是同一个点，两个发布者"意见一致"。

修法（两处，缺一不可）：
1. `waypoint_nav` 增加 `handover_grace`(0.5 s)：DONE 之后只保留 0.5 s 交接窗口，然后打日志「已停止发布设定点，控制权归 mission_manager」并彻底闭嘴；
2. `mission_manager` 的 `monitorAuto()` 从航点阶段切到降落阶段前**显式** `callTrigger("/waypoint/stop")`，不依赖宽限期。

教训：**OFFBOARD 阶段"谁在发设定点"必须是全局唯一且可验证的**。任何"我跑完了但继续维持位置"的便利写法，都会在下一次控制权交接时变成隐形冲突。诊断手段首选"直接听话题"（本项目的 `m5_probe.py`），而不是继续加日志猜。

**坑 U：`sync.sh` 用 mtime 判断文件新旧，把改过的脚本静默跳过**
`/mnt/c` 是 DrvFs，mtime 有秒级/时区误差；`cp -u` 的 `-nt` 判断因此把刚改过的 `dev_run_f4.sh` 判成"更旧"而跳过同步，结果 F4 白跑了一轮（探针根本没启动、`/tmp/f4_probe.log` 不存在，日志里也看不出异常）。→ 改成 `cmp -s` **比内容**，`src/` 用 `rsync -a --checksum`；只有 `SYNC_FORCE=1` 才无条件覆盖。"同步成功"这句话在跨文件系统时必须由内容比较来背书。

---

## 7. M6 — 异常处理 + F5 流程

### 7.1 交付物

| 文件 | 说明 |
| --- | --- |
| `drone_aruco/src/landing_controller.cpp` | 丢码后**先抬升再搜索**（`relost_climb_gain` 2.0 m，上限 `max_search_alt` 4.0 m）；新增 `final_commit`：FINAL 阶段丢码不再回 SEARCH，按最后对准结果继续降到上锁 |
| `drone_bringup/launch/flow_f5.launch.py` | F5 入口（sim_bringup + 检测 + 航点 + 降落 + mission_manager(F5)） |
| `drone_tools/drone_tools/verify_flow.py` | `run_f4(occlude=True)`：AUTO 全链路 + 在 Gazebo 侧真实遮挡 3 s；新增遮挡分析、回升量、单次恢复判据 |
| `drone_bringup/config/flows.yaml` | `f5_trigger_alt 2.4`、`f5_occlusion 3.0`、`f5_rise_min 1.0`、`f5_rise_window 12.0`、`f5_flow_timeout 240.0` 等 |
| `scripts/dev_run_f5.sh` / `scripts/run_flow.sh` | F5 一键跑 / 五套流程统一入口（`run_flow.sh F1..F5`） |
| `docs/flows.md` | 五套流程的入口、判据、实测结果与"判据怎么算" |

### 7.2 F5 验收结果（真实输出）

```
[FLOW F5] 开始：F5 异常处理（AUTO 全自动 + 下降途中遮挡相机 3 s）
[FLOW F5] 请求模式 AUTO（此后不再发任何指令，遮挡由本脚本从 Gazebo 侧施加）
[FLOW F5] 航点 1/2/3 稳态误差 = 0.153 / 0.097 / 0.076 m
[FLOW F5] 下降段已到位：高度 2.00 m，正准备施加遮挡
[FLOW F5] 遮挡板已生成（z = 1.50 m），保持 3.0 s
[FLOW F5] 诊断：occluder 位姿 = Model: [58] - Name: occluder - Pose [-0.080 0.013 1.497] [0 0 0]
[FLOW F5] 遮挡分析：7 帧 / 最长无检测间隔 3.00 s（遮挡 3.0 s）
[FLOW F5] 遮挡板已撤除（ok）
[FLOW F5] 遮挡后最高 3.72 m，遮挡期间最低 1.85 m，回升 = 1.87 m
[FLOW F5] 状态序列 = IDLE → AUTO_WAYPOINT → AUTO_LAND → AUTO_DONE
[FLOW F5] 降落阶段 = IDLE → ALIGN → DESCEND → SEARCH → ALIGN → DESCEND → FINAL → DONE
[FLOW F5] marker_detected = yes（全程 457 帧）
[FLOW F5] landed = yes（6.2 s），final_altitude = 0.02 m，elapsed = 47.0 s

[FLOW F5] marker_occluded = yes（判据 == yes）→ ok
[FLOW F5] altitude_rise = 1.87 m（判据 >= 1.00）→ ok
[FLOW F5] no_crash = yes（判据 == yes）→ ok
[FLOW F5] final_lateral = 0.01 m（判据 <= 0.25）→ ok
[FLOW F5] final_altitude = 0.02 m（判据 <= 0.25）→ ok
[FLOW F5] elapsed = 46.97 s（判据 <= 240.00）→ ok
[FLOW F5] RESULT: PASS
```

### 7.3 判据对照（需求书 §5.6 F5）

| 指标 | 判据 | 实测 | 结论 |
| --- | --- | --- | --- |
| 是否真的被遮挡 | yes | 最长连续无检测 3.00 s（遮挡 3.0 s） | ✅ |
| 遮挡后回升 | ≥1.0 m | 1.85 → 3.72 m，回升 1.87 m | ✅ |
| 不坠机 | yes | 无 crash，状态机单次恢复 | ✅ |
| 恢复后仍降落 | 水平 ≤0.25 / 高度 ≤0.25 | 0.01 m / 0.02 m | ✅ |
| 全程耗时 | ≤240 s | 47.0 s | ✅ |

### 7.4 决定性的坑

**坑 V：遮挡板"创建成功"但其实没有创建 —— SDF 引号被截断**
`gz service -s /world/aruco/create -r "sdf: '<sdf version="1.6">...', name: 'occluder'"`：外层已经是双引号，SDF 里的属性又用双引号，gz 的文本解析在第一个内层双引号处截断，实体创建**静默失败但返回 true**。表现是"遮挡板已生成"、检测帧数却和不遮挡一模一样（60 帧）。

三条查证手段缺一不可：① `gz model -m occluder -p` 看实体是否真存在（返回 `Name: occluder - Pose [...]` 才算成功）；② 遮挡期间统计帧数；③ `gz model -l` 看世界模型列表。修法：SDF 内部属性一律单引号。

**坑 W：FINAL 阶段丢码回 SEARCH 会造成死循环**
0.15 m 处 0.5 m 的码已超出相机 1.74 rad 视场，检测必然丢；原来的"丢码 → 悬停 → 回 SEARCH 抬到 2.5 m"逻辑在这里变成死循环（F5 第二轮 60 s 里刷了 5 轮 `FINAL → SEARCH → ALIGN → DESCEND → FINAL`）。

修法：`final_commit`（默认 true）——FINAL 阶段丢码视为"已经对准、继续降"，按 `-final_speed` 降到上锁；同时给"丢码后重新搜索"加上"先抬升 `relost_climb_gain`"（F5 要的回升量就来自这里）。

**坑 X：验收判据本身把成功判成失败（两次）**
1. "遮挡期间检测帧数 == 0" 太严 —— 板子加载有延迟，窗口开头必然有几帧。改成"最长连续无检测间隔 ≥ 60% 遮挡时长"。
2. "`armed && altitude < 0.5` 即 crash" —— 正常降落必然穿越 0.5 m，且状态切换消息比高度晚几十毫秒，于是每一次成功降落都被记为撞地。改成"低于 0.5 m 持续 ≥1 s 且不在 FINAL/TOUCHDOWN/DONE"。

**这两次的共同教训**：验收指标必须表达"物理事实"，而不是"实现细节的副产品"。指标错了会把好结果判死，比没指标更危险。

**坑 Y：YAML 缩进错 2 个空格，rcl 直接崩进程**
用脚本往 `landing.yaml` 插参数时缩进写错，rcl 报 `Couldn't parse params file: ... Cannot have a value before ros__parameters at line 27, at ./src/parse.c:787`，节点 `exit code -6`。改完必须 `python3 -c "import yaml; yaml.safe_load(open(...))"` 验一遍。

---

## 8. 当前状态与下一步

| 里程碑 | 状态 |
| --- | --- |
| M0 环境打通 | ✅ 完成（悬停 30 s PASS） |
| M1 视觉检测 | ✅ 完成（id/字典/尺寸标定完成，精度受 EKF 限制已说明） |
| M2 飞控封装 + 键盘 + F1 | ✅ 完成（F1 PASS） |
| M3 视觉伺服降落 + F2 | ✅ 完成（F2 PASS，落点水平误差 0.01 m） |
| M4 航点导航 + F3 | ✅ 完成（F3 PASS，三点稳态误差 0.158/0.075/0.102 m） |
| M5 全自主 + F4 | ✅ 完成（F4 PASS，42.5 s，落点水平误差 0.04 m） |
| M6 异常处理 + F5 | ✅ 完成（F5 PASS，遮挡 3.00 s，回升 1.87 m，47.0 s） |

代码提交：`712d516`(M0) → `7e62583`(需求书修正) → `8b52c1e`(M1) → `7bb6e6b`(M2) → `0223ee3`(M3) → `a329c60`(M4) → `c4c1190`(M4 收尾) → M5 见 `git log`。

---

## 9. 全流程回归（M6 收尾，一次连续跑）

五套流程在同一天、同一环境下连续跑一遍，确认 `final_commit`（FINAL 阶段丢码不再回搜）等
M6 改动没有破坏前面四套：

| 流程 | 关键实测 | 结果 |
| --- | --- | --- |
| F1 纯手动 | 起飞 1.97 m，降落 7.7 s，最终 0.02 m | **PASS** |
| F2 视觉降落 | 起飞 2.02 m，落点 `final_lateral = 0.00 m`、`final_altitude = 0.10 m`、31.7 s | **PASS** |
| F3 航点 | 0.165 / 0.101 / 0.103 m，29.6 s，最终 0.01 m | **PASS** |
| F4 全自主 | 0.155 / 0.115 / 0.108 m，37.4 s，最终 0.02 m | **PASS** |
| F5 异常处理 | 遮挡 3.00 s，回升 1.87 m，47.0 s，落点 0.01 m | **PASS** |

复现方式：`bash scripts/run_flow.sh F1`（依次 F1~F5）。

## 10. 交付物总览

| 类别 | 内容 |
| --- | --- |
| 包 | `drone_interfaces` / `drone_bringup` / `drone_control` / `drone_aruco` / `drone_nav` / `drone_mission` / `drone_teleop` / `drone_tools` |
| 流程 | `flow_f1` ~ `flow_f5` 五个 launch，各有自动验收脚本 |
| 文档 | `docs/README.md`（上手）、`docs/flows.md`（五套流程）、`docs/troubleshooting.md`（33 条实测坑）、本文件（阶段成果） |
| 脚本 | `sim.sh` / `start_until_healthy.sh` / `stop_ros.sh` / `run_flow.sh` / `fix_gz_plugins.sh` / `fix_gz_model.sh` 等 |
| 证据图 | `docs/images/m1_debug_image.png`（检测叠加角点框与坐标轴） |

---

## 11. 交付自检（M6 收尾后做的一次完整核对）

| 检查项 | 结果 |
| --- | --- |
| `bash scripts/run_flow.sh F1` | 统一入口可用：`=== run_flow: F1 退出码 0（0 = PASS）===`，`[FLOW F1] RESULT: PASS` |
| `bash scripts/check_env.sh` | 修好 `set -u` 崩溃后完整跑通，输出与实际一致（见下） |
| 文档交叉引用 | 35 处路径引用与 md 相对链接全部可解析 |
| 仓库残留 | 删除 `test/`（早期泄漏实验的 4 个临时世界），`.cache/ build/ install/ log/ logs/` 均在 `.gitignore` |

`check_env.sh` 的真实输出（节选）：

```
/dev/dri: 不存在（走软件渲染，必须用 Ogre1）
ROS_DISTRO=humble  rclcpp=/opt/ros/humble
  mavros / mavros_msgs / mavros_extras / cv_bridge: 已安装
  gz-transport13: 13.5.0      gz-msgs10: 10.3.2
  警告: 系统里还有 Fortress(ignition) 版的传输库，别用 ros_gz_bridge（ABI 不匹配）
  PX4: v1.17.0-alpha1-338-gdf42ef84f1（main）
  PX4_GZ_SIM_RENDER_ENGINE 未设置（脚本 scripts/sim.sh 会自动设成 ogre）
```

自检本身也踩了一个坑：脚本第一版用 `set -uo pipefail`，`source /opt/ros/humble/setup.bash`
立刻报 `AMENT_TRACE_SETUP_FILES: unbound variable` 退出，什么都检查不到。改成只 `set -o pipefail`
并加注释说明原因 —— **这个环境里所有"source ROS 环境"的脚本都不能开 `set -u`**。

---

## 12. 附加交付：演示视频录制管线

M6 之后按需求追加：用户要"录一段视频"用于展示。直接录 WSLg 窗口不可靠（远程桌面合成通道
会让 3D 视口 stalled，录出来是冻住的画面），因此改为**离屏录制**：把相机放进仿真里，
画面走 ROS 话题，再由 Python 编码成 mp4。整条链路不依赖任何窗口。

### 12.1 实现

| 组件 | 文件 | 作用 |
|---|---|---|
| 俯视机位 | `drone_bringup/models/overhead_cam/model.sdf` | 世界里一台静态相机：`pose 0 0 6.0 / pitch 90°`、`fov 1.05`、`960x540@10` |
| 挂载 | `scripts/spawn_overhead_cam.sh` | 向运行中的世界 `create` 该模型，幂等 |
| 桥接 | `drone_bringup/src/gz_sensor_bridge.cpp` 第二路图像通道 | gz 图像 → `/camera/overhead/image_raw`（参数 `enable_overhead` 等） |
| 录制 | `drone_tools/drone_tools/record_video.py` | 订阅图像 + `/mission/status` + `/aruco/debug_image`，叠加 HUD 与小窗，写 mp4 |
| 一条命令 | `scripts/record_video.sh` | 起 SITL → 挂机位 → 起流程（`start_overhead:=true`）→ 录制 → 跑验收 → 收尾 |

### 12.2 实测结果

`bash scripts/record_video.sh F2 70 auto`：

```
[overhead] 就位：/world/aruco/model/overhead_cam/link/link/sensor/camera/image 已有发布者
相机帧率 26.8 Hz（134 帧 / 最近 5.0 s）        俯视相机帧率 8.8 Hz（44 帧 / 最近 5.0 s）
录制结束：写入 1916 帧 / 85.2 s（22.5 fps），收到 698 帧、过快丢弃 0 帧
[FLOW F2] RESULT: PASS   final_lateral = 0.03 m  final_altitude = 0.04 m  elapsed = 40.28 s
产物：logs/videos/F2_125848.mp4（6.5 MB，960×540@25）
```

### 12.3 四个必须记住的坑

1. **俯视相机不能配太大**：`1280×720@30` 会把机载主相机拖到 ~5 Hz、检测 4.6 Hz，F2 因此 FAIL
   （ALIGN 反复丢码）。`960×540@10` 下主相机 26~28 Hz、俯视 8.6~9.6 Hz，F2 恢复 PASS。
   软件渲染下两台相机抢同一个渲染器 —— 加相机就要降配。
2. **`gz service create` 的字符串值必须双引号**：单引号会让请求串被截断，服务返回 `data: true`
   但实体没建（判定要看 `gz topic -i` 有没有发布者，不能看 `gz model -p`）。
3. **`-p duration:=85` 是 INTEGER**，节点若按 double 声明会抛
   `InvalidParameterTypeException`，改用 `ParameterDescriptor(dynamic_typing=True)`。
4. **`pkill -f` 的模式串会匹配到调用它的 shell 自己**，脚本里必须用更具体的
   `lib/drone_tools/record_video`，否则表现为"命令无任何输出、退出码 0"。

### 12.4 附带结论：MobaXterm 的 X server 可用

用户提出"能不能用 X server 替代 WSLg"。实测：`XWin_MobaX.exe` 在 127.0.0.1:6000 监听，
`xdpyinfo` 显示 X.Org 1.20.13 + GLX 存在，`DISPLAY=127.0.0.1:0 gz sim -g --render-engine ogre`
能正常渲染出无人机与码板（无 OpenGL context 报错）。X11 的屏幕尺寸是 1024×768，比 WSLg 小。
用法见 `manual_test.md` §6。

## 13. 附加交付：多工位巡回演示（TOUR）

**需求**（m03708）：视频里要同时看到无人机和码、机位尽量斜俯视；
多航点演示要"从基站起飞 → 到每个工位降落 → 再起飞 → 绕一圈回基站"。

**实现**

| 交付物 | 说明 |
|---|---|
| `mission_manager` 的 `TOUR` 模式 | GOTO（本节点发位置设定点）→ LAND（交给 `landing_controller`）→ 落地上锁 → 原地起飞 → 下一站；最后一站落地上锁后 `TOUR_DONE` |
| `config/tour.yaml` | 四个工位坐标 + 各阶段超时（代码里无魔法数字） |
| `launch/flow_tour.launch.py` | 一条 launch 起全链路，`marker_id:=-1` 表示"取画面中心最近的码"（多码同屏） |
| `models/demo_cam` | 斜俯视演示机位：`(-2.2, -2.2, 3.8)`、俯角 37.7°、偏航 45°、FOV 1.3 |
| `models/arucotag_{1,2,3}` | 三个工位码（DICT_4X4_50 的 1/2/3），与基站码 0 同尺寸 |
| `scripts/spawn_course.sh` | 向运行中的世界摆三个工位码 + 演示机位（幂等，会回读世界确认） |
| `scripts/demo_tour.sh` | 一条命令：清场 → 内存预检 → 起 SITL → 摆场景 → 起链路 → 录制 → 发 TOUR → 等完成 → 收尾 |
| `scripts/tour_watch.py` | 盯 `/mission/status` 打印状态迁移，`TOUR_DONE` 退出 0 |

**实测（两次完整跑）**

```
TOUR 1/4 到站（误差 0.12 m），交给视觉降落
TOUR 1/4 已落到工位码上（17.4 s）
TOUR 2/4 到站（误差 0.12 m）… 已落到工位码上（17.2 s）
TOUR 3/4 到站（误差 0.10 m）… 已落到工位码上（15.2 s）
TOUR 4/4 到站（误差 0.09 m）… 已落到工位码上（17.3 s）
TOUR 全部 4 个工位完成（总耗时 110.5 s），已回到基站
```

录像：`logs/videos/tour_140756.mp4`（9.2 MB, 95.5 s）、
`logs/videos/tour_141335.mp4`（14 MB, 134.4 s, 960×540 @ 25 fps，含 HUD 与机载小窗）。

**这一轮踩到的坑**

| 坑 | 现象 | 根因与修法 |
|---|---|---|
| Z1 工位数不是 3 的倍数 | `tour_stations 长度 8 不是 3 的倍数` | 代码按 `(x,y,z)` 三元组取下标（z 目前用 `tour_approach_alt`），配置必须写 12 个值 |
| Z2 模式名不在白名单 | `未知模式 TOUR（可选 MANUAL/…/AUTO）` | 只加了处理分支、忘了加模式校验白名单，补 `&& mode != "TOUR"` |
| Z3 上一站的 `DONE` 污染下一站 | 第 2 站"已落到工位码上（0.0 s）"，随后第 3 站启动降落超时 → `TOUR_ABORT` | `landing_state_` 是 latched 的最后一帧，进入 LAND 阶段时先 `clear()`，并要求 `spent > 1.0 s` 才承认 `DONE` |
| Z4 HUD 中文变问号 | 视频左上角 `ArUco ?????????` | `cv2.putText` 只认 ASCII，标题统一用英文，代码里也做了非 ASCII 过滤兜底 |
| Z5 软件渲染下视频比真实更短 | 134 s 的飞行录成 95 s（快放） | 录制器按"落后就复制当前帧"保持时基，原来每次只补 3 帧，改成最多补 8 帧 |
