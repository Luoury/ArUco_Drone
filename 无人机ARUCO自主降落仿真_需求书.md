# 无人机 ArUco 自主降落 / 自主导航仿真 —— 交接需求书

> **本文件的用法**：整份文档直接交给另一个 AI 会话（或人），它应当能独立完成工程。
> 文档自包含：环境事实、架构、接口契约、状态机、验收标准、禁止事项、里程碑。
> 环境实测日期：2026-10-28。若与实测不符，以实测为准并记录偏差。

---

## 0. 粘贴给对方的原话

把下面这段连同本文件路径一起发过去即可（路径按实际修改）：

```text
你是一名 ROS 2 机器人工程师。请在一个已配置好的 WSL2 环境里，从零实现一个
"无人机仿真 + ArUco 自主降落 + 自主导航" 的 ROS 2 工程。

请先完整阅读需求书：
  /home/luosury/ArUco_Drone/无人机ARUCO自主降落仿真_需求书.md

硬性要求（细节全在需求书里，务必先读完再动手）：
1. ROS 2 Humble，C++17 为主，Python 只用于键盘遥控、工具脚本、验收脚本。
2. 仿真栈：PX4 v1.17 SITL + Gazebo Harmonic (gz sim 8) + MAVROS。不要换技术栈。
3. 复用 PX4 自带资产：Tools/simulation/gz/worlds/aruco.sdf、
   models/arucotag、models/x500_mono_cam_down。不要自己从零写飞行器 SDF。
4. 工作目录：~/ArUco_Drone/ArUco_Drone_ws/src （当前为空，从零建）。
5. 必须交付 5 套可独立复现的流程 F1~F5，每套一个 launch + 一个自动验收脚本。
6. 严格按需求书的里程碑 M0→M6 推进。**每完成一个里程碑就停下来汇报**，
   附上你实际跑出来的终端输出和关键话题数据，等我确认后再进入下一个。
   不要一次性把 5 套流程全写完再测。

现在只做 M0：打通 PX4 + Gazebo + MAVROS + 相机图像 + OFFBOARD 起飞悬停。
M0 通过并向我汇报后停止。
```

---

## 1. 项目目标

做一个**纯仿真**的无人机项目（无真机、无室外场地），用 ROS 2 + C++ 实现：

1. 在 Gazebo 里有一个带**下视相机**的四旋翼，地面贴一张 ArUco 码；
2. 操作者可以用**键盘实时遥控**飞行；
3. 按一个键，无人机**自动切到 ArUco 视觉模式**：搜索码 → 对准 → 自主降落到码上；
4. 另外支持**航点自主导航**（飞几个点、定点悬停）；
5. 上述能力可以**组合成多套完整任务流程**，每套一条命令跑起来，并且能自动判定成败。

一句话验收：`ros2 launch drone_bringup flow_f2.launch.py` 之后什么都不用按，无人机自己起飞、找到码、落上去，终端最后打印 `RESULT: PASS`。

---

## 2. 环境事实（已实测，勿重复踩坑）

| 组件 | 版本 / 路径 | 状态 |
|---|---|---|
| 系统 | Ubuntu 22.04 on WSL2，WSLg 可用（`DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`） | ✅ |
| ROS 2 | **Humble**，`/opt/ros/humble` | ✅ |
| Gazebo | **Gazebo Sim 8.10.0** = `gz sim`（Harmonic）；另有 Ignition Fortress 6.17（`ign gazebo`）。**Gazebo Classic 未安装**（无 `gazebo` 命令） | ✅ |
| PX4 | `~/PX4-Autopilot`，`v1.17.0-alpha1-338-gdf42ef84f1`，已有 `build/px4_sitl_default`（编译过） | ✅ |
| MAVROS | `mavros` / `mavros_msgs` / `mavros_extras` 均已安装 | ✅ |
| ArUco | C++ `aruco_ros` + `aruco_msgs` 已安装；Python `cv2 4.13.0` 含 `cv2.aruco`；系统 C++ OpenCV 4.5.4 | ✅ |
| 桥接 | `ros_gz_bridge` / `ros_gz_image` / `ros_gz_sim` / `ros_ign_gazebo` | ✅ |
| 其他 | `xacro`、`cv_bridge`、`image_transport`、`as2_gazebo_assets`（Aerostack2，**不要用**） | ✅ |
| 硬件 | 24 vCPU，内存 7.6 GiB（WSL2 上限，偏紧） | ⚠️ |
| 工作区 | `~/ArUco_Drone/ArUco_Drone_ws`（**空目录**）；`~/aruco_det_ros2` 是空壳包（只有 CMakeLists，无 .cpp）；`~/drone_1` 是旧尝试的 Gazebo 模型 | — |

### PX4 已自带的现成资产（关键，直接复用）

| 资产 | 路径 | 说明 |
|---|---|---|
| ArUco 世界 | `~/PX4-Autopilot/Tools/simulation/gz/worlds/aruco.sdf` | 只有地面 + 一张 `arucotag`，原点 0,0 |
| ArUco 码模型 | `.../models/arucotag/`（`model.sdf` + `arucotag.png`） | 0.5 m × 0.5 m 平面贴图，位于 0,0,0.001 |
| **下视相机机型** | `.../models/x500_mono_cam_down` | **就是本项目的目标机型** |
| **下视相机参数** | `.../models/mono_cam/model.sdf` | 传感器名 `camera`，`gz_frame_id = camera_link`，**1280×960 @ 30 Hz**，`horizontal_fov = 1.74 rad`（≈100°，广角），相对 `base_link` 俯仰 +90°（即正下方） |
| 备用机型 | `x500_mono_cam`（前视）、`x500`（无相机）、`x500_depth` | 备用 |

### 由此得出的 7 条硬结论（每条都有原因，别改）

1. **必须用 Gazebo Harmonic（`gz`），不能用 Gazebo Classic** —— 机器上根本没有 `gazebo` 可执行文件，PX4 v1.17 的 gz 后端也只支持 Harmonic（`gz-sim8`）。写 `ros2 launch gazebo_ros ...` 那套教程全部作废。
2. **必须用 MAVROS，不要用 uXRCE-DDS / `px4_msgs`** —— 机器上**没有** `px4_msgs`、没有 `MicroXRCEAgent`；而 MAVROS 已经装好。uXRCE-DDS 还要处理 PX4 微秒时间戳和 `use_sim_time` 的对齐，对大一项目属于纯负担。
   - **连接串必须是** `fcu_url:=udp://:14540@127.0.0.1:14580`。依据是本机实读的 `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`：
     ```
     udp_offboard_port_local  = 14580 + px4_instance   # PX4 自己 bind 14580
     udp_offboard_port_remote = 14540 + px4_instance   # PX4 往外发到 14540
     mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote
     ```
     MAVROS 的 URL 语义是 `udp://[bind_host]:bind_port@[remote_host]:remote_port`，所以要 **bind 14540、发往 14580**。
   - ⚠️ 网上老教程一律写 `udp://:14540@127.0.0.1:14557`（那是 PX4 ≤1.13 的端口分配），**在本机连不上**，不要照抄。连不上时用 `ss -ulnp | grep 145` 看谁占着端口来定位。
3. **复用 `aruco.sdf` + `x500_mono_cam_down`** —— 一条命令起飞，省掉自己写 SDF / world / 相机位姿的两天。**确切命令是 `make px4_sitl gz_x500_mono_cam_down_aruco`**：
   - PX4 v1.17 为「每个机型 × 每个世界」生成独立 make 目标，命名规则就是 `gz_<model_name>_<world_name>`（源码 `src/modules/simulation/gz_bridge/CMakeLists.txt` 第 94 行原文注释：`# Syntax: gz_<model_name>_<world_name>`）。
   - 已用 `ninja -C build/px4_sitl_default -t targets` 确认 `gz_x500_mono_cam_down_aruco` 目标真实存在。不带 `_aruco` 后缀的是 default 世界，**里面没有码**。
   - 换场景只改后缀：`..._baylands`、`..._walls`、`..._forest`、`..._lawn` 等，与 `worlds/*.sdf` 一一对应。
4. **`arucotag.png` 的实际黑框边长必须先量再写参数** —— 那个 0.5 m 是平面尺寸，码本身（黑框外沿）更小。`solvePnP` 的 `marker_size` 填错，测出来的距离会整体缩放错，降落高度判据全崩。**M1 阶段必须用平面真值校准这个值**（见 §8.2）。
5. **内存只有 7.6 GiB** —— 同时开 PX4 + gz GUI + RViz 会 OOM。规则：默认关 RViz；调试时才开；CI/批量跑用无 GUI（`gz sim -s`）。
6. **时间统一用仿真时间** —— 所有 launch 加 `use_sim_time:=true`，并把 gz 的 `/clock` 桥接进 ROS。否则 `mavros` 和控制器的时间戳对不上，TF 会间歇性报 `LookupException`。
7. **无人机和 ArUco 码都出生在原点** —— `aruco.sdf` 里 `arucotag` 的 pose 是 `(0,0,0.001)`；而 PX4 v1.17 **没有** `PX4_GZ_MODEL_POSE` 这个环境变量（已全仓库 grep，`src/` 和 `Tools/` 里零命中），机型只能落到 `model.sdf` 的默认位姿 = 原点。
   - 好处：起飞即在码正上方，F2「精确降落」天然成立，`marker_size` 校准也方便（悬停 2 m，真值距离就是 2 m）。
   - 代价：没有"从远处飞过来找码"的初始条件。**F4 想演"先导航、再降落"，就把流程设计成"起飞 → 飞离到 (3,0,2) → 再回原点上方 → 搜码降落"，不要试图改出生点。**
   - 若一定要让码偏移：把 `aruco.sdf` 复制成 `drone_bringup/worlds/aruco_offset.sdf`，改 `arucotag` 的 `<pose>`，再把该目录塞进 `GZ_SIM_RESOURCE_PATH`、用 `./bin/px4` 直接启动（`make` 不会为自建世界生成目标）。**这条路必须实测；跑不通就回到上面的方案 A，别在它上面耗超过半天。**

---

## 3. 目录结构（交付形态）

```text
~/ArUco_Drone/ArUco_Drone_ws/
├── src/
│   ├── drone_interfaces/        # 自定义 msg/srv（只放最小集合）
│   ├── drone_bringup/           # 所有 launch + 参数 yaml + world 覆盖
│   ├── drone_control/           # C++ 库：飞控封装（起飞/降落/速度/位置/ENU-NED）
│   ├── drone_aruco/             # C++：aruco_detector、landing_controller
│   ├── drone_nav/               # C++：waypoint_nav
│   ├── drone_mission/           # C++：mission_manager（模式调度 + 状态机）
│   ├── drone_teleop/            # Python：键盘遥控（termios 单键）
│   └── drone_tools/             # Python：verify_flow.py、bag 录制、marker 校准
├── scripts/
│   ├── sim.sh                   # 起 PX4 SITL（gz_<model>_aruco 目标）
│   ├── run_flow.sh              # 一条命令跑某套流程（含超时和退出码）
│   └── check_env.sh             # 环境自检（版本/包/端口）
├── docs/
│   ├── README.md                # 安装 + 5 套流程的运行方法 + 排错表
│   ├── flows.md                 # 每套流程的时序与验收数据
│   └── troubleshooting.md
└── logs/                        # 每次跑流程的 bag + 终端日志 + debug 图
```

**强制约束**：`colcon build` 必须零 error；每个包 `package.xml` 写全 `depend`；不要在包外放可达可执行文件。

---

## 4. 架构与数据流

```text
                    ┌──────────── Gazebo Harmonic (gz sim 8) ────────────┐
                    │  aruco.sdf 世界        arucotag 0.5m @ (0,0)       │
                    │        └── x500_mono_cam_down（下视相机）           │
                    └───────────────┬───────────────────────────────────┘
                        ros_gz_bridge│  /camera/image_raw, /camera/camera_info, /clock
                                     ▼
   ┌──────────┐   ┌─────────────────────────┐   /aruco/detection(PoseStamped, frame=map)
   │ 键盘     │   │  aruco_detector (C++)    │──────────────┐
   │ teleop   │   │  OpenCV aruco + solvePnP│              │
   │ (Python) │   └──────────┬──────────────┘              │
   └────┬─────┘              │ /aruco/debug_image          │
        │ /mission/mode      ▼                             ▼
        ▼            （画角点/轴/ID，演示用）      ┌───────────────────────┐
   ┌──────────────────┐                           │ landing_controller    │
   │ mission_manager  │──── 指令 ────────────────▶│ (C++) 状态机           │
   │ (C++) 模式调度    │                           │ SEARCH→ALIGN→DESCEND  │
   └───┬──────────┬───┘                           └──────────┬────────────┘
       │          │                                          │
       ▼          ▼                                          │
 ┌───────────┐ ┌──────────────┐                              │
 │ waypoint_ │ │ drone_control│◀─────────────────────────────┘
 │ nav (C++) │ │ (C++ 库)     │  ← 所有速度/位置指令的唯一出口
 └─────┬─────┘ └──────┬───────┘
       └──────────────┘
                      ▼  /mavros/setpoint_raw/local  (PositionTarget)
               ┌──────────────┐  MAVLink UDP:14540  ┌────────────┐
               │    mavros    │◀───────────────────▶│ PX4 SITL   │
               └──────┬───────┘                     └────────────┘
                      │ /mavros/local_position/pose (ENU)、/mavros/state
                      └──────────────▶ 各控制器（作为唯一位姿来源）
```

**职责分离铁律**：`aruco_detector` 只做视觉，绝不发控制指令；所有 setpoint 只从 `drone_control` 一个出口发；`landing_controller` / `waypoint_nav` 之间不互相调用，全部由 `mission_manager` 调度。

---

## 5. 功能需求

### 5.1 F1 键盘遥控（人工飞）

- Python 节点 `drone_teleop`，`termios` 原始模式，**按一次键立即生效，不用回车**。
- 键位（写死在 README 里，终端打印 help）：

| 键 | 动作 |
|---|---|
| `w` / `s` | 前 / 后 ±0.5 m/s |
| `a` / `d` | 左 / 右 ±0.5 m/s |
| `r` / `f` | 升 / 降 ±0.4 m/s |
| `q` / `e` | 左转 / 右转 ±0.5 rad/s |
| `Space` | 起飞到 1.5 m / 已起飞则原地降落 |
| `1` | 切 `MANUAL` |
| `2` | 切 `ARUCO_LAND`（核心需求） |
| `3` | 切 `WAYPOINT` |
| `4` | 切 `AUTO`（全自动任务链） |
| `0` | 立即降落（LOITER→LAND） |
| `ESC` | 紧急 LAND（不管当前状态） |

- 松键即停（速度指令衰减到 0，不能"粘键"）。实现方式：每 100 ms 检查上次按键时间，超过 300 ms 未被重复触发则速度清零（终端连发不保证，靠定时器兜底）。
- 发布 `/teleop/cmd`（`geometry_msgs/Twist`，body 系），由 `drone_control` 转成 setpoint。

### 5.2 F2 ArUco 检测（纯视觉，可单独测）

- C++ 节点 `aruco_detector`：
  - 订阅 `/camera/image_raw` + `/camera/camera_info`；
  - `cv::aruco::ArucoDetector`，字典用 `DICT_4X4_50`（**先确认 `arucotag.png` 的真实字典与 ID**，不确定就把字典和 ID 都做成参数，M1 里用一张离线图试出来）；
  - `solvePnP` 得到码在相机系下的位姿 → 用 TF 转成 `map` 系 → 发布 `geometry_msgs/PoseStamped` 到 `/aruco/detection`，`frame_id: map`；
  - 位姿做 EMA 平滑（α≈0.3），否则降落时 x/y 抖；
  - 丢失超过 `lost_timeout`(0.5 s) 视为不可见，通过 `MissionStatus.marker_visible=false` 告知；
  - 发布 `/aruco/debug_image`：画四个角点、`cv::drawFrameAxes`、ID、估计距离、`fps`。**这是最终演示视频的主要来源**（WSLg 下 Gazebo GUI 可能卡，debug image 最可靠）。
  - 参数：`marker_size`、`dictionary`、`target_id`、`smooth_alpha`、`lost_timeout`。
  - **必须**用 `CameraInfo` 的 `K` 做内参，禁止硬编码 640/480/fx。

### 5.3 F3 自主降落（核心）

`landing_controller`（C++）状态机：

```text
        ┌──────┐  start(2)
        │ IDLE │────────────┐
        └──────┘            ▼
                     ┌─────────────┐  marker lost > 0.5s
              ┌─────▶│   SEARCH    │──────────────┐
              │      │ 爬升到 2.5m │              │
              │      │ 原地/小幅偏航扫描          │
              │      └──────┬──────┘              │
              │             │ marker visible      │
              │             ▼                     │
              │      ┌─────────────┐  |err|>0.40m │
              │      │   ALIGN     │──────────────┘
              │      │ 像素误差→速度│  (回到 SEARCH 或空中悬停重试)
              │      └──────┬──────┘
              │             │ |err| < 0.15m 且 连续 10 帧
              │             ▼
              │      ┌─────────────┐  marker lost
              │      │  DESCEND    │──────────────▶ 悬停 1.0s 后回 SEARCH
              │      │ vz = -0.30  │
              │      └──────┬──────┘
              │             │ alt < 0.5m
              │             ▼
              │      ┌─────────────┐
              │      │   FINAL     │  vz = -0.12，容差收紧到 0.08m
              │      └──────┬──────┘
              │             │ alt < 0.15m
              │             ▼
              │      ┌─────────────┐
              └──────│  TOUCHDOWN  │ → 上锁 → DONE
                     └─────────────┘
```

控制律（全部用 ENU，内部统一，只在发布时转 NED）：

- 像素误差 → 速度：`vx = clamp(kp_xy * (cx - u)/fx * alt, ±0.6)`，`vy` 同理（`alt` 是当前高度，做尺度归一化，否则高空时增益过大）。
- 偏航：把码的朝向对齐，或者降落阶段干脆锁定偏航（`yaw_rate = 0`），**推荐锁偏航**，简单稳。
- 高度：`ALIGN` 保持 2.5 m（用位置环），`DESCEND`/`FINAL` 用速度环。
- 参数（写进 `config/landing.yaml`）：`align_alt: 2.5`、`descend_speed: 0.30`、`final_speed: 0.12`、`lateral_tol: 0.15`、`final_tol: 0.08`、`kp_xy: 0.8`、`max_vel_xy: 0.6`、`lost_timeout: 0.5`、`search_yaw_rate: 0.3`。
- 安全：任一时刻 `alt < 0.12 m` 或收到 `ESC` → 立即 `LAND` 并上锁；`|vx|,|vy| > 1.0` 时钳位。

### 5.4 F4 航点自主导航

`waypoint_nav`（C++）：
- 航点从 `config/waypoints.yaml` 读（ENU，`[{x,y,z}, ...]`，z 为正高度）；
- 位置 P 控制 + 到达判据（半径 0.25 m 且速度 < 0.2 m/s）→ 切下一个点；
- 偏航朝向下一航点（可选，默认开启，看起来更像"自主导航"）；
- 到达最后一个点后原地悬停 `hover_after: 5.0` s，然后发布 `arrived`，交回 `mission_manager`；
- 必须支持 `PAUSE` / `RESUME`（按 `p`），这是答辩时的加分项。

> 不要用 Nav2。它是 2D 地面导航栈，套到 3D 飞行器上是负收益。

### 5.5 F5 任务调度（"按键切换"的实现）

`mission_manager`（C++）：
- 订阅 `/mission/mode`（`std_msgs/String`），取值：`MANUAL` / `ARUCO_LAND` / `WAYPOINT` / `AUTO` / `HOVER` / `LAND`；
- 任一模式切换都走统一流程：`先停当前模式的输出 → 置 HOVER → 等 0.5s → 启动新模式`，**避免两个控制器同时发 setpoint 打架**（这是最容易出的诡异 bug）；
- 发布 `/mission/status`（见 §6），供 `verify_flow.py` 判定；
- 打印人类可读日志：`[MISSION] MANUAL -> ARUCO_LAND (trigger: key '2')`。

### 5.6 五套流程（交付的核心）

| 流程 | 一条命令 | 内容 | 通过判据 |
|---|---|---|---|
| **F1** 纯手动 | `flow_f1.launch.py` | 起飞 → 键盘飞一圈 → 按 `0` 降落 | 起飞高度 >1.2 m；最终高度 <0.2 m；全程无 crash |
| **F2** 手动转视觉降落 | `flow_f2.launch.py` | 起飞 → 键盘飞到码附近 → 按 `2` → 自动对准降落 | 最终水平误差 ≤0.25 m；最终高度 ≤0.25 m；≤150 s |
| **F3** 纯自主航点 | `flow_f3.launch.py` | 起飞 → 依次飞 3 个航点 → 悬停 | 每个航点稳态误差 ≤0.35 m；全程 ≤180 s |
| **F4** 全自动链路 | `flow_f4.launch.py` | 一条命令：起飞 → 飞到码上方航点 → 搜码 → 降落 | 全程零人工输入；水平误差 ≤0.25 m；≤180 s |
| **F5** 异常处理 | `flow_f5.launch.py` | 下降中 marker 被遮挡（`drone_tools` 临时置不可见 3 s）→ 应中止下降、爬升重搜 → 恢复后继续降落 | 遮挡期间高度回升 ≥1.0 m；不坠机；恢复后仍能完成降落 |

F5 是"多套流程"里最有价值的一套：它证明你的状态机不是脚本，而是真的会处理失败。

---

## 6. 接口契约（照抄，别自创名字）

### 6.1 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | gz→ROS | 必须桥接 |
| `/camera/image_raw` | `sensor_msgs/Image` | gz→ROS | `x500_mono_cam_down` 下视 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | gz→ROS | 内参来源 |
| `/mavros/state` | `mavros_msgs/State` | ←PX4 | `armed` / `mode` |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | ←PX4 | **ENU，z 向上** |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | →PX4 | 控制出口 |
| `/aruco/detection` | `geometry_msgs/PoseStamped` | 内部 | `frame_id: map` |
| `/aruco/debug_image` | `sensor_msgs/Image` | 内部 | 演示/调试 |
| `/teleop/cmd` | `geometry_msgs/Twist` | 内部 | body 系速度 |
| `/mission/mode` | `std_msgs/String` | 内部 | 模式指令 |
| `/mission/status` | `drone_interfaces/MissionStatus` | 内部 | 验收数据源 |
| `/mission/set_mode` | `drone_interfaces/SetMode` | srv | 供 launch 脚本调用 |

### 6.2 自定义消息（`drone_interfaces`，就这两个，别扩）

```text
# msg/MissionStatus.msg
string state            # IDLE/TAKEOFF/MANUAL/SEARCH/ALIGN/DESCEND/FINAL/LANDED/WAYPOINT/ABORT
string mode             # 当前模式
string flow             # F1..F5
float32 altitude        # 当前高度 m (ENU z)
float32 lateral_error   # 相对目标码的水平误差 m，无目标时 -1
float32 vertical_speed  # m/s，向上为正
bool    marker_visible
bool    armed
builtin_interfaces/Time stamp

# srv/SetMode.srv
string mode             # MANUAL/ARUCO_LAND/WAYPOINT/AUTO/HOVER/LAND
---
bool success
string message
```

### 6.3 参数文件

`drone_bringup/config/` 下：`mavros.yaml`、`landing.yaml`、`waypoints.yaml`、`aruco.yaml`、`flows.yaml`。所有数值参数**只在这里改**，代码里不许出现魔法数字。

### 6.4 三个必须记住的坑（写进代码注释）

1. **ENU / NED 符号相反**：`/mavros/local_position/pose` 是 ENU（z 向上），而 `/mavros/setpoint_raw/local` 在 `FRAME_LOCAL_NED` 下是 **NED（z 向下）**。下降指令 `vz = +0.30`（NED 正下），别写成 `-0.30` 然后发现无人机往天上冲。统一在 `drone_control` 内做转换，接口对外只用 ENU。
2. **setpoint 掩码**：
   - 速度控制：`type_mask = 0b0000111111000111`（忽略位置/加速度/力/yaw，使用 `vx,vy,vz` + `yaw_rate`）
   - 位置控制：`type_mask = 0b0000111111111000`（忽略速度/加速度/力，使用 `position` + `yaw_rate`）
   - 掩码和填充字段必须一致，否则 PX4 会静默忽略你的一半指令。
3. **OFFBOARD 的前置条件**：必须先以 ≥2 Hz 持续发 setpoint（建议 20–50 Hz），再调 `SetMode OFFBOARD`、再 `arming`。顺序错了 PX4 直接拒绝，且错误码不明显。固定节奏：发 2 s setpoint → 切 OFFBOARD → 等状态回读 → arming → 等 `armed==true`。

---

## 7. 非功能需求

- **控制频率**：setpoint 发布 50 Hz（定时器），视觉 ≤30 Hz，状态 10 Hz。
- **CPU**：F4 全程 `top` 里单个 ROS 节点 <30%，gz 的 real-time-factor ≥0.7（低于 0.5 要在 README 里写明并降载：关 GUI、降相机分辨率）。
- **内存**：默认不启 RViz。`ros2 topic hz` 抖动要记录。
- **日志**：每次跑流程，`logs/<flow>_<timestamp>/` 里留：终端完整输出、`ros2 bag`（`/mavros/local_position/pose`、`/aruco/detection`、`/mission/status`）、至少 3 张 `debug_image` 截图。
- **可复现**：同一命令跑 3 次，F2 的最终水平误差离散度 <0.1 m。不满足就说明控制有随机性，要查（通常是没等 EKF 收敛就起飞）。
- **代码风格**：`ament_lint` 通过；C++ 用 `rclcpp` 的 `create_wall_timer`；不允许 `while(ros::ok())` 阻塞主线程。

---

## 8. 验收标准（可自动判定）

### 8.1 逐条硬指标

1. `colcon build --symlink-install` 零 error、零 warning（`-Wall -Wextra`）。
2. `colcon test && colcon test-result --verbose` 全通过（至少要有 F3 状态机的 gtest 单测：给定模拟的 marker 序列，验证状态迁移正确）。
3. `scripts/check_env.sh` 全绿：ROS 版本、gz 版本、PX4 路径、OpenCV aruco、14540 端口。
4. F1~F5 **各自**一条命令跑通，且 `verify_flow.py` 打印 `RESULT: PASS`。
5. F2 连跑 3 次都 PASS。
6. 无 `[ERROR]` / segfault / `LookupException` 出现在任何一次运行的日志里。
7. README 里的每条命令，在一个**干净 shell** 里照抄能跑（不依赖任何手工 `export`）。

### 8.2 校准步骤（M1 必做，否则验收数字全是假的）

`marker_size` 必须实测：

```bash
# 让无人机悬停在码正上方已知高度，例如 z=2.0
gz model -m x500_mono_cam_down_0 -p          # 读真值位姿
ros2 topic echo /aruco/detection --once      # 读估计位姿
# 两者距离应相差 <0.05 m。差得多就调 marker_size：距离估计与 marker_size 成正比。
```

把校准后的值写进 `config/aruco.yaml`，并在 README 里注明"已按 X m 校准"。

### 8.3 验收脚本的输出格式（写死，便于肉眼和 CI 判断）

```text
[FLOW F2] marker_detected      = yes
[FLOW F2] final_lateral_error  = 0.11 m   (<= 0.25  PASS)
[FLOW F2] final_altitude       = 0.18 m   (<= 0.25  PASS)
[FLOW F2] duration             = 63.4 s   (<= 150   PASS)
[FLOW F2] ros_errors           = 0
[FLOW F2] RESULT: PASS
```

非零退出码 + `RESULT: FAIL` 时，要打印**失败原因**（哪个指标超了、超了多少），并保留当次 bag。

---

## 9. 实施里程碑（每个结束必须停下来汇报）

| 里程碑 | 内容 | 完成标志（贴什么给用户看） |
|---|---|---|
| **M0** | 环境打通 | `make px4_sitl gz_x500_mono_cam_down_aruco` 起飞成功；`ros2 topic hz /camera/image_raw` 有 30 Hz；`/mavros/state` 显示 `connected: true`；`/mavros/local_position/pose` 高度随起飞变化；`ros2 run drone_control hover_test` 能 OFFBOARD 悬停 1.5 m 30 s 不漂 |
| **M1** | ArUco 检测 | 一张 `arucotag.png` 离线识别出 ID；悬停时 `/aruco/detection` 与 `gz model -p` 真值误差 <0.05 m；`/aruco/debug_image` 能看到角点框和坐标轴（贴图） |
| **M2** | 飞控封装 + 键盘 | `drone_control` 库接口齐（takeoff/land/hover/velocity/goto、ENU-NED 转换、OFFBOARD 前置流程）；F1 可玩 |
| **M3** | 自主降落 | F2 PASS；贴一段 30 s 的 debug_image 序列或 bag 里的高度/误差曲线 |
| **M4** | 航点导航 | F3 PASS |
| **M5** | 任务调度 | F4 PASS，模式切换日志清晰 |
| **M6** | 异常 + 交付 | F5 PASS；README/troubleshooting 完成；5 套流程各留一份 logs；`colcon test` 全绿；`check_env.sh` 全绿 |

**汇报纪律**：每个里程碑只报三件事——① 我跑的命令，② 真实终端输出（贴原文，不要美化），③ 下一个里程碑准备做什么。不要在没有真实输出的情况下声称"已完成"。

---

## 10. 明确禁止的事（防止跑偏，逐条都有原因）

1. ❌ 用 Gazebo Classic / `gazebo_ros` —— 机器上没有该二进制，所有相关教程作废。
2. ❌ 引入 `px4_msgs` + uXRCE-DDS —— 未安装 `MicroXRCEAgent`，且时间戳同步会吃掉两天。统一 MAVROS。
3. ❌ 用 Nav2 / MoveIt / Aerostack2（`as2_gazebo_assets` 虽在但别碰）—— 3D 飞行器用不上，只会让依赖爆炸。
4. ❌ 用 `aruco_ros` 作为运行时依赖 —— 它假设相机装在机械臂末端、给的是相机系位姿；我们的 `aruco_detector` 要自己写（这是项目的"自主识别"卖点）。`aruco_ros` 只允许拿来对照验证。
5. ❌ 硬编码相机内参 —— 必须读 `CameraInfo`。
6. ❌ 用 `sleep` 等起飞 —— 必须用状态反馈（`armed` + 高度 > 阈值 + `mode==OFFBOARD`）。
7. ❌ 在视觉节点里发控制指令 —— 破坏职责分离，后续调不动。
8. ❌ 修改 `~/PX4-Autopilot` 源码 —— 只允许读它的 world/model 资产，需要改就在 `drone_bringup/worlds/` 里复制一份改。
9. ❌ 一次性写完 5 套流程再测 —— 必须 M0 绿了才写 M1，否则错误会叠加到无法定位。
10. ❌ 改用户现有的 `~/drone_1`、`~/aruco_det_ros2` —— 那些是旧尝试，只在需要时**读**参考，动就在 `ArUco_Drone_ws` 里另建。

---

## 11. 已知风险与 Plan B

| 风险 | 现象 | 处置 |
|---|---|---|
| WSLg 下 gz GUI 卡/黑屏 | 窗口出来但画面不刷新，RTF 掉到 0.1 | 先试 `LIBGL_ALWAYS_SOFTWARE=1`；仍不行就用 `gz sim -s`（server only，无 GUI），演示全靠 `/aruco/debug_image` + `ros2 topic echo`。**演示视频用 debug_image 生成**，这是最稳的路径 |
| 内存不足 | PX4 或 gz 被 OOM kill | 关 RViz；相机降到 640×480；`~/.wslconfig` 里 `memory=12GB`（需用户重启 WSL，作为最后手段并明确告知） |
| 相机话题名对不上 | `ros_gz_bridge` 起来但 `/camera/image_raw` 没数据 | 用 `gz topic -l | grep -i image` 查真实话题名（形如 `/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`），把 bridge 配置改成实际名 |
| 起飞就漂走 | OFFBOARD 后水平速度越来越大 | 典型的 ENU/NED 符号或掩码错误；先单独用最简节点只发 `vz=0` 的零速度验证悬停，再叠加速度指令 |
| EKF 未收敛就起飞 | 高度乱跳、降落判据误触发 | 起飞前等 `/mavros/local_position/pose` 稳定 3 s（方差阈值），写进 `drone_control::waitReady()` |
| `marker_size` 错 | 估计距离整体偏大/偏小，降落高度判据失效 | §8.2 校准；把校准前后的误差写进 docs |
| ArUco 字典猜错 | 一张码都检测不到 | 用 `cv2.aruco` 逐字典暴力试 `arucotag.png`，把结果记进 docs，参数化字典 |
| 时间不同步 | TF `LookupException`、bag 回放对不上 | 所有节点 `use_sim_time:=true`；桥接 `/clock`；`ros2 param get /aruco_detector use_sim_time` 核对 |

**最终 Plan B（只在 M0 卡超过一天时启用）**：写一个纯 Python/C++ 的"运动学假仿真"节点，直接积分速度得到位姿、用 OpenCV 把一张地面图按相机位姿做透视变换生成下视图像。这样 F1~F5 全部可跑、可验收、可演示，日后接回真 PX4 只需换掉图像源和位姿源。**但这是退路，先尽全力把 M0 打通**，因为真 PX4 栈的答辩价值高得多。

---

## 12. 环境速查命令

```bash
# --- 环境自检 ---
source /opt/ros/humble/setup.bash
gz sim --version                                   # 期望 8.10.0
ros2 pkg list | grep -E 'mavros|ros_gz|cv_bridge'
pkg-config --modversion opencv4
ls ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down

# --- 起仿真（终端 1）---
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco
# 模型实例名/相机话题名以 gz topic -l 实际输出为准；
# 相机预期：/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image

# --- 桥接 + mavros（终端 2，由 drone_bringup 的 launch 统一做）---
ros2 launch drone_bringup sim.launch.py use_sim_time:=true

# --- 验证 ---
ros2 topic hz /camera/image_raw
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
gz topic -l | grep -i image
gz model -m x500_mono_cam_down_0 -p        # 真值位姿，校准用

# --- 跑流程 ---
./scripts/run_flow.sh F2
ros2 run drone_tools verify_flow.py --flow F2 --timeout 180
```

---

## 13. 一句话总结给执行者

> 先用 `make px4_sitl gz_x500_mono_cam_down_aruco` + MAVROS 把"能起飞、能看见下视图像"这件事做扎实（M0），再写 ArUco 检测（M1），再写控制（M2-M4），最后才拼流程（M5-M6）。**每个里程碑用真实终端输出汇报，不许提前宣布完成。** 五个最容易翻车的点：ENU/NED 符号、`marker_size` 校准、OFFBOARD 前置顺序、MAVROS 端口（14580 而非 14557）、gz 相机话题名。
