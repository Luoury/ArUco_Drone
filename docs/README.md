# ArUco 自主降落仿真 — 工程说明

PX4 v1.17 SITL + Gazebo Harmonic（gz sim 8.10）+ ROS 2 Humble + MAVROS。
需求书见仓库根目录 `无人机ARUCO自主降落仿真_需求书.md`。

## 目录结构

```
ArUco_Drone/
├── ArUco_Drone_ws/src/
│   ├── drone_interfaces/   # msg/MissionStatus.msg, srv/SetMode.srv
│   ├── drone_bringup/      # launch + config（唯一参数来源）+ gz_sensor_bridge
│   ├── drone_control/      # C++ 飞行控制库 flight_control + hover_test
│   ├── drone_aruco/        # aruco_detector
│   ├── drone_mission/      # mission_manager（模式调度 + 设定点唯一出口）
│   ├── drone_teleop/       # 键盘遥控 teleop_node
│   └── drone_tools/        # verify_flow 流程自动验收
├── scripts/                # 启动、诊断、验收脚本
├── docs/                   # 本文档、troubleshooting.md、stage_reports.md
└── logs/                   # 运行日志（git 忽略）
```

## 快速开始（三步）

```bash
# 1) 起 SITL（PX4 + Gazebo，headless + Ogre1）
bash ~/ArUco_Drone/scripts/sim.sh start aruco

# 2) 起桥接 + MAVROS（/clock、/camera/image_raw、/mavros/*）
source /opt/ros/humble/setup.bash
source ~/ArUco_Drone/ArUco_Drone_ws/install/setup.bash
ros2 launch drone_bringup sim_bringup.launch.py

# 3) 起飞 + OFFBOARD 悬停 30 s（自动降落收尾）
ros2 run drone_control hover_test
```

一键联调（含健康检查与验收）：
`bash scripts/dev_run_hover.sh`（M0）、`bash scripts/dev_run_aruco.sh`（M1）、
`bash scripts/dev_run_f1.sh`（M2 的 F1 流程）、`bash scripts/dev_run_f2.sh`（M3 的 F2 流程）、
`bash scripts/dev_run_f3.sh`（M4 的 F3 流程），
每个都自动跑完并打印 `[FLOW Fx] RESULT:`。

人工试飞 F2（键盘按下 `2` 交给视觉降落；`w/a/s/d/r/f/q/e` 手动飞）：

```bash
bash ~/ArUco_Drone/scripts/start_until_healthy.sh aruco 3
source /opt/ros/humble/setup.bash && source ~/ArUco_Drone/ArUco_Drone_ws/install/setup.bash
ros2 launch drone_bringup flow_f2.launch.py start_teleop:=true
```

人工试飞 F1（需要真实终端，键盘模式下才看得到按键响应）：

```bash
bash ~/ArUco_Drone/scripts/start_until_healthy.sh aruco 3
source /opt/ros/humble/setup.bash && source ~/ArUco_Drone/ArUco_Drone_ws/install/setup.bash
ros2 launch drone_bringup flow_f1.launch.py start_teleop:=true
```

停止：`bash scripts/sim.sh stop`；清 ROS 进程：`bash scripts/stop_ros.sh`。

## 关键约定

| 项 | 值 |
| --- | --- |
| MAVROS 连接串 | `udp://:14540@127.0.0.1:14580` |
| 控制频率 | 50 Hz（`/mavros/setpoint_raw/local`） |
| 位姿坐标系 | `/mavros/local_position/pose` 为 **ENU**；mavros 自己转 NED，代码里**不要**再转 |
| `type_mask` | 位置 `0b0000111111111000`，速度 `0b0000111111000111` |
| 起飞方式 | `AUTO.TAKEOFF` → 离地后按 §5.3 顺序交 OFFBOARD（本机 OFFBOARD 无法从地面起飞） |
| 相机 | 1280×960 @30 Hz，`fx=fy≈539.94`，`cx=640, cy=480`，光轴指向机体 -z |
| ArUco | `DICT_4X4_50`，id=0，`marker_size=0.487 m`（实测标定值，非名义 0.5） |
| 参数来源 | 一律 `drone_bringup/config/*.yaml`，代码里不写魔法数字 |
| 相机轴系 | `x_body=-y_cam, y_body=-x_cam, z_body=-z_cam`（`kOptToFlu`，行列式 +1） |
| 降落接管 | `mission_manager` 置 HOVER → 等 0.5 s → `/aruco/landing/start`，交接期间不再发 setpoint |

## 里程碑状态

- **M0 环境打通：PASS**。`hover_test` 起飞到 1.5 m 悬停 30 s：
  `final_altitude = 1.486 m`、`max_altitude_error = 0.146 m`（容差 0.15）、
  `max_horizontal_drift = 0.045 m`（容差 0.30）、`RESULT: PASS`。
- **M1 视觉检测：功能 PASS，绝对精度未稳定达标 0.05 m**。
  - 离线：`m1_detect_offline.py` 识别出 id=0（`DICT_4X4_50`）。
  - 在线：`/aruco/detection` 稳定输出（13~30 Hz，`frame_id=map`），
    `/aruco/debug_image` 正确叠加角点框与坐标轴（见 `docs/images/m1_debug_image.png`）。
  - 精度：2 m 悬停下平均误差 0.026 m、单帧中位误差 0.061 m（判据 ≤0.05 m），
    垂直方向已压到 −0.02 m（`marker_size` 标定生效）。
    残余误差来源：PX4 EKF 相对 gz 真值本身有 2~6 cm 抖动
    （`m1_frame_check.py` 实测 14 组，无恒定偏置），逐帧噪声再叠加约 2 cm。
    后续在降落控制里用时序滤波（需求书 §7 本来就要做）收敛这一项。
- **M2 飞控封装 + 键盘 + F1：PASS**。`verify_flow --flow F1` 实测
  `takeoff_altitude = 1.95 m`（判据 >1.2）、`final_altitude = 0.03 m`（判据 <0.2）、
  飞行中最低 1.84 m（无 crash）→ `[FLOW F1] RESULT: PASS`。
  （MANUAL 已改成"速度积分成位置目标"，掉高问题不复存在。）
- **M3 视觉伺服降落 + F2：PASS**。`verify_flow --flow F2` 实测：从 0.81 m 水平误差
  接管，`IDLE → ALIGN → DESCEND → FINAL`，`landed = yes`，
  `final_lateral = 0.01 m`、`final_altitude = -0.01 m`、`elapsed = 31.1 s`
  （判据 0.25 / 0.25 / 150）→ `[FLOW F2] RESULT: PASS`。
- **M4 航点导航 + F3：PASS**。`verify_flow --flow F3` 实测：只发一条 `WAYPOINT` 模式指令，
  三点稳态误差 `0.158 / 0.075 / 0.102 m`（判据 ≤0.35），`elapsed = 30.6 s`（判据 ≤180），
  落地 `final_altitude = -0.04 m` → `[FLOW F3] RESULT: PASS`。
- **M5 全自主 + F4：PASS**。`verify_flow --flow F4` 实测：全程只有一条 `AUTO` 模式请求，
  `IDLE → AUTO_WAYPOINT → AUTO_LAND → AUTO_DONE`，`elapsed = 42.5 s`，
  落点 `final_lateral = 0.04 m`、`final_altitude = -0.03 m` → `[FLOW F4] RESULT: PASS`。
- **M6 异常处理 + F5：PASS**。`verify_flow --flow F5` 在下降途中由 Gazebo 侧真实遮挡 3.0 s
  （最长连续无检测 3.00 s），高度 1.85 → 3.72 m（回升 1.87 m），重新对准后降落，
  `final_lateral = 0.01 m`、`elapsed = 47.0 s` → `[FLOW F5] RESULT: PASS`。

## 脚本清单

| 脚本 | 用途 |
| --- | --- |
| `scripts/sim.sh start\|stop\|status` | 起停 SITL；`status` 打 RTF/iterations/相机话题数 |
| `scripts/start_until_healthy.sh <world> <次数>` | 起 SITL 并轮询 IMU 是否真在发布，坏了自动重启 |
| `scripts/stop_ros.sh` | 清掉 bringup/检测节点 |
| `scripts/check_env.sh` | 环境自检（ROS/PX4/gz/包） |
| `scripts/fix_gz_plugins.sh apply\|restore\|show` | 替换 PX4 的 gz 插件清单（去 GstCameraSystem 泄漏 + 锁 Ogre1） |
| `scripts/fix_gz_model.sh apply\|restore\|show` | 展平 x500 模型（绕开 merge 丢传感器） |
| `scripts/set_px4_param.py <名> <值>` | rclpy 版 PX4 参数设置（绕开 ros2 CLI daemon） |
| `scripts/offboard_probe.py pos\|vel <v>` | 直接发 setpoint_raw/local 的最小探针 |
| `scripts/probe_motors.sh` | 采样 gz 电机指令，判断 PX4 是否真在出力 |
| `scripts/dev_run_hover.sh` | M0 一键联调 + 验收 |
| `scripts/dev_run_aruco.sh` | M1 一键联调 + 验收（`ARUCO_POSE_TOPIC` 可切位姿源） |
| `scripts/dev_run_f1.sh` | M2：起 SITL + `flow_f1.launch.py` + `verify_flow --flow F1`，跑完自动收尾 |
| `scripts/dev_run_f2.sh` | M3：起 SITL + `flow_f2.launch.py` + `verify_flow --flow F2` |
| `scripts/dev_run_f3.sh` | M4：起 SITL + `flow_f3.launch.py` + `verify_flow --flow F3` |
| `scripts/dev_run_f4.sh` | M5：F4 全自主链路验收（`M5_PROBE=1` 同时录设定点） |
| `scripts/dev_run_f5.sh` | M6：F5 异常处理验收（Gazebo 侧真实遮挡 3 s） |
| `scripts/run_flow.sh F1..F5` | 五套流程统一入口 |
| `scripts/trace_alt.py [秒]` | 10 Hz 录高度+模式到 CSV，判断"真掉高"还是"读数跳变" |
| `scripts/dev_manual_probe.sh` | 起 F2 链路 → 发 `/mission/mode=MANUAL` → 录高度 |
| `scripts/dev_run_det_probe.sh` / `dev_det_probe.py` | 飞离码上空后逐帧比对"真实偏移 vs 估计偏移"（验相机轴系） |
| `scripts/m1_detect_offline.py` | 离线识别 `arucotag.png` |
| `scripts/m1_check_detection.py` | 在线精度校验（稳态悬停后再采样） |
| `scripts/m1_detector_accuracy.py` | 把 EKF 误差与检测器误差分开度量 |
| `scripts/m1_save_debug.py <png>` | 存一张 `/aruco/debug_image` 供人工核对 |
| `scripts/m1_frame_check.py` | 比对 `gz model -p` 与 mavros 位姿，量化坐标系偏置 |
| `scripts/sync.sh` | 开发机专用：把 Windows 暂存目录同步进工作区 |
