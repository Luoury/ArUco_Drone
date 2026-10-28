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
`bash scripts/dev_run_f1.sh`（M2 的 F1 流程，自动跑完并打印 `[FLOW F1] RESULT:`）。

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
  `takeoff_altitude = 2.06 m`（判据 >1.2）、`final_altitude = -0.00 m`（判据 <0.2）、
  飞行中最低 0.98 m（无 crash）→ `[FLOW F1] RESULT: PASS`。
  已知现象：OFFBOARD **速度**模式下 `vz=0` 不保高，水平机动时掉高约 1 m；
  F3 的航点飞行改用**位置**设定点（自带高度保持）后再复核。
- M3 视觉伺服降落 + F2：下一步。

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
| `scripts/m1_detect_offline.py` | 离线识别 `arucotag.png` |
| `scripts/m1_check_detection.py` | 在线精度校验（稳态悬停后再采样） |
| `scripts/m1_detector_accuracy.py` | 把 EKF 误差与检测器误差分开度量 |
| `scripts/m1_save_debug.py <png>` | 存一张 `/aruco/debug_image` 供人工核对 |
| `scripts/m1_frame_check.py` | 比对 `gz model -p` 与 mavros 位姿，量化坐标系偏置 |
| `scripts/sync.sh` | 开发机专用：把 Windows 暂存目录同步进工作区 |
