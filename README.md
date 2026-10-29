# 无人机 ArUco 自主降落仿真

PX4 SITL + Gazebo Harmonic + ROS 2 Humble 的 ArUco 视觉自主降落工程：无人机从基站起飞、
飞到工位正上方的码、用相机对准并降落到码上，支持手动遥控、单码降落、多航点、全自主链路
和遮挡异常处理五种玩法，全部有一键验收脚本与录屏。

- 五套流程 **F1–F5 全部 PASS**（判据统计见 [docs/flows.md](docs/flows.md)）
- 验收：单码降落落点水平误差 **0.01–0.10 m**、一轮完整工位巡回 **110.5 s**
- 无窗口也能出视频（离屏机位 + 自建 gz 桥接 + HUD），见 [docs/video_demo.md](docs/video_demo.md)

> 本文只讲"怎么跑起来"。每个脚本的作用见 [scripts/README.md](scripts/README.md)，
> 踩过的坑见 [docs/troubleshooting.md](docs/troubleshooting.md)，全部实现细节见
> [docs/stage_reports.md](docs/stage_reports.md)。

---

## 一、跑仿真（20 分钟内可复现）

### 1. 环境要求

| 项 | 版本 | 说明 |
|---|---|---|
| Ubuntu | 22.04 | WSL2 也可以（本工程就是在 WSL2 上开发验证的） |
| ROS 2 | Humble | `/opt/ros/humble` |
| Gazebo | Harmonic（`gz sim` 8.x） | 提供 `libgz-transport13`、`libgz-msgs10` |
| PX4 | v1.17（源码 + 已编译 `px4_sitl`） | 需要 `~/PX4-Autopilot` |
| OpenCV | 4.5.4（含 aruco 模块） | `libopencv-aruco-dev` |
| 其他 | `mavros` / `mavros_msgs` / `cv_bridge` / `rqt_image_view` | apt 安装 |

依赖一键检查：

```bash
bash scripts/check_env.sh
```

### 2. 编译

```bash
cd ~/ArUco_Drone/ArUco_Drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 内存 8 GB 的机器建议加 `--parallel-workers 1` + `MAKEFLAGS=-j4`，否则并行编译容易把系统拖死。

### 3. 跑之前的两次"环境修补"（只做一次，脚本幂等）

```bash
bash scripts/fix_gz_plugins.sh apply    # 去掉 PX4 默认清单里会泄漏内存的 GstCameraSystem
bash scripts/fix_gz_model.sh apply      # 绕开 gz 富模型 merge 丢传感器的 bug（否则 PX4 起不来）
```

这两步都是改 `~/PX4-Autopilot` 下的文件，第一次执行会自动备份（`*.px4bak`），
`restore` 子命令可还原。**不做的后果**：内存一路涨到把机器卡死 / PX4 报 `Accel Sensor 0 missing`。

### 4. 起飞

```bash
bash scripts/sim.sh start aruco            # 起 PX4 + Gazebo（含 IMU 健康检查）
bash scripts/manual_demo.sh start          # 再起检测、任务调度、降落控制器（F2 模式）
bash scripts/manual_demo.sh teleop         # 另开一个终端：键盘遥控
```

键位：`Space` 起飞 1.5 m / 已起飞则降落，`w a s d` 平移，`r f` 升降，`q e` 转向，
`1` 手动、`2` 视觉降落、`3` 航点、`4` 全自主、`0` 降落、`ESC` 紧急降落。
完整清单见 [docs/manual_test.md](docs/manual_test.md)。

### 5. 看画面

```bash
bash scripts/manual_demo.sh view     # rqt 看检测叠加图（有角点框、坐标轴、距离、帧号）
bash scripts/manual_demo.sh gui      # WSLg 里开 Gazebo 3D 窗口
```

3D 窗口在 WSL2 上偶发卡顿（软件渲染）。要稳定看/录，用离屏机位：

```bash
bash scripts/record_video.sh F2 90 auto    # 自动跑一遍 F2 并录成 logs/videos/*.mp4
```

### 6. 五套流程一键验收

```bash
bash scripts/run_flow.sh F1     # 手动起飞降落
bash scripts/run_flow.sh F2     # 手动飞到码附近 → 按 2 → 自动对准降落
bash scripts/run_flow.sh F3     # 三个航点自主飞行
bash scripts/run_flow.sh F4     # 全自主：起飞 → 航点 → 搜码 → 降落（零人工输入）
bash scripts/run_flow.sh F5     # 全自主 + 下降途中被遮挡 3 s，要求回升后仍能降落
```

每条流程结尾都会打印 `[FLOW Fx] RESULT: PASS/FAIL`，失败会指出超了哪个指标。
判据与实测值对照见 [docs/flows.md](docs/flows.md)。

### 7. 多工位巡回演示（录视频用）

```bash
bash scripts/demo_tour.sh 280      # 基站 → 三个工位逐个降落 → 回基站，同时录像
```

---

## 二、部署到实机

仿真里所有控制、视觉、状态机代码都是照着"真机可用"写的，迁移到实机主要是换掉
仿真侧的三样东西：**飞控连接、相机、坐标基准**。

### 1. 硬件与接线

- 飞控：PX4 固件（同版本参数可沿用），通过 USB/串口接到机载计算机；
- 机载计算机：Ubuntu 22.04 + ROS 2 Humble + 本工程（不需要 Gazebo）；
- 相机：**朝下**安装在机体腹部，光轴与机体 −z 对齐（与仿真里 `mono_cam` 安装方向一致）；
- 可选：云台/补光灯，室外强光下建议加遮光罩。

### 2. 换成真机的三处配置

| 改什么 | 改哪里 | 说明 |
|---|---|---|
| 飞控连接串 | `drone_bringup/config/mavros.yaml` 的 `fcu_url` | USB 用 `serial:///dev/ttyACM0:921600`，串口用 `serial:///dev/ttyS0:921600`，网口用 `udp://:14540@<飞控IP>:14580`。仿真用的是 `udp://:14540@127.0.0.1:14580`，**不要带进实机** |
| 相机内参 | `drone_bringup/config/aruco.yaml` 的 `camera_fx/fy/cx/cy` | 用 `camera_calibration` 标定，或在 launch 里改成订阅真实 `/camera/camera_info` |
| 码的物理尺寸 | `drone_bringup/config/aruco.yaml` 的 `marker_size` | 实测打印出来的黑框外沿边长（米），量完直接改。仿真实测标定值是 0.487 |

其余参数（`landing.yaml` 的下降速度、容差，`flows.yaml` 的超时）建议先在实机上保守取值：
先降 `descend_speed` 到 0.15、`final_speed` 到 0.08，确认稳定后再按需加快。

### 3. 上机前必须做的三件事

1. **切掉仿真专属节点**：实机不要启动 `sim_bringup.launch.py`（它连的是 Gazebo 的
   `/camera/image_raw` 与 `/clock`）。实机只需启动：`mavros_node` → `aruco_detector` →
   `landing_controller` → `mission_manager`，用 `use_sim_time:=false`。
2. **确认 OFFBOARD 前置流程**：本工程的控制库统一走"先发 2 s 设定点 → 切 OFFBOARD →
   等状态回读 → 解锁 → 等 armed"，实机上必须先在悬停状态验证这一步（`AUTO.TAKEOFF`
   起飞后交给 OFFBOARD），不要在地面直接切 OFFBOARD。
3. **安全设置**：`NAV_DLL_ACT` 置 0 只适用于"无地面站"的仿真场景；**实机请保持默认值**，
   并确保遥控器随时能切回 `POSCTL`/`STABILIZED` 接管。降落段建议有人在旁边握杆。

### 4. 首次实机建议顺序

1. 地面定点：无人机悬停 2 m，看 `/aruco/detection` 的位姿是否稳定（对应仿真里 M1 的精度验证脚本
   `scripts/m1_check_detection.py`，实机把 `drone_pose_topic` 换成 `/mavros/local_position/pose`）；
2. 手动降到 0.5 m 以上观察对准效果（`landing_controller` 的 `align_alt` 改到 1.5 再试）；
3. 再放开完整 `ARUCO_LAND`；
4. 最后才是 `AUTO` 全链路（航点 + 搜码 + 降落）。

---

## 三、目录速查

```
ArUco_Drone/
├── ArUco_Drone_ws/src/
│   ├── drone_interfaces/   自定义消息与服务的接口包
│   ├── drone_bringup/      launch / 参数 / 自建 gz 模型
│   ├── drone_control/      飞控封装库（ENU/NED、OFFBOARD 前置流程）
│   ├── drone_aruco/        ArUco 检测 + 视觉伺服降落
│   ├── drone_nav/          航点导航
│   ├── drone_mission/      模式调度（MANUAL/HOVER/ARUCO_LAND/WAYPOINT/AUTO/TOUR）
│   ├── drone_teleop/       键盘遥控
│   └── drone_tools/        验收脚本与录制工具
├── scripts/                一键脚本（见 scripts/README.md）
├── docs/                   流程说明、踩坑记录、阶段报告、手动测试
└── logs/                   bag 与视频
```

## 许可与致谢

本工程为课程/立项自研代码，依赖 PX4-Autopilot（BSD-3）、Gazebo（Apache-2.0）、
ROS 2（Apache-2.0）、OpenCV（Apache-2.0）。
