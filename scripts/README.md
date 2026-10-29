# scripts/ 脚本清单

所有脚本都在仓库根目录下执行，例如 `bash scripts/sim.sh status`。
分成四类：**日常使用**、**验收与录制**、**环境修补**、**开发期诊断**。
（最后两类一般不用手动跑，留在这里是为了出问题时能复现。）

---

## 一、日常使用

| 脚本 | 作用 | 典型用法 |
|---|---|---|
| `sim.sh` | 起/停/查 PX4 + Gazebo。`start` 里会先做内存与插件修补，`stop` 会硬清掉残留的 `gz sim` / `px4` 进程 | `bash scripts/sim.sh start aruco`，`bash scripts/sim.sh status`，`bash scripts/sim.sh stop` |
| `start_until_healthy.sh` | 反复起 SITL 直到 IMU 真的有发布者（规避偶发的"模型起了但传感器不发"） | `bash scripts/start_until_healthy.sh aruco 2` |
| `manual_demo.sh` | 手动演示的总入口：`start` 起任务链、`view` 看检测图、`gui` 开 3D 窗口、`teleop` 键盘遥控、`status` 查话题、`stop` 收尾 | `bash scripts/manual_demo.sh start` |
| `stop_ros.sh` | 停掉所有 ROS 节点（按进程名匹配，不用记 PID） | `bash scripts/stop_ros.sh` |
| `check_env.sh` | 依赖体检：ROS/Gazebo/PX4/OpenCV 版本与关键包是否齐全 | `bash scripts/check_env.sh` |

> 注意：不要在**同一条命令行**里既写 `ros2 launch ...` 又调用 `stop_ros.sh`,
> 因为它是按命令行字符串匹配杀进程的，会把自己那条 shell 一起杀掉。要写就写进脚本文件。

## 二、验收与录制

| 脚本 | 作用 | 典型用法 |
|---|---|---|
| `run_flow.sh` | 五套流程的统一入口，透传退出码（0 = PASS） | `bash scripts/run_flow.sh F2` |
| `dev_run_f1.sh` … `dev_run_f5.sh` | 单条流程的完整链路（起仿真 → 起 launch → 跑验收 → 收尾） | `bash scripts/dev_run_f4.sh` |
| `record_video.sh` | 跑一条流程并录成 mp4（离屏机位 + HUD + 机载小窗） | `bash scripts/record_video.sh F2 90 auto` |
| `demo_tour.sh` | 多工位巡回演示：基站 → 三个工位逐个降落 → 回基站，同时录像 | `bash scripts/demo_tour.sh 280` |
| `spawn_course.sh` | 往运行中的世界里摆三个工位码与演示机位（幂等） | `bash scripts/spawn_course.sh aruco` |
| `spawn_overhead_cam.sh` | 只摆一个正俯视机位（`record_video.sh` 用） | `bash scripts/spawn_overhead_cam.sh aruco` |
| `tour_watch.py` | 盯着 `/mission/status` 打印状态迁移，`TOUR_DONE` 退出 0 | `python3 scripts/tour_watch.py 320` |

## 三、环境修补（每台机器只需做一次）

| 脚本 | 作用 |
|---|---|
| `fix_gz_plugins.sh apply\|restore\|show` | 替换 PX4 的 `server.config`：去掉会按帧泄漏内存的 `GstCameraSystem`/`OpticalFlowSystem`，并把渲染引擎固定成 Ogre1 |
| `fix_gz_model.sh apply\|restore\|show` | 用展平的 `x500_mono_cam_down` 模型替换 PX4 的模型路径：绕开 gz 富模型 `merge=true` 丢传感器的 bug |
| `sync.sh` | 把 Windows 侧暂存目录同步进 WSL 工作区（`cp -u`，只覆盖更新的文件；`SYNC_FORCE=1` 强制） |

## 四、开发期诊断（排障用）

| 脚本 | 用途 |
|---|---|
| `probe_mem.sh [次数] [间隔]` | 采样 gz 的 RSS 与系统内存，判断有没有内存泄漏 |
| `probe_motors.sh` | 采样执行器指令（`command/motor_speed`），判断 PX4 有没有真的给电机出力 |
| `offboard_probe.py` | 直接用 rclpy 发 `/mavros/setpoint_raw/local`，验证位置/速度设定点是否生效 |
| `dev_offboard_probe.sh` | 起链路 + 跑一次 `offboard_probe.py` |
| `dev_m0_hover.sh` / `dev_run_hover.sh` | M0 时期的悬停测试 |
| `dev_run_aruco.sh` | 起链路 + 只跑检测节点（M1 精度验证用） |
| `m1_check_detection.py` | 等稳态悬停后采 15 s，统计检测 vs 真值的三维/水平/垂直误差 |
| `m1_detector_accuracy.py` | 把"检测误差"和"EKF 估计误差"分开量（输出偏置与扣除偏置后的误差） |
| `m1_frame_check.py` | 反复对比 `gz model -p` 与 `/mavros/local_position/pose`，量化两坐标系的偏置 |
| `m1_detect_offline.py` | 对一张静态图做离线检测（不需要仿真） |
| `m1_save_debug.py <png>` | 保存一张 `/aruco/debug_image` |
| `dev_run_det_probe.sh` / `dev_det_probe.py` | 造出水平偏移，逐帧打印检测位置与真实偏移（验相机轴系） |
| `dev_manual_probe.sh` | 发 `MANUAL` 起飞并用 `trace_alt.py` 记录高度 |
| `trace_alt.py [秒数]` | 10 Hz 记录高度与模式到 CSV，区分"真掉高"和"读数跳变" |
| `m5_probe.py` | 采样 `/mavros/setpoint_raw/local` 的 type_mask，用来抓"两个发布者抢设定点" |
| `camera_check.py` | 检查相机话题是否有数据、图像尺寸与内参 |
| `set_px4_param.py <名> <值>` | 用 rclpy 设 PX4 参数（等价 `ros2 param set`，但绕开 mavros 的服务坑） |
