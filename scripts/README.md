# 脚本说明

所有脚本都在**仓库根目录**执行：

```bash
bash scripts/sim.sh start aruco
```

---

## 一、起停与检查

| 脚本 | 作用 | 用法 |
|---|---|---|
| `sim.sh` | 起 / 停 / 查 PX4 + Gazebo。`start` 里会先做模型与插件修补，`stop` 会清掉残留进程 | `bash scripts/sim.sh start aruco` · `status` · `stop` |
| `start_until_healthy.sh` | 反复起 SITL，直到 IMU 真的在发布（规避偶发的"模型起了但传感器不发"） | `bash scripts/start_until_healthy.sh aruco 2` |
| `stop_ros.sh` | 停掉全部 ROS 节点（按进程名匹配） | `bash scripts/stop_ros.sh` |
| `check_env.sh` | 依赖体检：ROS / Gazebo / PX4 / OpenCV 版本与关键包 | `bash scripts/check_env.sh` |

> 注意：不要在**同一条命令行**里既写 `ros2 launch ...` 又调用 `stop_ros.sh`，
> 它按命令行字符串匹配进程，会把自己这条 shell 一起杀掉。写进脚本文件执行即可。

## 二、手动操作

| 脚本 | 作用 |
|---|---|
| `manual_demo.sh start` | 起任务链：检测 + 任务调度 + 降落控制器 |
| `manual_demo.sh view` | 打开检测叠加图（角点框、坐标轴、距离、帧号） |
| `manual_demo.sh gui` | 打开 Gazebo 3D 窗口 |
| `manual_demo.sh teleop` | 键盘遥控（另开终端） |
| `manual_demo.sh status` | 查仿真状态与各话题发布者 |
| `manual_demo.sh stop` | 停任务链并关闭窗口 |

## 三、五套流程与录制

| 脚本 | 作用 | 用法 |
|---|---|---|
| `run_flow.sh` | 五套流程统一入口，透传退出码（0 = PASS） | `bash scripts/run_flow.sh F2` |
| `run_f1.sh` … `run_f5.sh` | 单条流程的完整链路：起仿真 → 起 launch → 自动验收 → 收尾 | `bash scripts/run_f4.sh` |
| `record_video.sh` | 跑一条流程并录成 mp4（离屏机位 + HUD + 机载小窗） | `bash scripts/record_video.sh F2 90 auto` |
| `demo_tour.sh` | 多工位巡回演示：基站 → 三个工位逐个降落 → 回基站，同时录像 | `bash scripts/demo_tour.sh 280` |
| `tour_watch.py` | 盯 `/mission/status` 打印状态迁移，`TOUR_DONE` 退出 0 | `python3 scripts/tour_watch.py 320` |
| `spawn_course.sh` | 往运行中的世界里摆三个工位码 + 演示机位（幂等） | `bash scripts/spawn_course.sh aruco` |
| `spawn_overhead_cam.sh` | 只摆一个正俯视机位（录制用） | `bash scripts/spawn_overhead_cam.sh aruco` |

## 四、环境修补与参数

| 脚本 | 作用 |
|---|---|
| `fix_gz_plugins.sh apply\|restore\|show` | 替换 PX4 的 `server.config`：去掉会按帧泄漏内存的 `GstCameraSystem` / `OpticalFlowSystem`，并把渲染引擎固定为 Ogre1 |
| `fix_gz_model.sh apply\|restore\|show` | 用展平后的机体模型替换 PX4 的模型路径，绕开 gz 富模型 `merge=true` 丢传感器的 bug |
| `set_px4_param.py <名> <值>` | 用 rclpy 设置 PX4 参数（绕开 mavros 的服务差异） |

前两个脚本只在**仿真**环境需要，实机不必执行；它们都会先备份再修改，`restore` 可还原。

## 五、诊断工具（`scripts/diagnostics/`）

调试与标定用的一次性工具，正常使用不需要。清单与用途见
[`scripts/diagnostics/README.md`](diagnostics/README.md)。
