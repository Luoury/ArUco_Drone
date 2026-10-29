# 诊断与标定工具

这些脚本是开发过程中用来定位问题和标定参数的，**正常运行不需要**。
每个都对应一个具体的排查场景，出问题时可以直接拿来复现。

## 标定与精度

| 脚本 | 用途 |
|---|---|
| `m1_check_detection.py` | 等机体稳态悬停后采样 15 s，统计检测位姿与真值的三维 / 水平 / 垂直误差 |
| `m1_detector_accuracy.py` | 把"检测误差"与"EKF 估计误差"分开统计（输出偏置与扣除偏置后的误差） |
| `m1_frame_check.py` | 反复对比 `gz model -p` 与 `/mavros/local_position/pose`，量化两坐标系的偏置 |
| `m1_detect_offline.py` | 对一张静态图片做离线检测，不依赖仿真（移植到别的机器上测算力也用它） |
| `m1_save_debug.py` | 保存一张 `/aruco/debug_image` 到文件 |

## 设定点与执行器

| 脚本 | 用途 |
|---|---|
| `offboard_probe.py` | 直接用 rclpy 发 `/mavros/setpoint_raw/local`，验证位置 / 速度设定点是否生效 |
| `dev_offboard_probe.sh` | 起链路并跑一次 `offboard_probe.py` |
| `m5_probe.py` | 采样设定点的 `type_mask`，用于发现"两个节点同时发设定点" |
| `probe_motors.sh` | 采样执行器指令（`command/motor_speed`），判断 PX4 是否真的在给电机出力 |
| `trace_alt.py` | 10 Hz 记录高度与飞行模式到 CSV，区分"真掉高"和"读数跳变" |

## 链路与内存

| 脚本 | 用途 |
|---|---|
| `probe_mem.sh` | 采样 Gazebo 进程的 RSS 与系统内存，判断是否存在内存泄漏 |
| `camera_check.py` | 检查相机话题是否有数据、图像尺寸与内参是否正确 |
| `dev_run_hover.sh` | 在已就绪的仿真上跑一次 `hover_test`（起飞 1.5 m 悬停 30 s） |
| `dev_run_aruco.sh` | 起链路但只跑检测节点（标定检测精度用） |
| `dev_manual_probe.sh` | 切 `MANUAL` 起飞并用 `trace_alt.py` 记录高度 |
| `dev_det_probe.py` / `dev_run_det_probe.sh` | 造出水平偏移，逐帧打印检测位置与真实偏移（验证相机轴系） |
