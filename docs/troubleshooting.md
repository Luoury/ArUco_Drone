# 踩坑记录（全部为本机实测结论，含现场证据）

> 本机环境：Windows 11 + WSL2(Ubuntu 22.04) / PX4 v1.17.0-alpha1-338-gdf42ef84f1 /
> Gazebo Sim 8.10.0（Harmonic）/ ROS 2 Humble / mavros 2.14 / 无 `/dev/dri`（软件渲染）。

## 1. `ros_gz_bridge` 与 Gazebo 8 不是一个发现域（最隐蔽）

- **现象**：`parameter_bridge` 起得来，`ros2 node info` 显示它确实发布了
  `/camera/image_raw`，但订阅者永远收不到数据；`gz topic -i -t <image>` 里
  `No subscribers on topic`；`/clock` 同样一帧都不通。
- **原因**：apt 的 `ros-humble-ros-gz-bridge 0.244.20` 链接的是 Fortress ABI
  （`libignition-transport11` / `libignition-msgs8`），而 gz sim 8.10 进程用的是
  `libgz-transport13`。两者不在同一个传输发现域，订阅静默失败（不报错）。
- **判定命令**：`ldd /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge | grep ignition`
  对比 `grep gz-transport /proc/$(pgrep -f "gz sim" | head -1)/maps`。
- **解决**：本仓库自建 `drone_bringup/src/gz_sensor_bridge.cpp`，直接用
  `gz-transport13` + `gz-msgs10` 订阅，再发 ROS 话题。`ros_gz_image` 的
  `image_bridge` 同样是 Fortress 版，一样不通。

## 2. PX4 拉起的 gz server 内存线性泄漏（会把 WSL 拖死）

- **现象**：gz sim 的 RSS 以约 105 MB/s 增长（`t=20s 680MB → 80s 6981MB`），
  随后整个 WSL2 卡死；`/mavros/state` 每 ~9 s 掉线一次。
- **原因**：PX4 的 `server.config` 默认加载 `custom::GstCameraSystem`，它用正则订阅
  世界里的**所有**相机图像并灌进 GStreamer，缓冲不释放。泄漏与分辨率、阴影、
  机体是否运动都无关，只在"PX4 拉起 gz"时出现。
- **注意**：`px4-rc.gzsim` 会自己 `source ./gz_env.sh`，而 gz_env.sh 把
  `GZ_SIM_SERVER_CONFIG_PATH` 指回 PX4 的清单，**外部设这个变量无效**。
- **解决**：`scripts/fix_gz_plugins.sh apply` 用自己的清单覆盖 PX4 的
  `server.config`（去掉 GstCameraSystem / OpticalFlowSystem，把传感器系统锁成
  `<render_engine>ogre</render_engine>`）；`restore` 可还原。`sim.sh start` 会自动 apply。
  修复后 RSS 恒定 470 MB、RTF ≈ 1.0。

## 3. Ogre2 软件渲染下相机把仿真拖到 1/180 实时

- **现象**：带相机时 `/world/aruco/stats` 的 `real_time_factor` 只有 0.0056，
  PX4 主循环被拖死，mavros 反复掉线。
- **对照**：无相机的 `gz_x500` + aruco world → RTF 0.9956；加相机 + 默认 Ogre2 →
  0.0056；加 `PX4_GZ_SIM_RENDER_ENGINE=ogre`（Ogre1）→ 0.9816。
- **解决**：`sim.sh` 固定 `HEADLESS=1 PX4_GZ_SIM_RENDER_ENGINE=ogre`。

## 4. `merge=true` 把 x500 的传感器全丢了

- **现象**：`Preflight Fail: Accel Sensor 0 missing` / `Gyro` / `barometer` /
  `Found 0 compass`；`gz topic -i` 里 IMU 只有 Subscribers 没有 Publishers；
  场景快照里 `x500_mono_cam_down_0` 只有相机，没有 base_link 的四个传感器。
- **原因**：`gz sdf -p` 展平后的 SDF 里传感器是齐的（`grep -c '<sensor '` = 5），
  但 gz sim 运行时按 `merge=true` 合并 x500_base 时把它们丢了（只在运行时可见）。
- **解决**：`scripts/fix_gz_model.sh apply`：把展平后的模型写进
  `~/.cache/gz_models/x500_mono_cam_down/model.sdf`，其余模型软链过去，并把
  `gz_env.sh` 的 `PX4_GZ_MODELS` 指向该目录。修复后传感器 5 个齐全、IMU 有发布者。
- **排查技巧**：`gz model -p` 只看位姿；要看传感器请用
  `gz service -s /world/<w>/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene -r ""`
  （**必须带 `-r ""`**，否则只打印用法）。

## 5. ENU→NED 不能转两次

- `/mavros/setpoint_raw/local` 的位置/速度字段就是 **ENU**，mavros 会自己做
  ENU→NED（`coordinate_frame=MAV_FRAME_LOCAL_NED`）。如果应用层再写
  `x=enu.y, y=enu.x, z=-enu.z`，目标会变成"地下 1.5 m"。
- **实测**：手工转的那次，无人机从 1.28 m 一路扎到 −0.09 m；改成直接发 ENU 后
  正常收敛到 1.41 m。

## 6. 本机 OFFBOARD 不能从地面起飞

- **现象**：地面状态下位置/速度设定点都只让电机停在 151 rad/s（怠速；x500 悬停约需
  657 rad/s），PX4 打 `Disarmed by auto preflight disarming`。
- **解决**：用 `AUTO.TAKEOFF` 起飞，离地后再按需求书 §5.3 顺序交 OFFBOARD
  （`FlightControl::takeoffTo()` 已封装）。

## 7. 解锁被拒：`NAV_DLL_ACT`

- `commander` 打 `Preflight Fail: No connection to the GCS` 并拒绝 arm，是
  `rcAndDataLinkCheck.cpp` 里 `NAV_DLL_ACT > 0` 导致的。设 `NAV_DLL_ACT=0` 后
  立刻 `Ready for takeoff!`。本机已写进 PX4 的 `parameters.bson`（重启仍生效），
  换机器要重设：
  `python3 scripts/set_px4_param.py NAV_DLL_ACT 0`
  （`/mavros/param/get` 不存在；`/mavros/param/set` 的类型是 `ParamSetV2`。）

## 8. 不要给 mavros 节点指定 `name='mavros'`

- 会把节点名强改成 `mavros`，与内部 router/uas 子节点撞名：
  `create_subscription() called for existing topic name rt/mavros/mavros/status with
  incompatible type ...` → `terminate called after throwing 'rclcpp::exceptions::RCLError'`
  （exit code -6）。launch 里去掉 `name=` 即可。

## 9. `ros2` CLI 的 daemon 会给出陈旧/空结果

- 同一时刻 `ros2 topic list` 只列 `/parameter_events` `/rosout`，加 `--no-daemon`
  才是真实列表。
- 支持面：`--no-daemon` 只有 `topic list/echo/info` 认；`topic hz`、`service call`
  不认并直接报 `unrecognized arguments`。带 daemon 跑 `service call` 会报
  `rcl node's context is invalid`。
- **可靠做法**：用 rclpy 写探针（本仓库 `scripts/*.py` 都是这么写的），或用
  `scripts/set_px4_param.py` 这类封装。

## 10. 3D 视觉库与 pip numpy 的 ABI 冲突

- `cv_bridge.imgmsg_to_cv2` 在本机会 `AttributeError: _ARRAY_API not found` 然后
  `Segmentation fault`。改用
  `np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)`。
- 也不要再装 pip 版 opencv 到系统 python 去喂 rclpy；C++ 侧用系统 OpenCV 4.5.4
  （`libopencv_aruco.so` 自带 aruco 模块）即可。

## 11. launch 里的默认值会覆盖 yaml

- `aruco_detect.launch.py` 曾把 `marker_size` 的 `DeclareLaunchArgument` 默认值
  0.50 塞进 Node 参数，把 `aruco.yaml` 里标定好的 0.487 覆盖掉，导致"改了 yaml
  却不生效"。**规则**：标定值只放在 yaml，launch 只暴露运行期开关。

## 12. 位姿时间戳是墙钟，图像时间戳是仿真钟

- `/mavros/local_position/pose` 的 header 是墙钟（约 1.79e9 s），gz 图像是仿真时间
  （几百秒）。按时间戳把两者配对，永远会挑到缓冲里最旧的位姿（实测误差从 0.03
  恶化到 0.13 m）。**做法**：取"最新位姿"，并给共享状态加锁。

## 13. gz 孤儿进程会让新 PX4 挂到空世界

- 上一轮 PX4 被杀但 gz 存活时，新 PX4 会打
  `INFO [init] gazebo already running world: aruco` 并复用那个没有无人机的旧世界，
  表现为 `MAVROS 未连接`、检测 0 帧。
- `sim.sh` 的 `hard_clean()` 会扫 `/proc/*/cmdline` 精确匹配 `gz sim` 与
  `bin/px4 -d` 后 kill -9，`start()`/`stop()` 都会调用。

## 14. WSL 里跑编译要限并行度

- `colcon build` 默认 24 路并行会把内存打满并让 WSL 失去响应。用
  `MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 1 --packages-select <pkg>`。
- 编译前必须 `source /opt/ros/humble/setup.bash`，否则
  `ModuleNotFoundError: No module named 'ament_package'`。
- 脚本里不要开 `set -u`：`source /opt/ros/humble/setup.bash` 会因为
  `AMENT_TRACE_SETUP_FILES` 未定义而直接退出。

## 15. 开发机（Windows + WSL）操作注意

- 从 MSYS bash 调进 WSL 时，以 `/home/...` 开头的参数会被改写成 Windows 路径，
  命令前加 `MSYS_NO_PATHCONV=1`。
- `wsl ... bash -lc '...'` 中间那层解析会吃掉 `$VAR` 与 `$(...)`，变量要写进脚本文件。
- `pkill -f <模式>` / `pgrep -f <模式>` 若模式字面量出现在当前命令行里，会把自己
  那条 shell 一起杀掉（表现为没有任何输出、exit 0）。清理逻辑一律写进脚本。
