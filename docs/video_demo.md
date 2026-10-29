# 录制演示视频（不依赖任何窗口）

> 一条命令，产出一个 mp4：俯视机位画面 + HUD 数据条 + 右下角机载检测小窗。
> 录制过程完全不碰 WSLg 的窗口，所以不会因为窗口卡住而毁掉整段视频。

---

## 1. 为什么不用录屏

WSLg 把 Linux 窗口的画面经"远程桌面协议"贴回 Windows。Gazebo 只要连续刷新画面，
这个合成通道就容易 stalled —— 表现是窗口里 3D 视口不动了，但右键菜单、属性面板还在响应。
录屏软件录到的就是那幅冻住的画面。

这条管线换了个思路：**相机放进仿真里**，画面从 ROS 话题拿，再由 Python 编码成 mp4。
帧率恒定、可重复、还能叠加 HUD，跟窗口卡不卡完全无关。

```
gz 俯视相机 ──gz-transport──▶ gz_sensor_bridge ──▶ /camera/overhead/image_raw
                                                      │
任务链 /mission/status ────────────────────────────────┤
机载检测 /aruco/debug_image ───────────────────────────┤
                                                      ▼
                                            record_video（Python）
                                                      │
                                                      ▼
                                        logs/videos/<流程>_<时分秒>.mp4
```

---

## 2. 一条命令

```bash
cd ~/ArUco_Drone
bash scripts/record_video.sh F2 70 auto
```

| 参数 | 含义 | 默认 |
|------|------|------|
| 第 1 个 | 流程：`F1` / `F2` / `F3` / `F4` / `F5` | `F2` |
| 第 2 个 | 录制多少秒 | `70` |
| 第 3 个 | `auto` 录制期间自动跑该流程验收；`manual` 只起仿真+录制，你自己用 teleop 飞 | `auto` |

脚本会依次：清进程 → 起 SITL（含 IMU 健康检查）→ 挂俯视机位 → 起流程（`start_overhead:=true`）
→ 起录制 → 跑验收 → 等录制收尾 → 收尾进程。全程约 3~4 分钟（SITL 启动占大头）。

产物：`logs/videos/F2_125848.mp4`（名字里的数字是当前时分秒）。

实测输出：

```
[record] 流程=F2 时长=70s 模式=auto 输出=/home/luosury/ArUco_Drone/logs/videos/F2_125848.mp4
[healthy] 第 1 次启动成功：IMU 有发布者
[overhead] 就位：/world/aruco/model/overhead_cam/link/link/sensor/camera/image 已有发布者
[record] 视频已生成：/home/luosury/ArUco_Drone/logs/videos/F2_125848.mp4（6.5M）
录制结束：写入 1916 帧 / 85.2 s（22.5 fps），收到 698 帧、过快丢弃 0 帧，6.7 MB
[FLOW F2] landed = yes（23.7 s），final_altitude = 0.04 m，final_lateral = 0.03 m，elapsed = 40.3 s
[FLOW F2] RESULT: PASS
```

视频规格：**960×540，25 fps，mp4v 编码**（本机没装 ffmpeg，所以不用 H.264；`mp4v` 免依赖、能直接用）。

---

## 3. 画面里有什么

抽帧（F2 降落中段，`t=34.7 s`）：

- **主画面**：俯视机位（正下方 6 m、视场 1.05 rad），无人机压在黑色码板上，能看清下降、对准、落地；
- **左上 HUD**：`t=34.7s mode=ARUCO_LAND`、`ALT 1.19 m  lateral_err 0.04 m`、`marker VISIBLE  ARMED`（丢码时 marker 变红）；
- **右下游标窗**：`/aruco/debug_image`，即机载下视相机加检测叠加（绿色角点框 + RGB 坐标轴 + `id=0 z_cam=1.18m`）。

> HUD 里 `mode` 来自 `/mission/status.mode`，`ALT`/`lateral_err` 也来自 `MissionStatus`；
> 小窗来自 `/aruco/debug_image`。想关掉小窗：`-p use_inset:=false`。

---

## 4. 分步做（不想用一条命令时）

```bash
# ① 起仿真（约 60~90 s）
bash scripts/sim.sh start aruco

# ② 挂俯视机位（幂等，可重复执行）
bash scripts/spawn_overhead_cam.sh aruco

# ③ 起流程，注意 start_overhead:=true 才会桥接俯视图像
ros2 launch drone_bringup flow_f2.launch.py start_overhead:=true

# ④ 录制（另开一个终端）
ros2 run drone_tools record_video --ros-args \
  -p output:=$HOME/ArUco_Drone/logs/videos/manual.mp4 \
  -p duration:=60.0 -p fps:=25.0 -p title:="F2 手动录制"
```

`record_video` 的常用参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `output` | `~/ArUco_Drone/logs/videos/record.mp4` | 输出文件（目录不存在会自动建） |
| `duration` | `0.0` | 录制秒数，`0` = 一直录到 Ctrl+C |
| `fps` | `25.0` | 输出帧率。相机实际到帧率会抖，录制器按"墙上时间"补帧/丢帧，**视频时长≈真实时长** |
| `image_topic` | `/camera/overhead/image_raw` | 主画面话题，可换成 `/camera/image_raw` 录机载视角 |
| `use_inset` | `true` | 是否叠加右下角检测小窗 |
| `inset_width` | `320` | 小窗宽度（像素） |
| `title` | `ArUco 自主降落` | HUD 第一行文字 |
| `stall_warn` | `2.0` | 超过这么多秒没画面就在日志里告警 |

---

## 5. 俯视机位怎么调

机位是**世界里的一个静态模型**，文件：`ArUco_Drone_ws/src/drone_bringup/models/overhead_cam/model.sdf`。

```xml
<model name="overhead_cam">
  <static>true</static>
  <pose>0 0 6.0 0 1.5708 0</pose>          <!-- x y z roll pitch yaw，pitch=90° 让光轴朝下 -->
  <link name="link">
    <sensor name="camera" type="camera">
      <horizontal_fov>1.05</horizontal_fov>
      <image><width>960</width><height>540</height></image>
      <update_rate>10</update_rate>
```

改完必须**重新 sync 并重启仿真**（模型是启动时从文件读的）：

```bash
# 改 Windows 侧暂存文件后
bash scripts/sync.sh
bash scripts/sim.sh stop && bash scripts/sim.sh start aruco
```

覆盖范围 ≈ `2 × 高度 × tan(fov/2)`。当前 `6.0 m / 1.05 rad` ≈ ±3.5 m，
码在画面中央够大，航点 (3,0)/(3,3) 会在边缘甚至短暂出画。

> **分辨率/帧率别乱加**：实测把俯视相机配成 `1280×720@30` 后，机载主相机的出图率被拖到 ~5 Hz，
> 检测只有 4.6 Hz，F2 直接 FAIL（ALIGN 阶段反复丢码）。降到 `960×540@10` 后主相机回到
> 26~28 Hz、俯视 8.6~9.6 Hz，F2 恢复 PASS。软件渲染（没有 `/dev/dri`）下，
> **两台相机是抢同一个渲染器的**，加相机就要降配。

---

## 6. 排错

| 现象 | 原因 | 处理 |
|------|------|------|
| `[overhead] 失败：世界里查不到…` | `gz service create` 的请求串用了单引号 | 字符串值必须双引号，已经是脚本现状；自己手敲时注意 |
| 视频只有黑屏/HUD | 主画面话题没数据 | `ros2 topic info /camera/overhead/image_raw --no-daemon` 看有没有发布者；确认 launch 带了 `start_overhead:=true` |
| 日志 `超过 2.0 s 没有收到图像帧` | 桥接没起 / 相机没挂上 | 先 `bash scripts/spawn_overhead_cam.sh aruco`，再确认桥接日志里有"俯视机位" |
| `InvalidParameterTypeException ... '85' of type 'INTEGER'` | `-p duration:=85` 传了整数 | 传 `85.0`，或按本文用脚本（已处理） |
| 视频像快放 | 录制器没做帧节奏对齐（老版本） | 现版本按墙上时间补帧，vidéo 时长≈真实时长；确认 `录制结束：写入 N 帧` 中 N/时长≈fps |
| 想连窗口一起录（比如要展示 Gazebo 界面） | 窗口可能卡 | 见下节，"看画面"用 rqt，不要录屏 |

---

## 7. 与"看画面"的分工

- **要交互地看** → `docs/manual_test.md`：`rqt_image_view` 看检测画面、`gz sim -g` 看 3D 世界、
  MobaXterm 的 X server 也能跑 3D 窗口（实测可渲染，见该文第 6 节）。
- **要留证据/交作业** → 本文这条管线，一条命令出 mp4。

两者可以同时开：录制用的是话题，你开不开窗口都不影响它。
