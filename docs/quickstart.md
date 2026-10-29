# 一页纸：怎么跑起来

只有三步。全部命令都在仓库根目录（`~/ArUco_Drone`）执行。

---

## 1. 起仿真（约 60 s）

```bash
bash scripts/sim.sh start aruco
```

看到 `[sim] PX4 就绪 (pid=xxxx)` 就好了。
（顺带确认 IMU 正常：`bash scripts/sim.sh status`）

## 2. 起任务链

三个玩法任选其一：

```bash
bash scripts/manual_demo.sh start      # 手动 + 视觉降落（最常用）
bash scripts/demo_tour.sh              # 多工位巡回 + 自动录像
bash scripts/run_flow.sh F4            # 全自主一条龙 + 自动验收
```

## 3. 看画面 / 操控

```bash
bash scripts/manual_demo.sh view       # 看检测叠加图（有码的角点框和坐标轴）
bash scripts/manual_demo.sh gui        # 看 Gazebo 3D 窗口
bash scripts/manual_demo.sh teleop     # 键盘遥控（另开一个终端）
```

### 键位

| 键 | 作用 | 键 | 作用 |
|---|---|---|---|
| `Space` | 起飞 1.5 m / 已起飞则降落 | `1` | 手动模式 |
| `w` / `s` | 前进 / 后退 | `2` | 视觉降落 |
| `a` / `d` | 左移 / 右移 | `3` | 航点飞行 |
| `r` / `f` | 上升 / 下降 | `4` | 全自主 |
| `q` / `e` | 左转 / 右转 | `0` | 降落 |
| `h` | 打印帮助 | `ESC` | 紧急降落（立即上锁） |

## 收尾

```bash
bash scripts/manual_demo.sh stop       # 停任务链 + 关窗口
bash scripts/sim.sh stop               # 停仿真
```

---

## 常见情况

| 现象 | 处理 |
|---|---|
| `bash: 找不到命令` | 先 `source /opt/ros/humble/setup.bash`，再 `source ArUco_Drone_ws/install/setup.bash` |
| 检测图一直 `no marker` | 无人机还在地面，`Space` 起飞到 2 m 就能看到码 |
| 3D 窗口卡住/灰屏 | 这是 WSL2 软件渲染的老问题，用 `view` 看图像，或用 `record_video.sh` 离屏录制 |
| 起飞后不落 | 先按 `2`（视觉降落）；还不行按 `0`（普通降落） |
| 整个机器卡死 | 只允许一个仿真在跑：`bash scripts/stop_ros.sh && bash scripts/sim.sh stop` 后再起 |

更多：脚本清单 [../scripts/README.md](../scripts/README.md)、
每步原理 [manual_test.md](manual_test.md)、踩坑记录 [troubleshooting.md](troubleshooting.md)。
