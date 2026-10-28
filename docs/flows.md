# 五套验收流程（F1 ~ F5）

本文档是"怎么跑、判据是什么、现在什么结果"的唯一出处。所有流程都是一条命令，
结束时打印 `[FLOW Fx] RESULT: PASS` 或 `FAIL`，退出码 0/1。

```bash
bash scripts/run_flow.sh F2      # F1~F5 任意一个
```

每次运行都会：重启 SITL（含 IMU 健康检查）→ 起对应 launch（`use_sim_time:=true`，
桥接 `/clock`）→ 起验收脚本 → 收尾停掉全部进程并保留日志。

---

## F1 — 纯手动飞行

| 项 | 内容 |
| --- | --- |
| 入口 | `ros2 launch drone_bringup flow_f1.launch.py start_teleop:=true` |
| 验收 | `bash scripts/dev_run_f1.sh` |
| 操作 | `Space` 起飞 1.5 m → `w/a/s/d` 前后左右 → `r/f` 升降 → `q/e` 偏航 → `0` 降落 |
| 判据 | 起飞高度 > 1.2 m；最终高度 < 0.2 m；全程无 crash |
| 实测 | 起飞 1.95 m，飞行中最低 1.84 m，降落 7.7 s，最终 0.03 m → **PASS** |

## F2 — 手动转视觉降落

| 项 | 内容 |
| --- | --- |
| 入口 | `ros2 launch drone_bringup flow_f2.launch.py` |
| 验收 | `bash scripts/dev_run_f2.sh` |
| 操作 | `Space` 起飞 → 键盘飞到码附近 → 按 `2`（或发 `/mission/mode = ARUCO_LAND`） |
| 判据 | marker_detected = yes；最终水平误差 ≤ 0.25 m；最终高度 ≤ 0.25 m；全程 ≤ 150 s |
| 实测 | 0.81 m 接管 → IDLE→ALIGN→DESCEND→FINAL→DONE，落点水平 0.01 m / 高度 -0.01 m / 31.06 s → **PASS** |

## F3 — 纯自主航点

| 项 | 内容 |
| --- | --- |
| 入口 | `ros2 launch drone_bringup flow_f3.launch.py` |
| 验收 | `bash scripts/dev_run_f3.sh` |
| 航点 | `config/waypoints.yaml`：`(3,0,2) → (3,3,2) → (0,0,2)`（最后一点是码正上方，给 F4 接力） |
| 判据 | 每点稳态误差 ≤ 0.35 m；全程 ≤ 180 s |
| 实测 | 0.158 / 0.075 / 0.102 m，30.6 s，落点高度 -0.04 m → **PASS** |

## F4 — 全自动任务链路

| 项 | 内容 |
| --- | --- |
| 入口 | `ros2 launch drone_bringup flow_f4.launch.py` |
| 验收 | `bash scripts/dev_run_f4.sh`（`M5_PROBE=1` 同时录设定点） |
| 操作 | 只有一条 `AUTO` 模式请求（等价于按 `4`），之后零人工输入 |
| 判据 | 全程零人工输入；最终水平误差 ≤ 0.25 m；最终高度 ≤ 0.25 m；≤ 180 s |
| 实测 | `IDLE → AUTO_WAYPOINT → AUTO_LAND → AUTO_DONE`，42.5 s，落点水平 0.04 m / 高度 -0.03 m → **PASS** |

## F5 — 异常处理（下降中遮挡）

| 项 | 内容 |
| --- | --- |
| 入口 | `ros2 launch drone_bringup flow_f5.launch.py` |
| 验收 | `bash scripts/dev_run_f5.sh` |
| 故障注入 | 验收脚本在 Gazebo 侧用 `gz service /world/aruco/create` 在机体正下方生成一块 3 m × 3 m 黑色遮挡板，遮住相机 3.0 s 后撤除（真实遮挡，不是模拟丢帧） |
| 判据 | marker_occluded = yes；遮挡后高度回升 ≥ 1.0 m；无 crash；最终水平误差 ≤ 0.25 m；最终高度 ≤ 0.25 m；≤ 240 s |
| 实测 | 遮挡期间最长无检测 3.00 s，高度 1.85 m → 3.72 m（回升 1.87 m），之后重新对准降落，落点 0.01 m / 0.02 m，47.0 s → **PASS** |

---

## 判据是怎么算出来的（不是"看着像成功"）

- **高度**：`/mavros/local_position/pose` 的 ENU `z`；姿态同理取 `map` 系。
- **水平误差**：`/aruco/detection` 与无人机位姿之差（同一时刻），末值取"最后一帧检测"。
  若一帧都没检测到，直接判 FAIL —— 不允许用哨兵值 `-1` 蒙过 `<= 0.25` 的比较。
- **遮挡是否真生效**：不看"帧数是否为 0"（板子加载有延迟，开头几帧必然还在），
  而是看"最长连续无检测间隔 ≥ 60% 遮挡时长"。
- **crash**：高度低于 0.5 m 且持续 1 s、且不处于 `FINAL/TOUCHDOWN/DONE` 阶段。
  正常降落必然穿越 0.5 m，早先版本没有这个限定，把所有成功降落都判成了撞地。
- **超时**：每个流程一个总预算（150/180/240 s），超时即 FAIL 并打印卡在哪个阶段。

## 每个阶段的日志位置

| 文件 | 内容 |
| --- | --- |
| `/tmp/f1~f5_bringup.log` | 该次运行所有节点的完整输出（含状态机切换） |
| `/tmp/f5_*.log` | F5 的验收脚本输出（遮挡时间线） |
| `/tmp/f4_probe.log` | `M5_PROBE=1` 时的设定点采样（查"谁在发设定点"） |
| `logs/px4_sitl.log` | PX4 侧日志（arm / failsafe / 模式切换） |
