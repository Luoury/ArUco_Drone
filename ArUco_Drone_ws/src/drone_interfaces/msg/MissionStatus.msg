# 任务状态（M0 起定义，M4 起由 mission_manager 发布，10 Hz）
builtin_interfaces/Time stamp

# 状态机状态：IDLE / SEARCH / ALIGN / DESCEND / FINAL / TOUCHDOWN / LAND
string state
# 当前飞行模式（MAVROS custom_mode，例如 OFFBOARD / AUTO.LAND）
string mode
# 当前流程标识：MANUAL / F1 / F2 / F3 / F4 / F5
string flow

# 相对起飞点的高度，ENU z（向上为正），单位 m
float32 altitude
# 水平误差（相对目标或码中心），单位 m
float32 lateral_error
# 垂直速度（向上为正），单位 m/s
float32 vertical_speed

# 是否看到 ArUco 码
bool marker_visible
# 是否已解锁
bool armed
