#!/usr/bin/env python3
"""直接向 /mavros/setpoint_raw/local 发 OFFBOARD 设定点，用于诊断"为什么不起飞"。

用法:
  offboard_probe.py pos 1.5            # 位置设定点，ENU 高度 1.5 m
  offboard_probe.py vel 1.0            # 速度设定点，ENU 向上 1.0 m/s
  offboard_probe.py pos 1.5 --yaw-valid # 位置 + 有效 yaw（清掉 type_mask 的 bit10）

会先预发 2 s、再切 OFFBOARD、再解锁，然后打印 10 s 的高度变化。
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

MASK_POS = 0b0000111111111000          # 位置有效，yaw/yaw_rate 都忽略
MASK_POS_YAW = 0b0000111111110000      # 位置 + yaw 有效（清 bit10）
MASK_VEL = 0b0000111111000111          # 速度 + yaw_rate 有效


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    mode = args[0] if args else "pos"
    target = float(args[1]) if len(args) > 1 else 1.5
    yaw_valid = "--yaw-valid" in sys.argv
    xy_only = "--xy" in sys.argv
    airborne = "--airborne" in sys.argv
    enu = "--enu" in sys.argv
    arm_first = "--arm-first" in sys.argv

    rclpy.init()
    node = Node("offboard_probe")
    pub = node.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
    pose = {"x": 0.0, "y": 0.0, "z": 0.0}
    st = {"armed": False, "mode": ""}

    def on_state(msg: State):
        st["armed"] = msg.armed
        st["mode"] = msg.mode

    def on_pose(msg: PoseStamped):
        pose["x"] = msg.pose.position.x
        pose["y"] = msg.pose.position.y
        pose["z"] = msg.pose.position.z

    # mavros 的 pose 发布是 BEST_EFFORT，订阅端必须匹配，否则一条都收不到
    pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", on_pose, pose_qos)
    node.create_subscription(State, "/mavros/state", on_state, 10)
    cli_mode = node.create_client(SetMode, "/mavros/set_mode")
    cli_arm = node.create_client(CommandBool, "/mavros/cmd/arming")
    for c in (cli_mode, cli_arm):
        c.wait_for_service(timeout_sec=10.0)

    def spin(sec: float):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def publish(dur: float):
        end = time.time() + dur
        while time.time() < end:
            msg = PositionTarget()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.header.stamp = node.get_clock().now().to_msg()
            if mode == "vel":
                msg.type_mask = MASK_VEL
                msg.velocity.x = 0.0
                msg.velocity.y = 0.0
                msg.velocity.z = target if enu else -target
                msg.yaw_rate = 0.0
            else:
                msg.type_mask = MASK_POS_YAW if yaw_valid else MASK_POS
                if xy_only:
                    # 只测水平：ENU x = target，z 保持 0
                    msg.position.x = target if enu else 0.0
                    msg.position.y = 0.0 if enu else target
                    msg.position.z = 0.0
                else:
                    msg.position.x = 0.0
                    msg.position.y = 0.0
                    msg.position.z = target if enu else -target
                if yaw_valid:
                    msg.yaw = 0.0
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)

    # 先等到"已上锁"，否则上一次飞行的残留状态会让 arming 被拒
    t_wait = time.time()
    while st["armed"] and time.time() - t_wait < 25.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if airborne:
        # 先用 PX4 自己的 AUTO.TAKEOFF 离地，再交给 OFFBOARD —— 用来隔离"从地面 OFFBOARD 起飞"这个变量
        req0 = SetMode.Request()
        req0.custom_mode = "AUTO.TAKEOFF"
        fut0 = cli_mode.call_async(req0)
        rclpy.spin_until_future_complete(node, fut0, timeout_sec=5.0)
        arm0 = CommandBool.Request()
        arm0.value = True
        fut0 = cli_arm.call_async(arm0)
        rclpy.spin_until_future_complete(node, fut0, timeout_sec=5.0)
        t_up = time.time()
        while pose["z"] < target + 0.4 and time.time() - t_up < 25.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"[probe] AUTO.TAKEOFF 后 alt={pose['z']:.2f} armed={st['armed']} mode={st['mode']}")
    print(f"[probe] 起始状态: armed={st['armed']} mode={st['mode']}")
    print(f"[probe] mode={mode} target={target} yaw_valid={yaw_valid} enu={enu}")
    print(f"[probe] 起始 pose: x={pose['x']:.2f} y={pose['y']:.2f} z={pose['z']:.2f}")
    publish(2.0)

    def do_arm():
        arm = CommandBool.Request()
        arm.value = True
        f = cli_arm.call_async(arm)
        rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        print(f"[probe] arming: {f.result()}  armed={st['armed']} mode={st['mode']}")

    def do_offboard():
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        f = cli_mode.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        print(f"[probe] OFFBOARD 请求: {f.result()}")

    if arm_first:
        # 先回到 AUTO.LOITER 再解锁，再切 OFFBOARD —— 用来验证"顺序"是否是地面起飞失败的原因
        req_l = SetMode.Request()
        req_l.custom_mode = "AUTO.LOITER"
        f_l = cli_mode.call_async(req_l)
        rclpy.spin_until_future_complete(node, f_l, timeout_sec=5.0)
        spin(1.0)
        do_arm()
        spin(1.0)
        do_offboard()
        spin(1.0)
    else:
        do_offboard()
        do_arm()
    print(f"[probe] 进入设定点保持阶段: armed={st['armed']} mode={st['mode']}")

    t0 = time.time()
    while time.time() - t0 < 10.0:
        publish(0.5)
        print(f"[probe] t={time.time() - t0:4.1f}s  alt={pose['z']:5.2f} m")
    print(f"[probe] 结束 pose: x={pose['x']:.2f} y={pose['y']:.2f} z={pose['z']:.2f}")

    req2 = SetMode.Request()
    req2.custom_mode = "AUTO.LAND"
    fut2 = cli_mode.call_async(req2)
    rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
    print(f"[probe] AUTO.LAND: {fut2.result()}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
