#!/usr/bin/env python3
"""诊断：PX4 自己能不能把这架飞机从地面飞起来（AUTO.TAKEOFF），
用来区分"整机/物理/估计器有问题"还是"只有 OFFBOARD 从地面起飞不工作"。
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

MODE = sys.argv[1] if len(sys.argv) > 1 else "AUTO.TAKEOFF"


def main() -> int:
    rclpy.init()
    node = Node("auto_takeoff_probe")
    pose = {"z": 0.0}
    st = {"armed": False, "mode": ""}

    pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                             lambda m: pose.__setitem__("z", m.pose.position.z), pose_qos)

    def on_state(msg: State):
        st["armed"] = msg.armed
        st["mode"] = msg.mode

    node.create_subscription(State, "/mavros/state", on_state, 10)
    cli_mode = node.create_client(SetMode, "/mavros/set_mode")
    cli_arm = node.create_client(CommandBool, "/mavros/cmd/arming")
    for c in (cli_mode, cli_arm):
        c.wait_for_service(timeout_sec=10.0)

    def spin(sec: float):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin(2.0)
    print(f"[auto_takeoff] 起始: armed={st['armed']} mode={st['mode']} alt={pose['z']:.2f}")

    req = SetMode.Request()
    req.custom_mode = MODE
    fut = cli_mode.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    print(f"[auto_takeoff] SetMode({MODE}): {fut.result()}")

    arm = CommandBool.Request()
    arm.value = True
    fut = cli_arm.call_async(arm)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    print(f"[auto_takeoff] arming: {fut.result()}")

    t0 = time.time()
    while time.time() - t0 < 20.0:
        spin(1.0)
        print(f"[auto_takeoff] t={time.time() - t0:5.1f}s alt={pose['z']:5.2f} m "
              f"armed={st['armed']} mode={st['mode']}")

    req2 = SetMode.Request()
    req2.custom_mode = "AUTO.LAND"
    fut2 = cli_mode.call_async(req2)
    rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
    print(f"[auto_takeoff] AUTO.LAND: {fut2.result()}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
