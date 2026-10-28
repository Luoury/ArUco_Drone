#!/usr/bin/env python3
"""对比"PX4/mavros 的 local_position"与"Gazebo 真值 pose"，量化两个坐标系的原点偏移。

背景：/aruco/detection 是在 map 系（= mavros local_position 的原点）里表达的，
而 arucotag 的真值 (0, 0, 0.001) 是 Gazebo 世界系里的。如果 PX4 的 EKF 原点
相对 Gazebo 世界原点有几十毫米的偏置，那么"检测结果 vs gz 真值"的误差里
就混进了这个偏置，不能全算在检测器头上。本脚本把这个偏置单独量出来。
"""

import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
MODEL = "x500_mono_cam_down_0"


def gz_pose(model):
    out = subprocess.run(["gz", "model", "-m", model, "-p"],
                         capture_output=True, text=True, timeout=20).stdout
    nums = re.findall(r"-?\d+\.\d+", out)
    if len(nums) < 3:
        return None, out.strip()
    return tuple(float(v) for v in nums[:3]), ""


def main(argv):
    n = int(argv[1]) if len(argv) > 1 else 6
    rclpy.init()
    node = Node("m1_frame_check")
    state = {"p": None}
    node.create_subscription(
        PoseStamped, "/mavros/local_position/pose",
        lambda m: state.__setitem__("p", m.pose.position), BEST)

    for i in range(n):
        t0 = time.time()
        while time.time() - t0 < 0.5:
            rclpy.spin_once(node, timeout_sec=0.05)
        wp, err = gz_pose(MODEL)
        p = state["p"]
        if wp is None:
            print(f"[frame] 取 gz pose 失败：{err}")
            continue
        if p is None:
            print("[frame] 还没收到 mavros pose")
            continue
        dx, dy, dz = p.x - wp[0], p.y - wp[1], p.z - wp[2]
        print(f"[frame] gz=({wp[0]:+.3f}, {wp[1]:+.3f}, {wp[2]:+.3f})  "
              f"mavros=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})  "
              f"偏置=({dx:+.3f}, {dy:+.3f}, {dz:+.3f})")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
