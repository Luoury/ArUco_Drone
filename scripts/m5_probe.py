#!/usr/bin/env python3
"""M5 诊断：把 /mavros/setpoint_raw/local 的内容与高度真值一起记下来。

用途：F4 的 DESCEND 段实测下降速率只有 0.014 m/s（指令 -0.30），且高度在 0.8~1.7 m
之间来回摆。两种可能：① 还有第二个发布者在本节点之外发位置设定点；② 高度估计在跳。
本脚本同时打印：
  - setpoint 的 type_mask / position.z / velocity.z（谁在发、发的是什么）
  - mavros 的 z（估计值）
  - gz 的 z（物理真值，`gz model -m x500_mono_cam_down_0 -p`）
用法：python3 m5_probe.py [秒]
"""

import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

MODEL = 'x500_mono_cam_down_0'


class Probe(Node):
    def __init__(self):
        super().__init__('m5_probe')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.z = -1.0
        self.mask = 0
        self.vz = 0.0
        self.pz = 0.0
        self.count = 0
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.on_setp, qos)

    def on_pose(self, msg):
        self.z = msg.pose.position.z

    def on_setp(self, msg):
        self.mask = msg.type_mask
        self.vz = msg.velocity.z
        self.pz = msg.position.z
        self.count += 1


def gz_z():
    try:
        out = subprocess.run(['gz', 'model', '-m', MODEL, '-p'],
                             capture_output=True, text=True, timeout=5).stdout
        nums = re.findall(r'-?\d+\.\d+(?:[eE][-+]?\d+)?', out.split(']')[0])
        return float(nums[2]) if len(nums) >= 3 else -1.0
    except Exception:
        return -1.0


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    node = Probe()
    t0 = time.monotonic()
    next_print = 0.0
    print('t     pose_z  gz_z   mask      setp_vz  setp_pz  setp_cnt', flush=True)
    while time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic() - t0
        if now >= next_print:
            next_print = now + 1.0
            print(f'{now:5.1f} {node.z:7.3f} {gz_z():6.3f} {node.mask:9d} '
                  f'{node.vz:8.3f} {node.pz:8.3f} {node.count:6d}', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
