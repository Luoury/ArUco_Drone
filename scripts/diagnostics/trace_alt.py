#!/usr/bin/env python3
"""同时记录 mavros 估计高度、gz 真值高度、模式与解锁状态（10 Hz）。

用途：判断"无人机是不是真的掉了"还是"位姿读数跳变"。
    python3 trace_alt.py 15 > /tmp/alt_trace.csv
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Tracer(Node):
    def __init__(self):
        super().__init__('trace_alt')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.mav_z = None
        self.gz_z = None
        self.mode = '?'
        self.armed = False
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_mav, qos)
        self.create_subscription(PoseStamped, '/gz/drone_pose', self.on_gz, qos)
        self.create_subscription(State, '/mavros/state', self.on_state, 10)

    def on_mav(self, msg):
        self.mav_z = msg.pose.position.z

    def on_gz(self, msg):
        self.gz_z = msg.pose.position.z

    def on_state(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = Tracer()
    period = 0.1
    t0 = time.monotonic()
    print('t,mavros_z,gz_z,mode,armed', flush=True)
    while time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=period)
        t = time.monotonic() - t0
        print('%.1f,%s,%s,%s,%d' % (
            t,
            '-' if node.mav_z is None else '%.3f' % node.mav_z,
            '-' if node.gz_z is None else '%.3f' % node.gz_z,
            node.mode, node.armed), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
