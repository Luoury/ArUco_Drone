#!/usr/bin/env python3
"""排查"检测到的码位置为什么离无人机 2.4 m"。

流程：请求 MANUAL 起飞 → 前飞 2 s 制造水平偏移 → 悬停 10 s，边打印
无人机位姿、检测到的码位置、二者水平距离，以及相机系下的原始距离。

    python3 det_probe.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__('det_probe')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose = None
        self.det = None
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos)
        self.create_subscription(PoseStamped, '/aruco/detection', self.on_det, qos)
        self.cmd_pub = self.create_publisher(Twist, '/teleop/cmd', 10)
        self.mode_pub = self.create_publisher(String, '/mission/mode', 10)

    def on_pose(self, msg):
        self.pose = msg

    def on_det(self, msg):
        self.det = msg

    def spin(self, seconds, twist=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if twist is not None:
                self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def mode(self, name, seconds=1.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.mode_pub.publish(String(data=name))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = Probe()
    node.spin(1.0)
    print('[probe] 请求 MANUAL', flush=True)
    node.mode('MANUAL')
    node.spin(12.0)  # 等起飞 + OFFBOARD 接管

    print('[probe] 前飞 2 s + 左移 2 s（造出 x/y 都有偏移的几何）', flush=True)
    fwd = Twist()
    fwd.linear.x = 0.5
    node.spin(2.0, fwd)
    left = Twist()
    left.linear.y = 0.5
    node.spin(2.0, left)
    node.cmd_pub.publish(Twist())
    node.spin(1.0)

    print('t, drone_x, drone_y, drone_z, det_x, det_y, det_z, lateral', flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        node.spin(0.5)
        if node.pose is None:
            continue
        p = node.pose.pose.position
        if node.det is None:
            print('%.1f, %.2f, %.2f, %.2f, -, -, -, -' % (
                time.monotonic() - t0, p.x, p.y, p.z), flush=True)
            continue
        d = node.det.pose.position
        lateral = math.hypot(p.x - d.x, p.y - d.y)
        # 真实偏移（码在原点）与估计偏移一起打印，直接看旋转关系
        print('%.1f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f | true=%.2f,%.2f est=%.2f,%.2f' % (
            time.monotonic() - t0, p.x, p.y, p.z, d.x, d.y, d.z, lateral,
            -p.x, -p.y, d.x - p.x, d.y - p.y), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
