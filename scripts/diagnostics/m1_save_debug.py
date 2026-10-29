#!/usr/bin/env python3
"""把一张 /aruco/debug_image 存成 PNG，便于人工核对角点框与坐标轴。

注意：本机不能用 cv_bridge（pip numpy 与 C++ 扩展 ABI 不一致，
imgmsg_to_cv2 会 AttributeError 然后 segfault），所以直接
np.frombuffer(msg.data) 重排。

用法：python3 scripts/diagnostics/m1_save_debug.py <输出路径>
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, path: str) -> None:
        super().__init__('m1_save_debug')
        self.path = path
        self.saved = False
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, '/aruco/debug_image', self.on_image, qos)

    def on_image(self, msg: Image) -> None:
        if self.saved:
            return
        enc = msg.encoding
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if enc == 'rgb8':
            img = buf.reshape(msg.height, msg.width, 3)[:, :, ::-1]
        elif enc == 'bgr8':
            img = buf.reshape(msg.height, msg.width, 3)
        else:
            self.get_logger().warn(f'不支持的编码 {enc}')
            return
        cv2.imwrite(self.path, img)
        self.get_logger().info(
            f'已保存 {self.path}（{msg.width}x{msg.height} {enc}，'
            f'frame_id={msg.header.frame_id}）')
        self.saved = True


def main() -> int:
    # 允许被 dev_run_aruco.sh 调用（它会额外传采样秒数之类的参数），
    # 所以只认"看起来像路径"的那个参数。
    path = '/mnt/c/Users/Administrator/AppData/Local/Temp/aruco_stage/dev/aruco_debug.png'
    for arg in sys.argv[1:]:
        if arg.startswith('-'):
            continue
        if arg.replace('.', '', 1).isdigit():
            continue
        path = arg
        break
    rclpy.init()
    node = Saver(path)
    import time
    deadline = time.monotonic() + 40.0
    while rclpy.ok() and not node.saved and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not node.saved:
        print('[DEBUG] RESULT: FAIL（40 s 内没收到 /aruco/debug_image）')
        return 1
    print(f'[DEBUG] RESULT: PASS（{path}）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
