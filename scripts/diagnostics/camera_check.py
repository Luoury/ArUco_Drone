#!/usr/bin/env python3
"""相机自检：订阅一帧 /camera/image_raw，打印尺寸/编码/亮度统计。

M0 用来确认"桥接过来的图不是全黑"——只是能收到消息不等于真的在渲染。

用法：source /opt/ros/humble/setup.bash && python3 scripts/diagnostics/camera_check.py
"""

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraCheck(Node):
    def __init__(self) -> None:
        super().__init__('camera_check')
        self._got = False
        self.create_subscription(Image, '/camera/image_raw', self._on_image,
                                 qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if data.size == 0:
            self.get_logger().error('收到空图像')
            return
        self._got = True
        self.get_logger().info(
            f'width={msg.width} height={msg.height} encoding={msg.encoding} '
            f'step={msg.step} bytes={data.size} min={data.min()} max={data.max()} '
            f'mean={data.mean():.2f} nonzero_ratio={np.count_nonzero(data) / data.size:.3f}')
        rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = CameraCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node._got else 1


if __name__ == '__main__':
    sys.exit(main())
