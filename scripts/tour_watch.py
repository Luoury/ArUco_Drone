#!/usr/bin/env python3
"""盯着 /mission/status，打印状态迁移，直到 TOUR_DONE（或超时）。

用法：
    python3 scripts/tour_watch.py [超时秒数，默认 320]

退出码：0 = TOUR_DONE；1 = 超时 / 进入 *_ABORT / 节点没数据。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import MissionStatus


class TourWatch(Node):
    def __init__(self, timeout: float) -> None:
        super().__init__('tour_watch')
        self.timeout = timeout
        self.state = ''
        self.mode = ''
        self.started = False
        self.done = False
        self.aborted = False
        self.t0 = self.get_clock().now()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            MissionStatus, '/mission/status', self.on_status, qos)
        self.timer = self.create_timer(0.5, self.tick)

    def on_status(self, msg: MissionStatus) -> None:
        if not self.started:
            self.started = True
        if msg.state != self.state or msg.mode != self.mode:
            elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            print(f'[{elapsed:7.1f}s] mode={msg.mode:12s} state={msg.state:14s} '
                  f'alt={msg.altitude:5.2f} lateral={msg.lateral_error:5.2f} '
                  f'marker={msg.marker_visible}', flush=True)
            self.state = msg.state
            self.mode = msg.mode
        if msg.state == 'TOUR_DONE':
            self.done = True
        if msg.state.endswith('_ABORT'):
            self.aborted = True

    def tick(self) -> None:
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if not self.started and elapsed > 20.0:
            print('[tour_watch] 20 s 内没收到 /mission/status，退出', flush=True)
            self.done = False
            raise SystemExit(1)
        if self.done:
            print('[tour_watch] TOUR_DONE：全部工位完成', flush=True)
            raise SystemExit(0)
        if self.aborted:
            print(f'[tour_watch] 进入异常状态 {self.state}，退出', flush=True)
            raise SystemExit(1)
        if elapsed > self.timeout:
            print(f'[tour_watch] 超时 {self.timeout:.0f} s，最后状态 {self.state}', flush=True)
            raise SystemExit(1)


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 320.0
    rclpy.init()
    node = TourWatch(timeout)
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        return int(exc.code or 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
