#!/usr/bin/env python3
"""把俯视机位录成 mp4（录视频专用，不依赖任何 GUI 窗口）。

为什么不用录屏：WSLg 的窗口画面要经远程桌面协议贴回 Windows，Gazebo 那种
连续更新的大画面很容易 stalled，录出来就是"卡住不动"。而仿真本身（物理 +
相机渲染）是满速的，所以直接从 ROS 话题取帧、自己写 mp4，帧率恒定、可重复。

画面组成：
  - 主画面：/camera/overhead/image_raw（俯视机位，第三人称看无人机与地面码）
  - HUD   ：模式 / 高度 / 到码的水平误差 / 是否看到码（来自 /mission/status）
  - 小窗  ：/aruco/debug_image（机载相机的检测叠加）缩到右下角

用法：
  ros2 run drone_tools record_video --ros-args \
    -p output:=/tmp/f2.mp4 -p fps:=30.0 -p duration:=60.0 -p title:="F2 visual landing"
"""

import os
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from drone_interfaces.msg import MissionStatus

# 图像话题：与桥接/config 保持一致（best_effort + depth 5，与 sensor data 语义一致）
IMAGE_QOS = QoSProfile(
    depth=5,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray（不依赖 cv_bridge，见 troubleshooting §10）。"""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.width, 3)
        return img if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        img = buf.reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'不支持的图像编码: {msg.encoding}')


def draw_text_block(img, lines, origin=(12, 30), line_height=30):
    """左上角黑底白字，保证任何背景下都能看清。"""
    width = 0
    for text, _ in lines:
        (w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        width = max(width, w)
    height = line_height * len(lines) + 14
    x, y = origin
    overlay = img.copy()
    cv2.rectangle(overlay, (x - 8, y - 24), (x + width + 10, y - 24 + height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)
    for i, (text, color) in enumerate(lines):
        cv2.putText(img, text, (x, y + i * line_height), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, color, 2, cv2.LINE_AA)
    return img


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('record_video')

        self.output = self.declare_parameter('output', '').value
        dyn = ParameterDescriptor(dynamic_typing=True)
        # 这两个参数可能被命令行以整数传入（-p duration:=85），用动态类型避免
        # InvalidParameterTypeException: ... '85' of type 'INTEGER', expecting 'DOUBLE'
        self.fps = float(self.declare_parameter('fps', 25.0, dyn).value)
        self.duration = float(self.declare_parameter('duration', 0.0, dyn).value)
        # cv2.putText 用的是 Hershey 矢量字体，只认 ASCII：中文会被画成一串问号，
        # 所以这里把非 ASCII 字符滤掉，滤空则回退成英文标题（HUD 用英文最稳）。
        raw_title = self.declare_parameter('title', '').value
        self.title = ''.join(c for c in raw_title if 32 <= ord(c) < 127) or 'ArUco Landing'
        self.overhead_topic = self.declare_parameter(
            'overhead_topic', '/camera/overhead/image_raw').value
        self.debug_topic = self.declare_parameter('debug_topic', '/aruco/debug_image').value
        self.status_topic = self.declare_parameter('status_topic', '/mission/status').value
        self.use_inset = bool(self.declare_parameter('use_inset', True).value)
        self.inset_width = int(self.declare_parameter('inset_width', 320).value)
        self.stall_warn = float(self.declare_parameter('stall_warn', 2.0, dyn).value)

        if not self.output:
            raise SystemExit('参数 output 不能为空（例：-p output:=/tmp/f2.mp4）')
        os.makedirs(os.path.dirname(os.path.abspath(self.output)), exist_ok=True)

        self.writer = None
        self.frame_size = None
        self.frames = 0
        self.written = 0
        self.skipped = 0
        self.timeline_start = None
        self.started = time.monotonic()
        self.last_frame_time = None
        self.last_warn = 0.0
        self.debug_img = None

        # 状态量（HUD 用），全部来自 /mission/status
        self.mode = '-'
        self.state = '-'
        self.altitude = 0.0
        self.lateral = -1.0
        self.marker = False
        self.armed = False

        self.create_subscription(Image, self.overhead_topic, self.on_image, IMAGE_QOS)
        self.create_subscription(Image, self.debug_topic, self.on_debug, IMAGE_QOS)
        self.create_subscription(MissionStatus, self.status_topic, self.on_status, 10)

        self.get_logger().info(f'录制主画面：{self.overhead_topic} -> {self.output}')
        self.get_logger().info(f'HUD 状态源：{self.status_topic}；小窗：{self.debug_topic}')

        self.timer = self.create_timer(0.5, self.tick)

    # ---------- 回调 ----------
    def on_debug(self, msg):
        try:
            self.debug_img = to_bgr(msg)
        except ValueError:
            self.debug_img = None

    def on_status(self, msg):
        self.mode = msg.mode or '-'
        self.state = msg.state or '-'
        self.altitude = float(msg.altitude)
        self.lateral = float(msg.lateral_error)
        self.marker = bool(msg.marker_visible)
        self.armed = bool(msg.armed)

    def on_image(self, msg):
        try:
            frame = to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            self.frame_size = (w, h)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output, fourcc, self.fps, self.frame_size)
            if not self.writer.isOpened():
                raise SystemExit(f'无法创建视频文件：{self.output}')
            self.get_logger().info(f'分辨率 {w}x{h} @ {self.fps:.0f} fps')

        # 固定分辨率（万一 gz 改了分辨率，缩放对齐，避免写坏文件）
        if (frame.shape[1], frame.shape[0]) != self.frame_size:
            frame = cv2.resize(frame, self.frame_size)

        now = time.monotonic()
        if self.timeline_start is None:
            self.timeline_start = now

        # 帧节奏控制：相机实际到帧率会抖（软件渲染 + Python 拷贝），
        # 若直接"来一帧写一帧"，视频时长会和真实时长对不上（看着像快放/慢放）。
        # 这里按"墙上时间 × 目标 fps"决定应该已经写到第几帧：
        #   落后 → 复制当前帧补上（最多补 3 帧，超了就重新对齐，避免雪崩）
        #   超前 → 丢掉这一帧（相机比目标 fps 快）
        target = int((now - self.timeline_start) * self.fps)
        if self.written > target:
            self.skipped += 1
            self.last_frame_time = now
            return
        frame = self.decorate(frame)
        dups = 0
        while self.written <= target and dups < 3:
            self.writer.write(frame)
            self.written += 1
            dups += 1
        if dups == 3 and self.written <= target:
            # 落后太多（比如仿真被卡了一下），直接重新对齐时间轴
            self.timeline_start = now - self.written / self.fps
        self.frames += 1
        self.last_frame_time = now

    # ---------- 画面装饰 ----------
    def decorate(self, frame):
        elapsed = time.monotonic() - self.started
        lines = []
        if self.title:
            lines.append((self.title, (255, 255, 255)))
        lines.append((f't={elapsed:5.1f}s   mode={self.mode}', (0, 255, 255)))
        lateral_text = 'n/a' if self.lateral < 0 else f'{self.lateral:4.2f} m'
        lines.append((f'ALT {self.altitude:4.2f} m    lateral_err {lateral_text}',
                      (255, 255, 255)))
        marker_color = (0, 255, 0) if self.marker else (0, 0, 255)
        lines.append((f'marker {"VISIBLE" if self.marker else "LOST"}    '
                      f'{"ARMED" if self.armed else "disarmed"}', marker_color))
        frame = draw_text_block(frame, lines)

        if self.use_inset and self.debug_img is not None:
            inset = self.debug_img
            h, w = inset.shape[:2]
            scale = self.inset_width / float(w)
            inset = cv2.resize(inset, (self.inset_width, int(h * scale)))
            ih, iw = inset.shape[:2]
            fh, fw = frame.shape[:2]
            x0, y0 = fw - iw - 12, fh - ih - 12
            if x0 > 0 and y0 > 0:
                frame[y0 - 2:y0 + ih + 2, x0 - 2:x0 + iw + 2] = (0, 0, 0)
                frame[y0:y0 + ih, x0:x0 + iw] = inset
        return frame

    # ---------- 定时检查 ----------
    def tick(self):
        now = time.monotonic()
        if self.last_frame_time is not None and now - self.last_frame_time > self.stall_warn:
            if now - self.last_warn > 5.0:
                self.last_warn = now
                self.get_logger().warn(
                    f'{now - self.last_frame_time:.1f} s 没有收到图像帧'
                    f'（检查 {self.overhead_topic} 是否有发布者）')
        if self.duration > 0 and now - self.started >= self.duration:
            self.get_logger().info('到达指定时长，收尾')
            self.finish()
            rclpy.shutdown()

    def finish(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        elapsed = time.monotonic() - self.started
        size = os.path.getsize(self.output) if os.path.exists(self.output) else 0
        rate = self.written / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f'录制结束：写入 {self.written} 帧 / {elapsed:.1f} s（{rate:.1f} fps），'
            f'收到 {self.frames} 帧、过快丢弃 {self.skipped} 帧，'
            f'{size / 1e6:.1f} MB -> {self.output}')


def main(argv=None):
    rclpy.init(args=argv)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    main()
