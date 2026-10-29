#!/usr/bin/env python3
"""键盘遥控（需求书 §5.1 / M2）。

只做一件事：把按键翻译成 /teleop/cmd（geometry_msgs/Twist，**body 系**）
和 /mission/mode（std_msgs/String）。不发任何 setpoint —— 需求书规定
只有 mission_manager 能下令，遥控指令也要经它出口。

键位：
    w / s    前进 / 后退   ±0.5 m/s
    a / d    左移 / 右移   ±0.5 m/s
    r / f    上升 / 下降   ±0.4 m/s
    q / e    左偏航 / 右偏航 ±0.5 rad/s
    Space    未解锁时起飞到 1.5 m；已解锁时降落
    1        进入 MANUAL（必要时先起飞）
    2        ARUCO_LAND（M3 才实现）
    3        WAYPOINT（M4 才实现）
    4        AUTO（M5 才实现）
    0        立即降落
    ESC      紧急降落（与 0 同效，只是语义更明确）
    h        重新打印这份帮助

松开按键（或终端不再连发）超过 key_timeout 就会清零速度 —— 定时器兜底，
不依赖终端是否开连发。
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from drone_interfaces.msg import MissionStatus

HELP = """键盘遥控（输入焦点必须在这个终端窗口里）：
  w / s    前进 / 后退        a / d    左移 / 右移
  r / f    上升 / 下降        q / e    左偏航 / 右偏航
  Space    未起飞 = 起飞到 1.5 m；已起飞 = 降落
  1  MANUAL（手动）      2  ARUCO_LAND（视觉降落）
  3  WAYPOINT（航点）    4  AUTO（全自主：航点→搜码→降落）
  0  立即降落（LAND）      ESC / Ctrl-C  紧急降落      h  帮助
"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('drone_teleop')

        # 参数全部来自 drone_bringup/config/flows.yaml
        self.publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)
        self.key_timeout = float(self.declare_parameter('key_timeout', 0.3).value)
        self.vel_xy = float(self.declare_parameter('vel_xy', 0.5).value)
        self.vel_z = float(self.declare_parameter('vel_z', 0.4).value)
        self.yaw_rate = float(self.declare_parameter('yaw_rate', 0.5).value)

        self.cmd_pub = self.create_publisher(Twist, '/teleop/cmd', 10)
        self.mode_pub = self.create_publisher(String, '/mission/mode', 10)
        self.status_sub = self.create_subscription(
            MissionStatus, '/mission/status', self.on_status, 10)

        self.armed = False
        self.active_key = ''
        self.active_since = 0.0

        if not sys.stdin.isatty():
            self.get_logger().error(
                '键盘遥控需要交互式终端：当前 stdin 不是 TTY（大概是被 nohup / 管道 / '
                'launch 后台启动了）。请在 Windows 里打开一个 WSL 终端，直接运行 '
                '`bash scripts/manual_demo.sh teleop`。')
            raise SystemExit(2)
        self.old_term = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print(HELP, flush=True)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

    # ---------- 回调 ----------

    def on_status(self, msg):
        self.armed = bool(msg.armed)

    def on_timer(self):
        # 读干净 stdin 里所有待处理字节；只保留最后一次按键
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                break
            data = sys.stdin.read(1)
            if not data:
                break
            if not self.handle_key(data):
                return  # 退出请求

        fresh = (time.monotonic() - self.active_since) < self.key_timeout
        twist = Twist()
        key = self.active_key if fresh else ''
        if key == 'w':
            twist.linear.x = self.vel_xy
        elif key == 's':
            twist.linear.x = -self.vel_xy
        elif key == 'a':
            twist.linear.y = self.vel_xy
        elif key == 'd':
            twist.linear.y = -self.vel_xy
        elif key == 'r':
            twist.linear.z = self.vel_z
        elif key == 'f':
            twist.linear.z = -self.vel_z
        elif key == 'q':
            twist.angular.z = self.yaw_rate
        elif key == 'e':
            twist.angular.z = -self.yaw_rate
        self.cmd_pub.publish(twist)

    def handle_key(self, key):
        """返回 False 表示请求退出。"""
        if key == '\x03':  # Ctrl-C
            self.request_land('紧急降落')
            return False

        if key == 'h':
            print(HELP, flush=True)
            return True

        if key in 'wsadrfqe':
            self.active_key = key
            self.active_since = time.monotonic()
            return True

        if key in ' \r\n':
            if self.armed:
                self.request_mode('LAND', '降落')
            else:
                self.request_mode('MANUAL', '起飞到 1.5 m')
            return True

        if key == '1':
            self.request_mode('MANUAL', 'MANUAL')
        elif key == '2':
            self.request_mode('ARUCO_LAND', 'ARUCO_LAND')
        elif key == '3':
            self.request_mode('WAYPOINT', 'WAYPOINT')
        elif key == '4':
            self.request_mode('AUTO', 'AUTO')
        elif key == '0':
            self.request_mode('LAND', '立即降落')
        elif key == '\x1b':
            self.request_land('紧急降落')
        return True

    # ---------- 发送 ----------

    def request_mode(self, mode, label):
        self.cmd_pub.publish(Twist())  # 切模式前先清零，避免残留速度
        self.mode_pub.publish(String(data=mode))
        print(f'[teleop] → {label}（{mode}）', flush=True)

    def request_land(self, label):
        self.request_mode('LAND', label)

    def restore_terminal(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)
        except Exception:  # noqa: BLE001
            pass


def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_land('退出前降落')
        node.restore_terminal()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
