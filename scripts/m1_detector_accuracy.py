#!/usr/bin/env python3
"""M1 检测器精度分解：把"检测误差"里的 EKF 估计误差单独量出来。

为什么要这个脚本：
  /aruco/detection 是在 map 系里发布的，而 map 系的原点由 PX4 EKF 决定。
  无人机其实悬停在码正上方（gz 世界系真值 ≈ (0,0)），但 EKF 报的
  /mavros/local_position/pose 会有 2~6 cm 的漂移，这部分会原封不动地
  进入检测结果。所以"检测 vs gz 真值"的残差 = 检测器本体误差 + EKF 误差。

  本脚本同时采两路真值：
    * `gz model -m <model> -p`   -> gz 世界系真值（厘米级可信）
    * /mavros/local_position/pose -> EKF 估计
  两者之差就是 EKF 在该时刻的估计误差，把它扣掉之后剩下的才是
  检测器本体（solvePnP + 姿态旋转 + 地图平移）的误差。

用法（需要 sim + bringup + aruco_detector 都在跑）：
  python3 scripts/m1_detector_accuracy.py [--model x500_mono_cam_down_0]
                                          [--duration 15] [--tag-z 0.001]
"""

import argparse
import re
import statistics
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

FLOAT_RE = re.compile(r'-?\d+\.\d+(?:[eE][-+]?\d+)?')


class Accuracy(Node):
    def __init__(self, model: str) -> None:
        super().__init__('m1_detector_accuracy')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.model = model
        self.pose = None
        self.detections = deque(maxlen=4000)
        self.offsets = []          # (t_mono, (dx, dy, dz)) = EKF - gz真值
        self.stop_sampling = False

        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos)
        self.create_subscription(PoseStamped, '/aruco/detection',
                                 self.on_detection, qos)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def on_detection(self, msg: PoseStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.detections.append((stamp,
                                (msg.pose.position.x, msg.pose.position.y,
                                 msg.pose.position.z)))

    def gz_truth(self):
        try:
            out = subprocess.run(
                ['gz', 'model', '-m', self.model, '-p'],
                capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            return None
        nums = [float(x) for x in FLOAT_RE.findall(out)]
        return tuple(nums[:3]) if len(nums) >= 3 else None

    def sample_offsets(self, period: float) -> None:
        while not self.stop_sampling:
            gz = self.gz_truth()
            if gz is not None and self.pose is not None:
                ekf = self.pose
                self.offsets.append(
                    (time.monotonic(),
                     (ekf[0] - gz[0], ekf[1] - gz[1], ekf[2] - gz[2])))
            time.sleep(period)


def fmt(vals) -> str:
    return '(' + ', '.join(f'{v:+.4f}' for v in vals) + ')'


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='x500_mono_cam_down_0')
    # 位置参数（dev_run_aruco.sh 会把采样秒数当位置参数传进来），也支持 --duration
    parser.add_argument('duration', nargs='?', type=float, default=None)
    parser.add_argument('--duration', dest='duration_opt', type=float, default=None)
    parser.add_argument('--tag-z', type=float, default=0.001,
                        help='码面在 gz 世界系的高度（arucotag 模型 pose z）')
    args = parser.parse_args()
    if args.duration is None:
        args.duration = args.duration_opt if args.duration_opt is not None else 15.0

    rclpy.init()
    node = Accuracy(args.model)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('[ACC] 等待检测数据与位姿（最多 40 s）')
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline:
        if len(node.detections) > 10 and node.pose is not None:
            break
        time.sleep(0.2)
    if len(node.detections) <= 10 or node.pose is None:
        print(f'[ACC] RESULT: FAIL（没收到数据：detections={len(node.detections)},'
              f' pose={node.pose}）')
        rclpy.shutdown()
        return 1

    sampler = threading.Thread(target=node.sample_offsets, args=(0.5,), daemon=True)
    sampler.start()

    start_count = len(node.detections)
    t0 = time.monotonic()
    print(f'[ACC] 采样 {args.duration:.0f} s')
    time.sleep(args.duration)
    node.stop_sampling = True
    elapsed = time.monotonic() - t0

    dets = list(node.detections)[start_count:]
    offs = node.offsets
    if not dets or not offs:
        print(f'[ACC] RESULT: FAIL（采样为空：detections={len(dets)}, offsets={len(offs)}）')
        rclpy.shutdown()
        return 1

    # EKF 误差：取中位数作为"常量偏置"，另外报告其漂移幅度
    mx = statistics.median([o[1][0] for o in offs])
    my = statistics.median([o[1][1] for o in offs])
    mz = statistics.median([o[1][2] for o in offs])
    bias = (mx, my, mz)
    spread = (max(o[1][0] for o in offs) - min(o[1][0] for o in offs),
              max(o[1][1] for o in offs) - min(o[1][1] for o in offs),
              max(o[1][2] for o in offs) - min(o[1][2] for o in offs))

    truth = (0.0, 0.0, args.tag_z)
    raw_3d, raw_xy, raw_z = [], [], []
    cor_3d, cor_xy, cor_z = [], [], []
    for _stamp, p in dets:
        e = [p[i] - truth[i] for i in range(3)]
        raw_3d.append(sum(v * v for v in e) ** 0.5)
        raw_xy.append((e[0] ** 2 + e[1] ** 2) ** 0.5)
        raw_z.append(e[2])
        c = [e[i] - bias[i] for i in range(3)]
        cor_3d.append(sum(v * v for v in c) ** 0.5)
        cor_xy.append((c[0] ** 2 + c[1] ** 2) ** 0.5)
        cor_z.append(c[2])

    print(f'[ACC] 采样帧数 {len(dets)}（{len(dets) / elapsed:.1f} Hz）')
    print(f'[ACC] EKF 相对 gz 真值的偏置（中位）= {fmt(bias)}  样本 {len(offs)} 个')
    print(f'[ACC] 该偏置在采样窗内的漂移幅度 = {fmt(spread)}')
    print(f'[ACC] 原始误差：三维中位 {statistics.median(raw_3d):.4f} m / '
          f'水平中位 {statistics.median(raw_xy):.4f} m / '
          f'垂直中位 {statistics.median(raw_z):+.4f} m')
    print(f'[ACC] 扣掉 EKF 偏置后（检测器本体）：三维中位 '
          f'{statistics.median(cor_3d):.4f} m / 水平中位 '
          f'{statistics.median(cor_xy):.4f} m / 垂直中位 '
          f'{statistics.median(cor_z):+.4f} m')

    ok = statistics.median(cor_3d) <= 0.05
    print(f'[ACC] RESULT: {"PASS" if ok else "FAIL"}'
          f'（检测器本体三维中位误差 {statistics.median(cor_3d):.4f} m，判据 ≤0.05 m）')
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
