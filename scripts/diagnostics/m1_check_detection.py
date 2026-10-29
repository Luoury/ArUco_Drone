#!/usr/bin/env python3
"""M1 在线验证：悬停时 /aruco/detection 与真值（码在 map 原点）的误差。

真值来源：arucotag 模型 <pose>0 0 0.001</pose>，即码中心在 map=(0, 0, 0.001)。
除误差外还输出"距离标定"信息：p_map.z = drone_z + p_body.z 且 p_body.z = -距离，
故 距离估计 = drone_z - p_map.z；隐含边长 = marker_size * 真实距离 / 估计距离。
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
TRUTH = (0.0, 0.0, 0.001)
TOLERANCE = 0.05  # m，需求书 M1 验收线
MARKER_SIZE = 0.50  # m，与 aruco.yaml 保持一致，仅用于反解真实边长


def main(argv):
    duration = float(argv[1]) if len(argv) > 1 else 12.0
    rclpy.init()
    node = Node("m1_check_detection")
    dets = []
    drone = {"pos": None, "last": None}

    def on_pose(msg):
        drone["pos"] = msg.pose.position
        drone["last"] = msg.pose.position

    def on_det(msg):
        dets.append((msg, drone["pos"]))

    node.create_subscription(PoseStamped, "/aruco/detection", on_det, BEST)
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", on_pose, BEST)

    t0 = time.time()
    # 先等无人机爬到稳态悬停：爬升阶段高度每帧都在变，采样进去会把"距离标定"
    # 和误差统计搞脏（实测爬升段的 px 边长标准差可达 3%）。
    zs = []

    def on_pose_hist(msg):
        zs.append(msg.pose.position.z)
        if len(zs) > 20:
            del zs[0]

    node.create_subscription(PoseStamped, "/mavros/local_position/pose", on_pose_hist, BEST)
    while time.time() - t0 < 25.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(zs) >= 20 and (max(zs) - min(zs)) < 0.02:
            break
    print(f"[M1] 悬停已稳定（高度波动 {0.0 if len(zs) < 2 else max(zs) - min(zs):.3f} m），开始采样")

    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    print(f"[M1] 采样 {duration:.0f} s，收到 /aruco/detection {len(dets)} 条")
    p = drone["last"]
    if p is not None:
        print(f"[M1] 无人机 ENU = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
    if not dets:
        print("[M1] 没有检测结果 → RESULT: FAIL（相机看不到码或检测器没工作）")
        return 1

    px = sum(d.pose.position.x for d, _ in dets) / len(dets)
    py = sum(d.pose.position.y for d, _ in dets) / len(dets)
    pz = sum(d.pose.position.z for d, _ in dets) / len(dets)
    errs = [math.dist((d.pose.position.x, d.pose.position.y, d.pose.position.z), TRUTH)
            for d, _ in dets]
    grid_err = [math.dist((d.pose.position.x, d.pose.position.y), TRUTH[:2]) for d, _ in dets]
    z_err = [d.pose.position.z - TRUTH[2] for d, _ in dets]

    print(f"[M1] detection 均值 = ({px:.3f}, {py:.3f}, {pz:.3f})  真值 = {TRUTH}")
    print(f"[M1] 三维误差：中位数 {sorted(errs)[len(errs) // 2]:.3f} m / 最大 {max(errs):.3f} m")
    print(f"[M1] 水平误差：中位数 {sorted(grid_err)[len(grid_err) // 2]:.3f} m / 最大 {max(grid_err):.3f} m")
    print(f"[M1] 垂直误差：中位数 {sorted(z_err)[len(z_err) // 2]:+.3f} m / 范围 "
          f"{min(z_err):+.3f} ~ {max(z_err):+.3f} m")

    # 距离标定：隐含边长 = MARKER_SIZE * 真实距离 / 估计距离
    implied = []
    for d, pos in dets:
        if pos is None:
            continue
        dist_est = pos.z - d.pose.position.z
        dist_true = pos.z - TRUTH[2]
        if dist_est > 0.3:
            implied.append(MARKER_SIZE * dist_true / dist_est)
    if implied:
        implied.sort()
        print(f"[M1] 距离标定：估计距离/真实距离 = {1.0 / (sorted(implied)[len(implied) // 2] / MARKER_SIZE):.4f}，"
              f"隐含 marker_size 中位数 = {implied[len(implied) // 2]:.4f} m（当前 {MARKER_SIZE:.2f}）")

    print(f"[M1] frame_id = {dets[-1][0].header.frame_id}")
    ok = sorted(errs)[len(errs) // 2] <= TOLERANCE
    print(f"[M1] RESULT: {'PASS' if ok else 'FAIL'}（中位误差 "
          f"{sorted(errs)[len(errs) // 2]:.3f} m，判据 ≤{TOLERANCE:.2f} m）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
