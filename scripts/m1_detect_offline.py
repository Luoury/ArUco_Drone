#!/usr/bin/env python3
"""M1 离线验证：从 arucotag.png 识别出字典与 ID（需求书 §8.2 校准步骤第 1 条）。

贴图本身没有白静区（黑框直接顶到图片边缘），ArUco 检测需要静区，
所以先补一圈白边再检测 —— 这是唯一对图像做的预处理。
"""

import sys

import cv2
import numpy as np

DEFAULT_PNG = "/home/luosury/PX4-Autopilot/Tools/simulation/gz/models/arucotag/arucotag.png"
EXPECTED_ID = 0


def load_with_quiet_zone(path, border=40):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"读不到图片: {path}")
    return cv2.copyMakeBorder(img, border, border, border, border,
                              cv2.BORDER_CONSTANT, value=255)


def main(argv):
    path = argv[1] if len(argv) > 1 else DEFAULT_PNG
    gray = load_with_quiet_zone(path)
    print(f"[M1] 载入 {path} → 补白边后 {gray.shape[1]}x{gray.shape[0]}")

    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    for name in ("DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_6X6_250"):
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            continue
        c = corners[0].reshape(4, 2)
        side_px = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
        print(f"[M1] dictionary = {name}")
        print(f"[M1] marker_id = {int(ids.ravel()[0])}")
        print(f"[M1] 检测到角点 {side_px:.1f} px（补白边后图宽 {gray.shape[1]} px）")
        ok = int(ids.ravel()[0]) == EXPECTED_ID
        print(f"[M1] RESULT: {'PASS' if ok else 'FAIL'}（期望 id={EXPECTED_ID}）")
        return 0 if ok else 1

    print("[M1] 未检出任何 ArUco 码")
    print("[M1] RESULT: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
