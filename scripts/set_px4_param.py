#!/usr/bin/env python3
"""通过 mavros 的 ParamSetV2 服务写 PX4 参数（绕开 ros2 CLI 的 daemon 问题）。

mavros2 里没有 /mavros/param/get，只有：
  /mavros/param/set  [mavros_msgs/srv/ParamSetV2]
  /mavros/param/pull [mavros_msgs/srv/ParamPull]
用法:
  set_px4_param.py set NAV_DLL_ACT 0
  set_px4_param.py set MPC_XY_VEL_MAX 5.0
"""
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue
from mavros_msgs.srv import ParamSetV2

# rcl_interfaces/ParameterValue.type 的取值
TYPE_INT = 2
TYPE_DOUBLE = 3


def make_value(raw: str) -> ParameterValue:
    pv = ParameterValue()
    try:
        pv.type = TYPE_INT
        pv.integer_value = int(raw)
        pv.double_value = float(int(raw))
    except ValueError:
        pv.type = TYPE_DOUBLE
        pv.double_value = float(raw)
    return pv


def main() -> int:
    if len(sys.argv) != 4 or sys.argv[1] != "set":
        print(__doc__)
        return 2
    name, raw = sys.argv[2], sys.argv[3]

    rclpy.init()
    node = Node("px4_param_tool")
    try:
        cli = node.create_client(ParamSetV2, "/mavros/param/set")
        if not cli.wait_for_service(timeout_sec=15.0):
            print("FAIL: /mavros/param/set 不可用")
            return 1
        req = ParamSetV2.Request()
        req.param_id = name
        req.force_set = False
        req.value = make_value(raw)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None:
            print("FAIL: 无响应")
            return 1
        print(f"SET {name}={raw}: success={res.success} "
              f"回读 type={res.value.type} integer={res.value.integer_value} "
              f"double={res.value.double_value}")
        return 0 if res.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
