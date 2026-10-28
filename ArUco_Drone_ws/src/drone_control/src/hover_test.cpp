// M0 验收程序：OFFBOARD 起飞到 1.5 m 悬停 30 s，测量水平漂移，最后自动降落。
// 用法（仿真与 MAVROS 已启动）：
//   ros2 run drone_control hover_test
//   ros2 run drone_control hover_test --ros-args --params-file <drone_bringup>/config/hover.yaml

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "drone_control/flight_control.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;

std::chrono::milliseconds toMs(double seconds)
{
  return std::chrono::milliseconds{static_cast<int>(seconds * 1000.0)};
}

std::chrono::seconds toSec(double seconds)
{
  return std::chrono::seconds{static_cast<int>(std::ceil(seconds))};
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hover_test");
  auto logger = node->get_logger();

  // 参数：默认值与 drone_bringup/config/hover.yaml 一致，launch 加载 yaml 时覆盖
  const double hover_altitude = node->declare_parameter("hover_altitude", 1.5);
  const double hover_duration = node->declare_parameter("hover_duration", 30.0);
  const double altitude_tolerance = node->declare_parameter("altitude_tolerance", 0.15);
  const double drift_tolerance = node->declare_parameter("drift_tolerance", 0.30);
  const double setpoint_rate = node->declare_parameter("setpoint_rate", 50.0);
  const double prestream_duration = node->declare_parameter("prestream_duration", 2.0);
  const double connect_timeout = node->declare_parameter("connect_timeout", 30.0);
  const double pose_timeout = node->declare_parameter("pose_timeout", 10.0);
  const double mode_timeout = node->declare_parameter("mode_timeout", 10.0);
  const double climb_timeout = node->declare_parameter("climb_timeout", 20.0);
  const bool auto_land = node->declare_parameter("auto_land", true);
  const double land_timeout = node->declare_parameter("land_timeout", 40.0);

  drone_control::FlightControl fc(node);

  RCLCPP_INFO(logger, "==== hover_test 开始：目标高度 %.2f m，悬停 %.1f s ====",
    hover_altitude, hover_duration);

  if (!fc.waitForConnection(toSec(connect_timeout))) {
    RCLCPP_ERROR(logger, "MAVROS 未连接（等 %.0f s）", connect_timeout);
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "MAVROS 已连接：mode=%s armed=%s", fc.mode().c_str(),
    fc.armed() ? "true" : "false");

  if (!fc.waitForLocalPose(toSec(pose_timeout))) {
    RCLCPP_ERROR(logger, "/mavros/local_position/pose 无数据（等 %.0f s）", pose_timeout);
    rclcpp::shutdown();
    return 1;
  }

  const drone_control::EnuPose start = fc.localPose();
  const drone_control::EnuPose target{start.x, start.y, hover_altitude};
  RCLCPP_INFO(logger, "起飞点 ENU (%.2f, %.2f, %.2f)，悬停目标 (%.2f, %.2f, %.2f)",
    start.x, start.y, start.z, target.x, target.y, target.z);

  drone_control::OffboardOptions options;
  options.prestream = toMs(prestream_duration);
  options.rate_hz = setpoint_rate;
  options.mode_timeout = toSec(mode_timeout);
  options.arm_timeout = toSec(mode_timeout);

  if (!fc.engageOffboard(target, options)) {
    RCLCPP_ERROR(logger, "OFFBOARD 接管失败");
    rclcpp::shutdown();
    return 1;
  }

  // 1) 爬升
  const bool climbed = fc.streamUntilPredicate(target, 0.0, setpoint_rate,
    toSec(climb_timeout),
    [&fc, hover_altitude, altitude_tolerance]() {
      return std::fabs(fc.localPose().z - hover_altitude) <= altitude_tolerance;
    });
  const drone_control::EnuPose after_climb = fc.localPose();
  RCLCPP_INFO(logger, "爬升结果：%s，当前 ENU (%.2f, %.2f, %.2f)",
    climbed ? "到达目标高度" : "超时未到达",
    after_climb.x, after_climb.y, after_climb.z);

  // 2) 悬停：持续 50 Hz 发设定点，统计水平漂移
  const std::chrono::milliseconds period{static_cast<int>(1000.0 / setpoint_rate)};
  const auto hover_end = SteadyClock::now() + toMs(hover_duration);
  double max_drift = 0.0;
  double max_alt_err = 0.0;
  double sum_alt = 0.0;
  double prev_z = fc.localPose().z;
  double max_vertical_speed = 0.0;
  int samples = 0;

  while (rclcpp::ok() && SteadyClock::now() < hover_end) {
    fc.publishPositionSetpoint(target, 0.0);
    fc.spinOnce();
    const drone_control::EnuPose now = fc.localPose();
    max_drift = std::max(max_drift, std::hypot(now.x - target.x, now.y - target.y));
    max_alt_err = std::max(max_alt_err, std::fabs(now.z - target.z));
    sum_alt += now.z;
    // 20 ms 采样间隔的粗略垂直速度
    max_vertical_speed = std::max(max_vertical_speed, std::fabs(now.z - prev_z) / 0.02);
    prev_z = now.z;
    ++samples;
    std::this_thread::sleep_for(period);
  }

  const drone_control::EnuPose final_pose = fc.localPose();
  const double mean_alt = (samples > 0) ? (sum_alt / samples) : final_pose.z;

  RCLCPP_INFO(logger, "==== hover_test 结果 ====");
  RCLCPP_INFO(logger, "hovers_samples      = %d", samples);
  RCLCPP_INFO(logger, "final_altitude      = %.3f m（目标 %.3f m）", final_pose.z, hover_altitude);
  RCLCPP_INFO(logger, "mean_altitude       = %.3f m", mean_alt);
  RCLCPP_INFO(logger, "max_altitude_error  = %.3f m（容差 %.3f m）", max_alt_err,
    altitude_tolerance);
  RCLCPP_INFO(logger, "max_horizontal_drift= %.3f m（容差 %.3f m）", max_drift, drift_tolerance);
  RCLCPP_INFO(logger, "max_vertical_speed  = %.3f m/s", max_vertical_speed);

  const bool pass = climbed && (max_drift <= drift_tolerance) &&
    (std::fabs(final_pose.z - hover_altitude) <= altitude_tolerance);
  RCLCPP_INFO(logger, "hover_test RESULT: %s", pass ? "PASS" : "FAIL");

  // 3) 降落
  if (auto_land) {
    const drone_control::EnuPose land_hold{final_pose.x, final_pose.y, 0.0};
    fc.landNow(land_hold, setpoint_rate, toSec(mode_timeout));
    const bool disarmed = fc.streamUntilPredicate(land_hold, 0.0, setpoint_rate,
      toSec(land_timeout),
      [&fc]() {return !fc.armed();});
    const drone_control::EnuPose landed = fc.localPose();
    RCLCPP_INFO(logger, "降落结果：%s，落点 ENU (%.2f, %.2f, %.2f)，armed=%s",
      disarmed ? "已上锁" : "超时仍解锁", landed.x, landed.y, landed.z,
      fc.armed() ? "true" : "false");
  }

  RCLCPP_INFO(logger, "==== hover_test 结束 ====");
  rclcpp::shutdown();
  return pass ? 0 : 1;
}
