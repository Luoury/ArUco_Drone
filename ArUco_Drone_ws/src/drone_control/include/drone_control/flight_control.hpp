#ifndef DRONE_CONTROL__FLIGHT_CONTROL_HPP_
#define DRONE_CONTROL__FLIGHT_CONTROL_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

namespace drone_control
{

/// setpoint_raw/local.type_mask 固定值（需求书 §5.2，禁止修改）
/// 位定义：bit0-2 位置，bit3-5 速度，bit6-8 加速度，bit9 力矩，bit10 yaw，bit11 yaw_rate
/// 置 1 表示"忽略该量"。下面两个掩码都忽略 yaw/yaw_rate，由 PX4 保持当前朝向。
constexpr std::uint16_t kTypeMaskPosition = 0b0000111111111000U;  // 只用 position
constexpr std::uint16_t kTypeMaskVelocity = 0b0000111111000111U;  // 只用 vx, vy, vz

/// mavlink 坐标系枚举；PX4 不接受 LOCAL_ENU(8)，必须发 LOCAL_NED(1)
constexpr std::uint8_t kFrameLocalNed = 1U;

/// ENU 位置（x 东、y 北、z 上），单位 m
struct EnuPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// OFFBOARD 接管参数（需求书 §5.3 前置顺序）
struct OffboardOptions
{
  std::chrono::milliseconds prestream{2000};
  double rate_hz{50.0};
  std::chrono::seconds mode_timeout{10};
  std::chrono::seconds arm_timeout{10};
};

/// 基于 MAVROS 的飞行控制原语：设定点发布、ENU↔NED、模式切换、解锁。
/// 不持有自己的节点，直接用调用方传入的节点（便于在 launch 中共享参数与时钟）。
class FlightControl
{
public:
  explicit FlightControl(const rclcpp::Node::SharedPtr & node);

  // --- 连接与状态 ---------------------------------------------------------
  bool waitForConnection(std::chrono::seconds timeout);
  bool waitForLocalPose(std::chrono::seconds timeout);
  EnuPose localPose() const;
  const mavros_msgs::msg::State & state() const;
  bool armed() const;
  std::string mode() const;

  // --- 设定点 -------------------------------------------------------------
  void publishPositionSetpoint(const EnuPose & enu, double yaw_rate_enu);
  void publishVelocitySetpoint(const EnuPose & enu_velocity, double yaw_rate_enu);

  /// 以固定频率连续发布同一位置设定点，持续 duration
  bool streamPositionSetpoint(const EnuPose & enu, double yaw_rate_enu,
                              std::chrono::milliseconds duration, double rate_hz);

  /// 在持续发布设定点的同时等待 pred 成立（防止 PX4 因设定点中断而掉出 OFFBOARD）
  bool streamUntilPredicate(const EnuPose & enu, double yaw_rate_enu, double rate_hz,
                            std::chrono::seconds timeout, const std::function<bool()> & pred);

  // --- 模式与解锁 ---------------------------------------------------------
  bool setMode(const std::string & mode, std::chrono::seconds timeout);
  bool arm(bool value, std::chrono::seconds timeout);

  /// 边发设定点边请求模式切换，避免切换期间设定点断流
  bool setModeWhileStreaming(const std::string & mode, const EnuPose & hold, double rate_hz,
                             std::chrono::seconds timeout);
  bool armWhileStreaming(bool value, const EnuPose & hold, double rate_hz,
                         std::chrono::seconds timeout);

  /// OFFBOARD 接管完整前置流程（需求书 §5.3）：
  /// 预发设定点 → 切 OFFBOARD → 等模式回读 → 解锁 → 等 armed
  bool engageOffboard(const EnuPose & hold, const OffboardOptions & options);

  /// 立即切 AUTO.LAND（安全兜底）
  bool landNow(const EnuPose & hold, double rate_hz, std::chrono::seconds timeout);

  // --- 自旋 ---------------------------------------------------------------
  void spinOnce();
  void spinFor(std::chrono::milliseconds duration);

private:
  bool pumpWhile(const EnuPose & enu, double yaw_rate_enu, double rate_hz,
                 std::chrono::seconds timeout, const std::function<bool()> & done);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped pose_;
  bool pose_received_{false};
};

}  // namespace drone_control

#endif  // DRONE_CONTROL__FLIGHT_CONTROL_HPP_
