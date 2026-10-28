#include "drone_control/flight_control.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <utility>

namespace drone_control
{

namespace
{
using SteadyClock = std::chrono::steady_clock;

/// 单次自旋后休眠的时长，配合 50 Hz 控制循环不会丢回调
constexpr std::chrono::milliseconds kSpinSlice{5};

double periodMs(double rate_hz)
{
  return (rate_hz > 0.0) ? (1000.0 / rate_hz) : 1000.0;
}
}  // namespace

FlightControl::FlightControl(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
    "mavros/setpoint_raw/local", rclcpp::QoS(10));

  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    "mavros/state", rclcpp::QoS(10),
    [this](const mavros_msgs::msg::State::SharedPtr msg) {state_ = *msg;});

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "mavros/local_position/pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      pose_ = *msg;
      pose_received_ = true;
    });

  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

  RCLCPP_INFO(node_->get_logger(), "FlightControl 已就绪（setpoint 主题 %s）",
    setpoint_pub_->get_topic_name());
}

// ---------------------------------------------------------------- 连接与状态

bool FlightControl::waitForConnection(std::chrono::seconds timeout)
{
  const auto deadline = SteadyClock::now() + timeout;
  while (rclcpp::ok() && SteadyClock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (state_.connected) {
      return true;
    }
    std::this_thread::sleep_for(kSpinSlice * 4);
  }
  return state_.connected;
}

bool FlightControl::waitForLocalPose(std::chrono::seconds timeout)
{
  const auto deadline = SteadyClock::now() + timeout;
  while (rclcpp::ok() && SteadyClock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (pose_received_) {
      return true;
    }
    std::this_thread::sleep_for(kSpinSlice * 4);
  }
  return pose_received_;
}

EnuPose FlightControl::localPose() const
{
  EnuPose enu;
  enu.x = pose_.pose.position.x;
  enu.y = pose_.pose.position.y;
  enu.z = pose_.pose.position.z;
  return enu;
}

const mavros_msgs::msg::State & FlightControl::state() const
{
  return state_;
}

bool FlightControl::armed() const
{
  return state_.armed;
}

std::string FlightControl::mode() const
{
  return state_.mode;
}

// ------------------------------------------------------------------ 设定点

void FlightControl::publishPositionSetpoint(const EnuPose & enu, double yaw_rate_enu)
{
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.coordinate_frame = kFrameLocalNed;
  msg.type_mask = kTypeMaskPosition;
  // ENU(x 东, y 北, z 上) → NED(x 北, y 东, z 下)
  msg.position.x = enu.y;
  msg.position.y = enu.x;
  msg.position.z = -enu.z;
  // NED 偏航顺时针为正，故取反；当前掩码忽略 yaw_rate，保留以便后续扫描使用
  msg.yaw_rate = static_cast<float>(-yaw_rate_enu);
  setpoint_pub_->publish(msg);
}

void FlightControl::publishVelocitySetpoint(const EnuPose & enu_velocity, double yaw_rate_enu)
{
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.coordinate_frame = kFrameLocalNed;
  msg.type_mask = kTypeMaskVelocity;
  msg.velocity.x = enu_velocity.y;
  msg.velocity.y = enu_velocity.x;
  msg.velocity.z = -enu_velocity.z;
  msg.yaw_rate = static_cast<float>(-yaw_rate_enu);
  setpoint_pub_->publish(msg);
}

bool FlightControl::streamPositionSetpoint(const EnuPose & enu, double yaw_rate_enu,
  std::chrono::milliseconds duration, double rate_hz)
{
  const std::chrono::milliseconds period{static_cast<int>(periodMs(rate_hz))};
  const auto deadline = SteadyClock::now() + duration;
  while (rclcpp::ok() && SteadyClock::now() < deadline) {
    publishPositionSetpoint(enu, yaw_rate_enu);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(period);
  }
  return rclcpp::ok();
}

bool FlightControl::streamUntilPredicate(const EnuPose & enu, double yaw_rate_enu, double rate_hz,
  std::chrono::seconds timeout, const std::function<bool()> & pred)
{
  return pumpWhile(enu, yaw_rate_enu, rate_hz, timeout, pred);
}

bool FlightControl::pumpWhile(const EnuPose & enu, double yaw_rate_enu, double rate_hz,
  std::chrono::seconds timeout, const std::function<bool()> & done)
{
  const std::chrono::milliseconds period{static_cast<int>(periodMs(rate_hz))};
  const auto deadline = SteadyClock::now() + timeout;
  while (rclcpp::ok()) {
    publishPositionSetpoint(enu, yaw_rate_enu);
    rclcpp::spin_some(node_);
    if (done()) {
      return true;
    }
    if (SteadyClock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(period);
  }
  return false;
}

// ------------------------------------------------------------- 模式与解锁

bool FlightControl::setMode(const std::string & mode, std::chrono::seconds timeout)
{
  if (!set_mode_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "服务 %s 不可用", set_mode_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;

  auto future = set_mode_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "SetMode(%s) 服务调用超时", mode.c_str());
    return false;
  }
  const bool ok = future.get()->mode_sent;
  RCLCPP_INFO(node_->get_logger(), "SetMode(%s) → mode_sent=%s", mode.c_str(),
    ok ? "true" : "false");
  return ok;
}

bool FlightControl::arm(bool value, std::chrono::seconds timeout)
{
  if (!arming_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "服务 %s 不可用",
      arming_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = value;

  auto future = arming_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "arming(%s) 服务调用超时", value ? "true" : "false");
    return false;
  }
  const bool ok = future.get()->success;
  RCLCPP_INFO(node_->get_logger(), "arming(%s) → success=%s", value ? "true" : "false",
    ok ? "true" : "false");
  return ok;
}

bool FlightControl::setModeWhileStreaming(const std::string & mode, const EnuPose & hold,
  double rate_hz, std::chrono::seconds timeout)
{
  if (!set_mode_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "服务 %s 不可用", set_mode_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;
  auto future = set_mode_client_->async_send_request(request);

  bool sent = false;
  bool responded = false;
  const bool finished = pumpWhile(hold, 0.0, rate_hz, timeout, [&]() {
    if (responded) {
      return true;
    }
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }
    sent = future.get()->mode_sent;
    responded = true;
    return true;
  });
  RCLCPP_INFO(node_->get_logger(), "SetMode(%s) → mode_sent=%s", mode.c_str(),
    (finished && sent) ? "true" : "false");
  return finished && sent;
}

bool FlightControl::armWhileStreaming(bool value, const EnuPose & hold, double rate_hz,
  std::chrono::seconds timeout)
{
  if (!arming_client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "服务 %s 不可用", arming_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = value;
  auto future = arming_client_->async_send_request(request);

  bool ok = false;
  bool responded = false;
  const bool finished = pumpWhile(hold, 0.0, rate_hz, timeout, [&]() {
    if (responded) {
      return true;
    }
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }
    ok = future.get()->success;
    responded = true;
    return true;
  });
  RCLCPP_INFO(node_->get_logger(), "arming(%s) → success=%s", value ? "true" : "false",
    (finished && ok) ? "true" : "false");
  return finished && ok;
}

bool FlightControl::engageOffboard(const EnuPose & hold, const OffboardOptions & options)
{
  auto logger = node_->get_logger();

  RCLCPP_INFO(logger, "① 预发位置设定点 %ld ms @ %.0f Hz（需求书 §5.3 要求 ≥2 s）",
    static_cast<long>(options.prestream.count()), options.rate_hz);
  if (!streamPositionSetpoint(hold, 0.0, options.prestream, options.rate_hz)) {
    RCLCPP_ERROR(logger, "预发设定点失败");
    return false;
  }

  RCLCPP_INFO(logger, "② 请求 OFFBOARD");
  if (!setModeWhileStreaming("OFFBOARD", hold, options.rate_hz, options.mode_timeout)) {
    RCLCPP_ERROR(logger, "SetMode(OFFBOARD) 失败");
    return false;
  }

  if (!streamUntilPredicate(hold, 0.0, options.rate_hz, options.mode_timeout,
    [this]() {return state_.mode == "OFFBOARD";}))
  {
    RCLCPP_ERROR(logger, "OFFBOARD 未生效，当前 mode=%s", state_.mode.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "③ 模式回读为 OFFBOARD，开始解锁");

  if (!armWhileStreaming(true, hold, options.rate_hz, options.arm_timeout)) {
    RCLCPP_ERROR(logger, "解锁服务返回失败");
    return false;
  }
  if (!streamUntilPredicate(hold, 0.0, options.rate_hz, options.arm_timeout,
    [this]() {return state_.armed;}))
  {
    RCLCPP_ERROR(logger, "解锁未生效，armed=%s", state_.armed ? "true" : "false");
    return false;
  }

  RCLCPP_INFO(logger, "④ OFFBOARD 已接管，armed=true");
  return true;
}

bool FlightControl::landNow(const EnuPose & hold, double rate_hz, std::chrono::seconds timeout)
{
  RCLCPP_WARN(node_->get_logger(), "切换到 AUTO.LAND");
  return setModeWhileStreaming("AUTO.LAND", hold, rate_hz, timeout);
}

// -------------------------------------------------------------------- 自旋

void FlightControl::spinOnce()
{
  if (!rclcpp::ok()) {
    return;
  }
  rclcpp::spin_some(node_);
}

void FlightControl::spinFor(std::chrono::milliseconds duration)
{
  const auto deadline = SteadyClock::now() + duration;
  while (rclcpp::ok() && SteadyClock::now() < deadline) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(kSpinSlice);
  }
}

}  // namespace drone_control
