// mission_manager —— 模式调度 + 遥控指令 → 设定点的唯一出口。
//
// 职责（需求书 §4 职责分离铁律）：
//   * 只由本节点把速度/位置指令发到 /mavros/setpoint_raw/local；
//   * 遥控（/teleop/cmd，body 系）在这里用当前偏航转成 ENU，再交给 drone_control；
//   * 对外统一用 /mission/mode（String）与 /mission/set_mode（srv）收指令；
//   * 以 10 Hz 发布 /mission/status，验收脚本只依赖它。
//
// 控制循环是"单线程顺序流水"：spinOnce → 发设定点 → 打状态 → 睡一个周期。
// 不用 rclcpp 定时器，避免与 FlightControl 内部的 spin_some 抢执行器
// （同一个节点被两处 spin 会触发 executor 冲突）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <drone_control/flight_control.hpp>
#include <drone_interfaces/msg/mission_status.hpp>
#include <drone_interfaces/srv/set_mode.hpp>

namespace drone_mission
{

using SteadyClock = std::chrono::steady_clock;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager()
  : rclcpp::Node("mission_manager")
  {
    // 参数全部来自 drone_bringup/config/flows.yaml（代码里不写魔法数字）
    control_rate_ = declare_parameter("control_rate", 50.0);
    status_rate_ = declare_parameter("status_rate", 10.0);
    teleop_timeout_ = declare_parameter("teleop_timeout", 0.3);
    teleop_max_vel_ = declare_parameter("teleop_max_vel", 0.5);
    teleop_max_vert_ = declare_parameter("teleop_max_vert", 0.4);
    teleop_max_yaw_ = declare_parameter("teleop_max_yaw", 0.5);
    manual_altitude_ = declare_parameter("manual_altitude", 1.5);
    min_altitude_ = declare_parameter("min_altitude", 0.12);
    land_timeout_ = declare_parameter("land_timeout", 40.0);
    flow_ = declare_parameter("flow", "F1");
    takeoff_tolerance_ = declare_parameter("takeoff_tolerance", 0.15);

    status_pub_ = create_publisher<drone_interfaces::msg::MissionStatus>("/mission/status", 10);

    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/teleop/cmd", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = SteadyClock::now();
        has_cmd_ = true;
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        requestMode(msg->data);
      });

    set_mode_srv_ = create_service<drone_interfaces::srv::SetMode>(
      "/mission/set_mode",
      [this](
        const std::shared_ptr<drone_interfaces::srv::SetMode::Request> req,
        std::shared_ptr<drone_interfaces::srv::SetMode::Response> res) {
        requestMode(req->mode);
        res->success = true;
        res->message = "mode request queued: " + req->mode;
      });

    RCLCPP_INFO(get_logger(), "mission_manager 就绪：flow=%s control=%.0f Hz", flow_.c_str(),
      control_rate_);
  }

  /// 主循环：跑完返回进程退出码
  int run()
  {
    fc_ = std::make_unique<drone_control::FlightControl>(
      std::static_pointer_cast<rclcpp::Node>(shared_from_this()));

    if (!fc_->waitForConnection(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "MAVROS 未连接");
      return 1;
    }
    if (!fc_->waitForLocalPose(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "/mavros/local_position/pose 无数据");
      return 1;
    }
    RCLCPP_INFO(get_logger(), "MAVROS 已连接：mode=%s armed=%s", fc_->mode().c_str(),
      fc_->armed() ? "true" : "false");

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_));
    const auto status_period = std::chrono::milliseconds(static_cast<int>(1000.0 / status_rate_));
    auto next_status = SteadyClock::now();

    while (rclcpp::ok()) {
      fc_->spinOnce();
      executePendingCommand();
      publishControl();
      if (SteadyClock::now() >= next_status) {
        publishStatus();
        next_status = SteadyClock::now() + status_period;
      }
      std::this_thread::sleep_for(period);
    }
    return 0;
  }

private:
  void requestMode(const std::string & mode)
  {
    if (mode == mode_) {
      return;
    }
    if (mode != "MANUAL" && mode != "HOVER" && mode != "LAND" && mode != "IDLE" &&
      mode != "ARUCO_LAND" && mode != "WAYPOINT" && mode != "AUTO")
    {
      RCLCPP_WARN(get_logger(), "未知模式 %s（可选 MANUAL/HOVER/LAND/ARUCO_LAND/WAYPOINT/AUTO）",
        mode.c_str());
      return;
    }
    pending_mode_ = mode;
    has_pending_ = true;
  }

  void executePendingCommand()
  {
    if (!has_pending_) {
      return;
    }
    has_pending_ = false;
    const std::string want = pending_mode_;
    pending_mode_.clear();

    RCLCPP_INFO(get_logger(), "模式请求：%s → %s", mode_.c_str(), want.c_str());

    if (want == "MANUAL") {
      if (!fc_->armed()) {
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        if (!fc_->takeoff(manual_altitude_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "起飞失败，保持 %s", mode_.c_str());
          return;
        }
      }
      mode_ = "MANUAL";
      state_ = "MANUAL";
      return;
    }

    if (want == "HOVER") {
      if (fc_->armed()) {
        hold_ = fc_->localPose();
      }
      mode_ = "HOVER";
      state_ = "HOVER";
      return;
    }

    if (want == "LAND" || want == "IDLE") {
      mode_ = "LAND";
      state_ = "LANDING";
      if (fc_->armed()) {
        fc_->land(fc_->localPose(), control_rate_,
          std::chrono::seconds(static_cast<int>(land_timeout_)));
      }
      mode_ = "IDLE";
      state_ = "LANDED";
      return;
    }

    // M3/M4/M5 才实现，先明确告警而不是假装成功
    RCLCPP_WARN(get_logger(), "模式 %s 要到后续里程碑才实现，忽略", want.c_str());
  }

  void publishControl()
  {
    if (mode_ == "MANUAL") {
      drone_control::EnuVelocity v;
      const bool fresh = has_cmd_ &&
        (std::chrono::duration<double>(SteadyClock::now() - last_cmd_time_).count() <
        teleop_timeout_);
      if (fresh) {
        // body → ENU：Twist.linear.x 前、y 左、z 上；angular.z 偏航角速度
        const double yaw = fc_->yaw();
        const double fwd = clampAbs(latest_cmd_.linear.x, teleop_max_vel_);
        const double left = clampAbs(latest_cmd_.linear.y, teleop_max_vel_);
        v.vx = fwd * std::cos(yaw) - left * std::sin(yaw);
        v.vy = fwd * std::sin(yaw) + left * std::cos(yaw);
        v.vz = clampAbs(latest_cmd_.linear.z, teleop_max_vert_);
        yaw_rate_ = clampAbs(latest_cmd_.angular.z, teleop_max_yaw_);
      } else {
        // 松键即停：不给惯性滑行，直接零速度（PX4 在 OFFBOARD 下会保持位置）
        v = drone_control::EnuVelocity{};
        yaw_rate_ = 0.0;
      }

      // 安全兜底：贴地还往下压 → 立刻 LAND（需求书 §5.3 安全条款）
      if (fc_->localPose().z < min_altitude_ && v.vz <= 0.0) {
        RCLCPP_WARN(get_logger(), "高度 %.2f m 低于安全门限 %.2f m，强制降落",
          fc_->localPose().z, min_altitude_);
        requestMode("LAND");
        return;
      }

      fc_->publishVelocitySetpoint(drone_control::EnuPose{v.vx, v.vy, v.vz}, yaw_rate_);
      return;
    }

    if (mode_ == "HOVER") {
      fc_->publishPositionSetpoint(hold_, 0.0);
      return;
    }

    // IDLE / LAND：不发设定点，交给 PX4 自己的模式
  }

  void publishStatus()
  {
    drone_interfaces::msg::MissionStatus msg;
    msg.stamp = now();
    msg.state = state_;
    msg.mode = mode_;
    msg.flow = flow_;
    msg.altitude = static_cast<float>(fc_->localPose().z);
    msg.lateral_error = marker_lateral_error_;
    msg.vertical_speed = static_cast<float>(verticalSpeed());
    msg.marker_visible = marker_visible_;
    msg.armed = fc_->armed();
    status_pub_->publish(msg);
  }

  double verticalSpeed()
  {
    const double z = fc_->localPose().z;
    const auto t = SteadyClock::now();
    if (!has_z_) {
      has_z_ = true;
      last_z_ = z;
      last_z_time_ = t;
      return 0.0;
    }
    const double dt = std::chrono::duration<double>(t - last_z_time_).count();
    const double vz = (dt > 1e-6) ? (z - last_z_) / dt : 0.0;
    last_z_ = z;
    last_z_time_ = t;
    return vz;
  }

  static double clampAbs(double v, double limit)
  {
    return std::max(-limit, std::min(limit, v));
  }

  std::unique_ptr<drone_control::FlightControl> fc_;

  double control_rate_{50.0};
  double status_rate_{10.0};
  double teleop_timeout_{0.3};
  double teleop_max_vel_{0.5};
  double teleop_max_vert_{0.4};
  double teleop_max_yaw_{0.5};
  double manual_altitude_{1.5};
  double min_altitude_{0.12};
  double land_timeout_{40.0};
  double takeoff_tolerance_{0.15};
  std::string flow_{"F1"};

  std::string mode_{"IDLE"};
  std::string state_{"IDLE"};
  std::string pending_mode_;
  bool has_pending_{false};

  geometry_msgs::msg::Twist latest_cmd_;
  SteadyClock::time_point last_cmd_time_{};
  bool has_cmd_{false};
  double yaw_rate_{0.0};

  drone_control::EnuPose hold_{};
  float marker_lateral_error_{-1.0F};
  bool marker_visible_{false};

  double last_z_{0.0};
  SteadyClock::time_point last_z_time_{};
  bool has_z_{false};

  rclcpp::Publisher<drone_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Service<drone_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
};

}  // namespace drone_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_mission::MissionManager>();
  const int code = node->run();
  rclcpp::shutdown();
  return code;
}
