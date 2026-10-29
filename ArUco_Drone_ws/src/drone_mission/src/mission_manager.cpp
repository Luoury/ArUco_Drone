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
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    manual_max_altitude_ = declare_parameter("manual_max_altitude", 5.0);
    land_timeout_ = declare_parameter("land_timeout", 40.0);
    flow_ = declare_parameter("flow", "F1");
    takeoff_tolerance_ = declare_parameter("takeoff_tolerance", 0.15);
    // 视觉链路的三个外部信号（M3 起用；默认值见 flows.yaml）
    marker_lost_timeout_ = declare_parameter("marker_lost_timeout", 0.5);
    mode_switch_hover_ = declare_parameter("mode_switch_hover", 0.5);
    landing_start_timeout_ = declare_parameter("landing_start_timeout", 5.0);
    const std::string marker_topic = declare_parameter("marker_topic", "/aruco/detection");
    const std::string landing_state_topic =
      declare_parameter("landing_state_topic", "/aruco/landing/state");
    // 航点链路（M4 起用；waypoint_nav 是唯一发航点设定点的节点）
    waypoint_start_timeout_ = declare_parameter("waypoint_start_timeout", 5.0);
    const std::string waypoint_state_topic =
      declare_parameter("waypoint_state_topic", "/waypoint/state");
    const std::string waypoint_progress_topic =
      declare_parameter("waypoint_progress_topic", "/waypoint/progress");
    // 多工位巡回（TOUR，演示用）：按 tour_stations 顺序逐个飞到工位正上方 →
    // 交给 landing_controller 视觉降落 → 落地上锁后原地起飞 → 下一个工位。
    // 每个工位一个 ArUco 码，检测端用 marker_id:=-1（任意码，取画面中心最近的）。
    tour_goto_tol_ = declare_parameter("tour_goto_tol", 0.35);
    tour_settle_hold_ = declare_parameter("tour_settle_hold", 1.0);
    tour_goto_timeout_ = declare_parameter("tour_goto_timeout", 45.0);
    tour_land_timeout_ = declare_parameter("tour_land_timeout", 90.0);
    tour_takeoff_alt_ = declare_parameter("tour_takeoff_alt", 2.0);
    tour_takeoff_timeout_ = declare_parameter("tour_takeoff_timeout", 40.0);
    tour_approach_alt_ = declare_parameter("tour_approach_alt", 2.0);
    tour_stations_ = declare_parameter<std::vector<double>>(
      "tour_stations", std::vector<double>{});
    if (!tour_stations_.empty() && tour_stations_.size() % 3 != 0) {
      RCLCPP_ERROR(get_logger(),
        "tour_stations 长度 %zu 不是 3 的倍数（应为 x,y,z 三元组），TOUR 会被拒绝",
        tour_stations_.size());
      tour_stations_.clear();
    }

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

    // 检测结果：只用来填 MissionStatus 的 marker_visible / lateral_error，
    // 真正的降落决策在 landing_controller 里（职责分离，需求书 §4）
    const auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
      rmw_qos_profile_sensor_data));
    marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      marker_topic, sensor_qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        marker_ = *msg;
        marker_seen_ = SteadyClock::now();
        has_marker_ = true;
      });

    landing_state_sub_ = create_subscription<std_msgs::msg::String>(
      landing_state_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        landing_state_ = msg->data;
      });

    landing_start_client_ = create_client<std_srvs::srv::Trigger>("/aruco/landing/start");
    landing_stop_client_ = create_client<std_srvs::srv::Trigger>("/aruco/landing/stop");

    waypoint_state_sub_ = create_subscription<std_msgs::msg::String>(
      waypoint_state_topic, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        waypoint_state_ = msg->data;
      });
    waypoint_progress_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      waypoint_progress_topic, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
          waypoint_index_ = static_cast<int>(msg->data[0]);
          waypoint_total_ = static_cast<int>(msg->data[1]);
          waypoint_error_ = static_cast<float>(msg->data[2]);
        }
      });
    waypoint_start_client_ = create_client<std_srvs::srv::Trigger>("/waypoint/start");
    waypoint_stop_client_ = create_client<std_srvs::srv::Trigger>("/waypoint/stop");

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
      mode != "ARUCO_LAND" && mode != "WAYPOINT" && mode != "AUTO" && mode != "TOUR")
    {
      RCLCPP_WARN(get_logger(),
        "未知模式 %s（可选 MANUAL/HOVER/LAND/ARUCO_LAND/WAYPOINT/AUTO/TOUR）", mode.c_str());
      return;
    }
    pending_mode_ = mode;
    has_pending_ = true;
  }

  /// 原地悬停指定秒数（需求书 §5.5：切模式前先悬停稳定，避免两个控制器抢设定点）
  void hoverFor(double seconds)
  {
    const auto until = SteadyClock::now() +
      std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    while (rclcpp::ok() && SteadyClock::now() < until) {
      fc_->spinOnce();
      fc_->publishPositionSetpoint(hold_, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
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
      teleop_target_ = fc_->localPose();
      teleop_target_valid_ = true;
      RCLCPP_INFO(get_logger(), "MANUAL 目标起点 = (%.2f, %.2f, %.2f)",
        teleop_target_.x, teleop_target_.y, teleop_target_.z);
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

    if (want == "WAYPOINT") {
      // 与 ARUCO_LAND 同一套交接纪律（需求书 §5.5）：先起飞 → 置 HOVER → 等 0.5 s
      // → 把设定点所有权交给 waypoint_nav。
      if (!fc_->armed()) {
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        if (!fc_->takeoff(manual_altitude_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "起飞失败，保持 %s", mode_.c_str());
          return;
        }
      }
      hold_ = fc_->localPose();
      mode_ = "HOVER";
      state_ = "SWITCHING";
      const auto until = SteadyClock::now() +
        std::chrono::milliseconds(static_cast<int>(mode_switch_hover_ * 1000.0));
      while (rclcpp::ok() && SteadyClock::now() < until) {
        fc_->spinOnce();
        fc_->publishPositionSetpoint(hold_, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!callTrigger("/waypoint/start", waypoint_start_client_, waypoint_start_timeout_)) {
        RCLCPP_ERROR(get_logger(), "WAYPOINT 启动失败（waypoint_nav 可能没起）");
        mode_ = "HOVER";
        state_ = "HOVER";
        return;
      }
      mode_ = "WAYPOINT";
      state_ = "WAYPOINT";
      return;
    }

    if (want == "AUTO") {
      // 全自动链路（需求书 §5.6 F4）：起飞 → 航点（终点 = 码正上方）→ 搜码降落。
      // 阶段 1 委托 waypoint_nav，阶段 2 委托 landing_controller；
      // 本节点的职责只有"交接 + 盯着阶段推进"，绝不与它们抢设定点。
      if (!fc_->armed()) {
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        if (!fc_->takeoff(manual_altitude_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "起飞失败，保持 %s", mode_.c_str());
          return;
        }
      }
      hold_ = fc_->localPose();
      mode_ = "HOVER";
      state_ = "SWITCHING";
      hoverFor(mode_switch_hover_);
      if (!callTrigger("/waypoint/start", waypoint_start_client_, waypoint_start_timeout_)) {
        RCLCPP_ERROR(get_logger(), "AUTO 阶段 1/2 启动失败（waypoint_nav 没起）");
        mode_ = "HOVER";
        state_ = "HOVER";
        return;
      }
      auto_phase_ = "WAYPOINT";
      auto_begin_ = SteadyClock::now();
      mode_ = "AUTO";
      state_ = "AUTO_WAYPOINT";
      RCLCPP_INFO(get_logger(), "AUTO 阶段 1/2：航点导航（终点 = 码正上方）");
      return;
    }

    if (want == "TOUR") {
      const size_t stations = tour_stations_.size() / 3;
      if (stations == 0) {
        RCLCPP_ERROR(get_logger(), "TOUR 需要 tour_stations 参数（x,y,z 三元组列表）");
        return;
      }
      if (!fc_->armed()) {
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        if (!fc_->takeoff(tour_takeoff_alt_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "TOUR 起飞失败，保持 %s", mode_.c_str());
          return;
        }
      }
      hold_ = fc_->localPose();
      mode_ = "HOVER";
      state_ = "SWITCHING";
      hoverFor(mode_switch_hover_);
      tour_index_ = 0;
      tour_phase_ = "GOTO";
      tour_dwell_ = 0.0;
      tour_begin_ = SteadyClock::now();
      tour_phase_begin_ = tour_begin_;
      mode_ = "TOUR";
      state_ = "TOUR_GOTO";
      RCLCPP_INFO(get_logger(), "TOUR 开始：共 %zu 个工位，第 1 站 (%.2f, %.2f)",
        stations, tour_stations_[0], tour_stations_[1]);
      return;
    }

    if (want == "ARUCO_LAND") {
      // 需求书 §5.5：先停当前输出 → 置 HOVER → 等 0.5 s → 再交给 landing_controller，
      // 避免两个节点同时发 setpoint 打架。
      if (!fc_->armed()) {
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        if (!fc_->takeoff(manual_altitude_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "起飞失败，保持 %s", mode_.c_str());
          return;
        }
      }
      hold_ = fc_->localPose();
      mode_ = "HOVER";
      state_ = "SWITCHING";
      const auto until = SteadyClock::now() +
        std::chrono::milliseconds(static_cast<int>(mode_switch_hover_ * 1000.0));
      while (rclcpp::ok() && SteadyClock::now() < until) {
        fc_->spinOnce();
        fc_->publishPositionSetpoint(hold_, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!callTrigger("/aruco/landing/start", landing_start_client_,
        landing_start_timeout_))
      {
        RCLCPP_ERROR(get_logger(), "ARUCO_LAND 启动失败（landing_controller 可能没起）");
        mode_ = "HOVER";
        state_ = "HOVER";
        return;
      }
      mode_ = "ARUCO_LAND";
      state_ = "ARUCO_LAND";
      return;
    }

    if (want == "IDLE" || want == "HOVER" || want == "MANUAL" || want == "LAND") {
      // 从航点模式切出去时先叫停 waypoint_nav，否则它会继续发航点设定点
      if (mode_ == "WAYPOINT") {
        callTrigger("/waypoint/stop", waypoint_stop_client_, waypoint_start_timeout_);
        waypoint_state_ = "IDLE";
      }
      // 从 TOUR 切出去时，先叫停 landing_controller
      if (mode_ == "TOUR") {
        tour_phase_.clear();
        if (landing_active()) {
          callTrigger("/aruco/landing/stop", landing_stop_client_, landing_start_timeout_);
          landing_state_ = "IDLE";
        }
      }
      // 从 AUTO 切出去时，两个被委托的节点都要叫停
      if (mode_ == "AUTO") {
        auto_phase_.clear();
        callTrigger("/waypoint/stop", waypoint_stop_client_, waypoint_start_timeout_);
        waypoint_state_ = "IDLE";
      }
      // 从降落模式切出去时，先叫停 landing_controller，否则它会继续发设定点
      if (landing_active()) {
        callTrigger("/aruco/landing/stop", landing_stop_client_, landing_start_timeout_);
        landing_state_ = "IDLE";
      }
    }

    // M4/M5 才实现，先明确告警而不是假装成功
    RCLCPP_WARN(get_logger(), "模式 %s 要到后续里程碑才实现，忽略", want.c_str());
  }

  bool landing_active() const
  {
    return landing_state_ != "IDLE" && landing_state_ != "DONE" && !landing_state_.empty();
  }

  /// 同步调用外部控制器的 Trigger 服务（本节点是单线程流水，直接自旋等结果）
  bool callTrigger(const std::string & name,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, double timeout_s)
  {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "服务 %s 不可用", name.c_str());
      return false;
    }
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline = SteadyClock::now() +
      std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0));
    while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(10)) !=
      std::future_status::ready)
    {
      fc_->spinOnce();
      if (SteadyClock::now() > deadline) {
        RCLCPP_ERROR(get_logger(), "服务 %s 调用超时", name.c_str());
        return false;
      }
    }
    const auto response = future.get();
    RCLCPP_INFO(get_logger(), "%s → success=%s message=%s", name.c_str(),
      response->success ? "true" : "false", response->message.c_str());
    return response->success;
  }

  void publishControl()
  {
    // AUTO：两个阶段都委托给别人，本节点只盯着阶段推进，一个字都不许发
    if (mode_ == "AUTO") {
      monitorAuto();
      return;
    }

    // TOUR：飞到工位阶段由本节点发位置设定点；降落阶段交给 landing_controller
    if (mode_ == "TOUR") {
      monitorTour();
      return;
    }

    // ARUCO_LAND：设定点由 landing_controller 独占，本节点必须停止发指令
    // （否则两个发布者抢 /mavros/setpoint_raw/local，无人机会抖）
    if (mode_ == "ARUCO_LAND") {
      monitorLanding();
      return;
    }

    // WAYPOINT：设定点由 waypoint_nav 独占，本节点只看它的状态
    if (mode_ == "WAYPOINT") {
      monitorWaypoint();
      return;
    }

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

      if (!teleop_target_valid_) {
        teleop_target_ = fc_->localPose();
        teleop_target_valid_ = true;
      }

      // 速度积分成位置目标：水平用手柄速度推进，高度靠位置环自然定高
      const double dt = 1.0 / control_rate_;
      teleop_target_.x += v.vx * dt;
      teleop_target_.y += v.vy * dt;
      teleop_target_.z = clampAbs(teleop_target_.z + v.vz * dt, manual_max_altitude_);

      // 安全兜底：贴地还往下压 → 立刻 LAND（需求书 §5.3 安全条款）
      if (fc_->localPose().z < min_altitude_ && v.vz <= 0.0) {
        RCLCPP_WARN(get_logger(), "高度 %.2f m 低于安全门限 %.2f m，强制降落",
          fc_->localPose().z, min_altitude_);
        requestMode("LAND");
        return;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "MANUAL 设定点 = (%.2f, %.2f, %.2f) 当前 z = %.2f 指令 = (%.2f, %.2f, %.2f)",
        teleop_target_.x, teleop_target_.y, teleop_target_.z, fc_->localPose().z,
        v.vx, v.vy, v.vz);
      fc_->publishPositionSetpoint(teleop_target_, yaw_rate_);
      return;
    }

    if (mode_ == "HOVER") {
      fc_->publishPositionSetpoint(hold_, 0.0);
      return;
    }

    // IDLE / LAND：不发设定点，交给 PX4 自己的模式
  }

  /// 监视 AUTO 的两个阶段：航点跑完 → 交给视觉降落 → 落地上锁 → 收工
  void monitorAuto()
  {
    const double spent = std::chrono::duration<double>(
      SteadyClock::now() - auto_begin_).count();
    if (spent > auto_timeout_) {
      RCLCPP_WARN(get_logger(), "AUTO 超过兜底超时 %.0f s（此时阶段 %s），强制降落",
        auto_timeout_, auto_phase_.c_str());
      auto_phase_.clear();
      mode_ = "LAND";
      state_ = "AUTO_TIMEOUT";
      fc_->land(fc_->localPose(), control_rate_,
        std::chrono::seconds(static_cast<int>(land_timeout_)));
      mode_ = "IDLE";
      state_ = "LANDED";
      return;
    }

    if (auto_phase_ == "WAYPOINT") {
      if (waypoint_state_ != "DONE") {
        return;
      }
      waypoint_state_ = "IDLE";
      RCLCPP_INFO(get_logger(), "AUTO 阶段 1/2 完成：航点全部到达（%.1f s）", spent);
      // 必须显式叫停 waypoint_nav：它在 DONE 状态下仍会发一会儿位置设定点，
      // 与 landing_controller 的速度设定点抢同一个话题（F4 第一次跑就栽在这里）。
      callTrigger("/waypoint/stop", waypoint_stop_client_, waypoint_start_timeout_);
      hold_ = fc_->localPose();
      hoverFor(mode_switch_hover_);
      if (!callTrigger("/aruco/landing/start", landing_start_client_,
        landing_start_timeout_))
      {
        RCLCPP_ERROR(get_logger(), "AUTO 阶段 2/2 启动失败（landing_controller 没起）→ 悬停");
        auto_phase_.clear();
        mode_ = "HOVER";
        state_ = "AUTO_ABORT";
        return;
      }
      auto_phase_ = "LAND";
      state_ = "AUTO_LAND";
      RCLCPP_INFO(get_logger(), "AUTO 阶段 2/2：视觉降落已接管");
      return;
    }

    if (auto_phase_ == "LAND" && landing_state_ == "DONE") {
      landing_state_ = "IDLE";
      auto_phase_.clear();
      mode_ = "IDLE";
      state_ = "AUTO_DONE";
      RCLCPP_INFO(get_logger(), "AUTO 全流程完成：已落地并上锁（总耗时 %.1f s）", spent);
    }
  }

  /// 多工位巡回：GOTO（本节点发位置设定点）→ LAND（交给 landing_controller）
  /// → 落地上锁 → 原地起飞 → 下一个工位。最后一站落地上锁后收工。
  void monitorTour()
  {
    const size_t stations = tour_stations_.size() / 3;
    if (tour_index_ >= stations) {
      mode_ = "IDLE";
      state_ = "TOUR_DONE";
      return;
    }

    const double sx = tour_stations_[tour_index_ * 3 + 0];
    const double sy = tour_stations_[tour_index_ * 3 + 1];
    const double spent = std::chrono::duration<double>(
      SteadyClock::now() - tour_phase_begin_).count();

    if (tour_phase_ == "GOTO") {
      drone_control::EnuPose target;
      target.x = sx;
      target.y = sy;
      target.z = tour_approach_alt_;
      fc_->publishPositionSetpoint(target, 0.0);
      const auto p = fc_->localPose();
      const double dx = p.x - sx;
      const double dy = p.y - sy;
      const double dz = p.z - tour_approach_alt_;
      const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (err <= tour_goto_tol_) {
        tour_dwell_ += 1.0 / control_rate_;
      } else {
        tour_dwell_ = 0.0;
      }
      if (tour_dwell_ >= tour_settle_hold_) {
        tour_dwell_ = 0.0;
        tour_phase_ = "LAND";
        tour_phase_begin_ = SteadyClock::now();
        state_ = "TOUR_LAND";
        // 清掉上一站留下的 DONE，否则新一站会在几十毫秒内被误判成"已落地"
        landing_state_.clear();
        RCLCPP_INFO(get_logger(), "TOUR %zu/%zu 到站（误差 %.2f m），交给视觉降落",
          tour_index_ + 1, stations, err);
        if (!callTrigger("/aruco/landing/start", landing_start_client_,
          landing_start_timeout_))
        {
          RCLCPP_ERROR(get_logger(), "TOUR 到站后启动降落失败 → 悬停");
          tour_phase_.clear();
          mode_ = "HOVER";
          state_ = "TOUR_ABORT";
        }
        return;
      }
      if (spent > tour_goto_timeout_) {
        RCLCPP_WARN(get_logger(), "TOUR 第 %zu 站飞行超时 %.0f s（误差 %.2f m）→ 直接尝试降落",
          tour_index_ + 1, spent, err);
        tour_phase_ = "LAND";
        tour_phase_begin_ = SteadyClock::now();
        state_ = "TOUR_LAND";
        callTrigger("/aruco/landing/start", landing_start_client_, landing_start_timeout_);
      }
      return;
    }

    if (tour_phase_ == "LAND") {
      // 只在降落真正开始 1 s 之后才承认 DONE：服务调用返回与新状态之间存在竞争
      if (landing_state_ == "DONE" && spent > 1.0) {
        landing_state_ = "IDLE";
        RCLCPP_INFO(get_logger(), "TOUR %zu/%zu 已落到工位码上（%.1f s）",
          tour_index_ + 1, stations, spent);
        if (tour_index_ + 1 >= stations) {
          const double total = std::chrono::duration<double>(
            SteadyClock::now() - tour_begin_).count();
          tour_phase_.clear();
          mode_ = "IDLE";
          state_ = "TOUR_DONE";
          RCLCPP_INFO(get_logger(), "TOUR 全部 %zu 个工位完成（总耗时 %.1f s），已回到基站",
            stations, total);
          return;
        }
        ++tour_index_;
        tour_phase_ = "TAKEOFF";
        tour_phase_begin_ = SteadyClock::now();
        state_ = "TOUR_TAKEOFF";
        drone_control::OffboardOptions options;
        options.rate_hz = control_rate_;
        options.mode_timeout = std::chrono::seconds(
          static_cast<int>(tour_takeoff_timeout_));
        if (!fc_->takeoff(tour_takeoff_alt_, options, takeoff_tolerance_)) {
          RCLCPP_ERROR(get_logger(), "TOUR 第 %zu 站起飞失败 → 悬停", tour_index_ + 1);
          tour_phase_.clear();
          mode_ = "HOVER";
          state_ = "TOUR_ABORT";
          return;
        }
        tour_phase_ = "GOTO";
        tour_phase_begin_ = SteadyClock::now();
        state_ = "TOUR_GOTO";
        RCLCPP_INFO(get_logger(), "TOUR 前往第 %zu/%zu 站 (%.2f, %.2f)",
          tour_index_ + 1, stations, tour_stations_[tour_index_ * 3],
          tour_stations_[tour_index_ * 3 + 1]);
        return;
      }
      if (spent > tour_land_timeout_) {
        RCLCPP_WARN(get_logger(), "TOUR 第 %zu 站降落超时 %.0f s → 悬停",
          tour_index_ + 1, spent);
        callTrigger("/aruco/landing/stop", landing_stop_client_, landing_start_timeout_);
        landing_state_ = "IDLE";
        tour_phase_.clear();
        mode_ = "HOVER";
        state_ = "TOUR_ABORT";
      }
      return;
    }
  }

  /// 监视 landing_controller：它落地上锁后（DONE）本节点收回控制权
  void monitorLanding()
  {
    if (landing_state_ == "DONE") {
      RCLCPP_INFO(get_logger(), "landing_controller 已 DONE，模式回 IDLE");
      landing_state_ = "IDLE";
      mode_ = "IDLE";
      state_ = "LANDED";
    }
  }

  /// 监视 waypoint_nav：它跑完（DONE）后本节点收回控制权，就地悬停
  void monitorWaypoint()
  {
    if (waypoint_state_ == "DONE") {
      RCLCPP_INFO(get_logger(), "waypoint_nav 已 DONE，模式回 HOVER（%d 个航点）", waypoint_total_);
      waypoint_state_ = "IDLE";
      hold_ = fc_->localPose();
      mode_ = "HOVER";
      state_ = "WAYPOINT_DONE";
    } else if (waypoint_state_ == "IDLE" && state_ == "WAYPOINT") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waypoint_nav 报 IDLE，可能已停止");
    }
  }

  void publishStatus()
  {
    drone_interfaces::msg::MissionStatus msg;
    msg.stamp = now();
    msg.state = state_;
    msg.mode = mode_;
    msg.flow = flow_;
    const drone_control::EnuPose p = fc_->localPose();
    msg.altitude = static_cast<float>(p.z);
    const double since_marker =
      std::chrono::duration<double>(SteadyClock::now() - marker_seen_).count();
    marker_visible_ = has_marker_ && since_marker <= marker_lost_timeout_;
    if (marker_visible_) {
      marker_lateral_error_ = static_cast<float>(std::hypot(
        marker_.pose.position.x - p.x, marker_.pose.position.y - p.y));
    } else {
      marker_lateral_error_ = -1.0F;
    }
    // WAYPOINT 模式下 lateral_error 指"到当前航点的距离"，便于验收脚本对照
    if (mode_ == "WAYPOINT") {
      msg.lateral_error = waypoint_error_;
    } else {
      msg.lateral_error = marker_lateral_error_;
    }
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
  double manual_max_altitude_{5.0};
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
  // MANUAL 用"速度积分成位置目标"的方式发位置设定点：
  // 位置设定点自带定高，避免速度模式 vz=0 掉高（troubleshooting 第 20 条实测 2.06→0.98 m）
  drone_control::EnuPose teleop_target_{};
  bool teleop_target_valid_{false};
  float marker_lateral_error_{-1.0F};
  bool marker_visible_{false};

  // 视觉链路（M3）
  double marker_lost_timeout_{0.5};
  double mode_switch_hover_{0.5};
  double landing_start_timeout_{5.0};
  geometry_msgs::msg::PoseStamped marker_;
  SteadyClock::time_point marker_seen_{};
  bool has_marker_{false};
  std::string landing_state_{"IDLE"};

  // 航点链路（M4）
  double waypoint_start_timeout_{5.0};
  // 多工位巡回（TOUR）
  std::vector<double> tour_stations_;   // 扁平 x,y 三元组（z 取 tour_approach_alt_）
  size_t tour_index_{0};
  std::string tour_phase_;              // "GOTO" / "LAND" / "TAKEOFF"
  double tour_dwell_{0.0};
  SteadyClock::time_point tour_phase_begin_{};
  SteadyClock::time_point tour_begin_{};
  double tour_goto_tol_{0.35};
  double tour_settle_hold_{1.0};
  double tour_goto_timeout_{45.0};
  double tour_land_timeout_{90.0};
  double tour_takeoff_alt_{2.0};
  double tour_takeoff_timeout_{40.0};
  double tour_approach_alt_{2.0};
  double auto_timeout_{120.0};      // s，AUTO 全链路兜底超时（超了就强制降落）
  std::string auto_phase_;          // AUTO 内部阶段："WAYPOINT" / "LAND"
  SteadyClock::time_point auto_begin_{};
  std::string waypoint_state_{"IDLE"};
  int waypoint_index_{0};
  int waypoint_total_{0};
  float waypoint_error_{-1.0F};

  double last_z_{0.0};
  SteadyClock::time_point last_z_time_{};
  bool has_z_{false};

  rclcpp::Publisher<drone_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Service<drone_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr landing_state_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr landing_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr landing_stop_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waypoint_progress_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr waypoint_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr waypoint_stop_client_;
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
