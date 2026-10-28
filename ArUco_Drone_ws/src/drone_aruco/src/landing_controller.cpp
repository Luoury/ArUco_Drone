// 视觉伺服降落状态机（需求书 §5.3）
//
//    IDLE ──start()──▶ SEARCH ──marker──▶ ALIGN ──|err|<lateral_tol 连续 N 帧──▶ DESCEND
//      ▲                 ▲                   │                                     │ alt<final_alt
//      │                 │                   └──|err|>0.40 m ─────────┐            ▼
//      │                 └── marker lost > lost_timeout ── hover 1 s ─┘          FINAL
//      │                                                                          │ alt<touchdown_alt
//      └──────────── DONE ◀── 上锁 ◀──────────────────────────────────────── TOUCHDOWN
//
// 落地即上锁；任一时刻 alt < safety_alt 或收到 stop() → 立即 LAND。
//
// 几个实测教训（别改回去）：
//   * 设定点一律 ENU，ENU→NED 由 mavros 做（重复转一次会把 1.5 m 变成 -1.5 m，见需求书 §6.4 坑 1）；
//   * 速度模式 vz=0 不保高（实测 8 s 掉 1 m），所以 ALIGN 用 vz = kp_z*(align_alt - alt) 闭环；
//   * 位置模式无法从地面起飞，未解锁时先走 AUTO.TAKEOFF（§6.4 坑 4）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "drone_control/flight_control.hpp"

namespace drone_aruco
{

using SteadyClock = std::chrono::steady_clock;

/// 降落阶段（字符串同时作为 /aruco/landing/state 的取值，便于外部脚本判定）
constexpr const char * kStateIdle = "IDLE";
constexpr const char * kStateSearch = "SEARCH";
constexpr const char * kStateAlign = "ALIGN";
constexpr const char * kStateDescend = "DESCEND";
constexpr const char * kStateFinal = "FINAL";
constexpr const char * kStateTouchdown = "TOUCHDOWN";
constexpr const char * kStateDone = "DONE";

class LandingController : public rclcpp::Node
{
public:
  LandingController()
  : rclcpp::Node("landing_controller")
  {
    // ---- 需求书 §5.3 固定参数名 ----------------------------------------
    align_alt_ = declare_parameter<double>("align_alt", 2.5);
    descend_speed_ = declare_parameter<double>("descend_speed", 0.30);
    final_speed_ = declare_parameter<double>("final_speed", 0.12);
    lateral_tol_ = declare_parameter<double>("lateral_tol", 0.15);
    final_tol_ = declare_parameter<double>("final_tol", 0.08);
    kp_xy_ = declare_parameter<double>("kp_xy", 0.8);
    max_vel_xy_ = declare_parameter<double>("max_vel_xy", 0.6);
    lost_timeout_ = declare_parameter<double>("lost_timeout", 0.5);
    search_yaw_rate_ = declare_parameter<double>("search_yaw_rate", 0.3);

    // ---- 实现细节参数（同样只在 landing.yaml 里出现，代码不写魔法数）----
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    kp_z_ = declare_parameter<double>("kp_z", 0.8);
    max_vz_ = declare_parameter<double>("max_vz", 0.30);
    align_frames_ = declare_parameter<int>("align_frames", 10);
    align_abort_tol_ = declare_parameter<double>("align_abort_tol", 2.50);
    final_alt_ = declare_parameter<double>("final_alt", 0.5);
    touchdown_alt_ = declare_parameter<double>("touchdown_alt", 0.15);
    safety_alt_ = declare_parameter<double>("safety_alt", 0.12);
    relost_hover_ = declare_parameter<double>("relost_hover", 1.0);
    search_timeout_ = declare_parameter<double>("search_timeout", 60.0);
    land_timeout_ = declare_parameter<double>("land_timeout", 40.0);
    prestream_ = declare_parameter<double>("prestream", 2.0);
    takeoff_tolerance_ = declare_parameter<double>("takeoff_tolerance", 0.15);
    takeoff_timeout_ = declare_parameter<double>("takeoff_timeout", 30.0);
    const std::string detection_topic =
      declare_parameter<std::string>("detection_topic", "/aruco/detection");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/aruco/landing/state");

    // 检测是"感知"数据：与 aruco_detector 的发布端保持 best_effort 一致
    const auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
      rmw_qos_profile_sensor_data));
    detection_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      detection_topic, sensor_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {onDetection(msg);});
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/aruco/landing/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (state_ != kStateIdle && state_ != kStateDone) {
          response->success = false;
          response->message = std::string("降落已在运行，当前阶段 ") + state_;
          return;
        }
        requested_start_ = true;
        response->success = true;
        response->message = "已受理，等待控制回路进入 SEARCH";
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/aruco/landing/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        requested_stop_ = true;
        response->success = true;
        response->message = "已受理，立即 LAND";
      });

    RCLCPP_INFO(get_logger(),
      "降落控制器就绪：align_alt=%.2f descend=%.2f final=%.2f lateral_tol=%.2f final_tol=%.2f "
      "kp_xy=%.2f max_vel_xy=%.2f", align_alt_, descend_speed_, final_speed_, lateral_tol_,
      final_tol_, kp_xy_, max_vel_xy_);
  }

  /// 主流水：先 spinOnce 到齐所有回调，再走一步状态机
  void run()
  {
    fc_ = std::make_unique<drone_control::FlightControl>(shared_from_this());
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    while (rclcpp::ok()) {
      fc_->spinOnce();
      tick();
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(period));
    }
  }

private:
  // --- 回调 -------------------------------------------------------------
  void onDetection(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    marker_ = *msg;
    has_marker_ = true;
    last_detection_ = SteadyClock::now();
  }

  bool markerFresh() const
  {
    if (!has_marker_) {return false;}
    const double age = std::chrono::duration<double>(SteadyClock::now() - last_detection_).count();
    return age <= lost_timeout_;
  }

  /// 水平误差（机体系）：marker 相对无人机的位置，旋到机体轴
  void lateralErrorBody(double & ex_b, double & ey_b, double & lateral) const
  {
    const drone_control::EnuPose p = fc_->localPose();
    const double yaw = fc_->yaw();
    geometry_msgs::msg::PoseStamped marker;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      marker = marker_;
    }
    const double dx = marker.pose.position.x - p.x;
    const double dy = marker.pose.position.y - p.y;
    ex_b = std::cos(yaw) * dx + std::sin(yaw) * dy;    // 前
    ey_b = -std::sin(yaw) * dx + std::cos(yaw) * dy;  // 左
    lateral = std::hypot(dx, dy);
  }

  static double clamp(double v, double limit)
  {
    return std::max(-limit, std::min(limit, v));
  }

  /// 水平速度控制：像素等价律 v = clamp(kp_xy * 偏移, ±max_vel_xy)，再旋回 ENU
  void publishHorizontalTracking(double vz)
  {
    double ex_b = 0.0;
    double ey_b = 0.0;
    double lateral = 0.0;
    lateralErrorBody(ex_b, ey_b, lateral);
    const double yaw = fc_->yaw();
    const double vx_b = clamp(kp_xy_ * ex_b, max_vel_xy_);
    const double vy_b = clamp(kp_xy_ * ey_b, max_vel_xy_);
    drone_control::EnuPose vel;
    vel.x = std::cos(yaw) * vx_b - std::sin(yaw) * vy_b;
    vel.y = std::sin(yaw) * vx_b + std::cos(yaw) * vy_b;
    vel.z = vz;
    fc_->publishVelocitySetpoint(vel, 0.0);
  }

  void enter(const char * next)
  {
    RCLCPP_INFO(get_logger(), "[LANDING] %s -> %s", state_, next);
    state_ = next;
    state_since_ = SteadyClock::now();
    align_frames_ok_ = 0;
  }

  void publishState()
  {
    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);
  }

  // --- 状态机 -----------------------------------------------------------
  void tick()
  {
    if (!fc_) {return;}
    const double alt = fc_->localPose().z;

    // 安全兜底：任何时候贴近地面就立刻上锁降落
    if (alt < safety_alt_ && std::string(state_) != kStateIdle &&
      std::string(state_) != kStateDone)
    {
      RCLCPP_WARN(get_logger(), "[LANDING] 高度 %.3f m < safety_alt %.3f m，立即 LAND", alt,
        safety_alt_);
      enter(kStateTouchdown);
    }
    if (requested_stop_) {
      requested_stop_ = false;
      requested_start_ = false;
      RCLCPP_WARN(get_logger(), "[LANDING] 收到 stop 请求，立即 LAND");
      enter(kStateTouchdown);
    }
    if (requested_start_) {
      requested_start_ = false;
      beginLanding();
    }

    const std::string state(state_);
    if (state == kStateIdle || state == kStateDone) {
      publishStateThrottled();
      return;
    }

    if (state == kStateSearch) {
      stepSearch(alt);
    } else if (state == kStateAlign) {
      stepAlign(alt);
    } else if (state == kStateDescend) {
      stepDescend(alt);
    } else if (state == kStateFinal) {
      stepFinal(alt);
    } else if (state == kStateTouchdown) {
      stepTouchdown();
    }
    publishStateThrottled();
  }

  void beginLanding()
  {
    if (!fc_->state().connected) {
      RCLCPP_ERROR(get_logger(), "[LANDING] mavros 未连接，拒绝启动");
      return;
    }
    const drone_control::EnuPose p = fc_->localPose();
    search_hold_ = p;
    search_hold_.z = align_alt_;
    RCLCPP_INFO(get_logger(), "[LANDING] 启动：当前位置 (%.2f, %.2f, %.2f)，目标悬停高度 %.2f m",
      p.x, p.y, p.z, align_alt_);

    if (!fc_->armed()) {
      // 未解锁：本机 PX4 无法用 OFFBOARD 从地面起飞，先 AUTO.TAKEOFF
      drone_control::OffboardOptions options;
      options.rate_hz = control_rate_;
      RCLCPP_INFO(get_logger(), "[LANDING] 未解锁，先 AUTO.TAKEOFF 到 %.2f m", align_alt_);
      if (!fc_->takeoffTo(search_hold_, control_rate_, takeoff_tolerance_,
        std::chrono::seconds(static_cast<int>(takeoff_timeout_)), options))
      {
        RCLCPP_ERROR(get_logger(), "[LANDING] 起飞失败");
        enter(kStateDone);
        return;
      }
    } else if (fc_->mode() != "OFFBOARD") {
      // 已解锁但不在 OFFBOARD：先按 §5.3 顺序接管
      drone_control::OffboardOptions options;
      options.rate_hz = control_rate_;
      options.prestream = std::chrono::milliseconds(
        static_cast<int>(prestream_ * 1000.0));
      if (!fc_->engageOffboard(fc_->localPose(), options)) {
        RCLCPP_ERROR(get_logger(), "[LANDING] OFFBOARD 接管失败");
        enter(kStateDone);
        return;
      }
    } else {
      // 已在 OFFBOARD：预发 2 s 位置设定点，避免交接瞬间掉高
      fc_->streamPositionSetpoint(fc_->localPose(), 0.0,
        std::chrono::milliseconds(static_cast<int>(prestream_ * 1000.0)), control_rate_);
    }
    enter(kStateSearch);
  }

  void stepSearch(double alt)
  {
    if (markerFresh()) {
      enter(kStateAlign);
      return;
    }
    // 爬升到 align_alt 并在原地小幅偏航扫描（位置 + yaw_rate，掩码见 flight_control.hpp）
    drone_control::EnuPose hold = search_hold_;
    hold.z = align_alt_;
    fc_->publishPositionSetpointWithYawRate(hold, search_yaw_rate_);

    const double elapsed =
      std::chrono::duration<double>(SteadyClock::now() - state_since_).count();
    if (elapsed > search_timeout_) {
      RCLCPP_ERROR(get_logger(), "[LANDING] SEARCH 超时（%.1f s）仍未见码，降回地面", elapsed);
      enter(kStateTouchdown);
    }
    (void)alt;
  }

  void stepAlign(double alt)
  {
    if (!markerFresh()) {
      handleMarkerLost("ALIGN");
      return;
    }
    double ex_b = 0.0;
    double ey_b = 0.0;
    double lateral = 0.0;
    lateralErrorBody(ex_b, ey_b, lateral);

    // 高度保持用闭环（速度模式 vz=0 会掉高，实测 8 s 掉 1 m）
    const double vz = clamp(kp_z_ * (align_alt_ - alt), max_vz_);
    publishHorizontalTracking(vz);

    if (lateral < lateral_tol_) {
      ++align_frames_ok_;
      if (align_frames_ok_ >= align_frames_) {
        RCLCPP_INFO(get_logger(), "[LANDING] 对准完成：水平误差 %.3f m（连续 %d 帧 < %.3f）",
          lateral, align_frames_ok_, lateral_tol_);
        enter(kStateDescend);
      }
    } else if (lateral > align_abort_tol_) {
      // 误差过大（例如码在视野边缘、解算跳变）：退回 SEARCH 重新搜索
      RCLCPP_WARN(get_logger(), "[LANDING] 水平误差 %.3f m 超过 %.2f m，回到 SEARCH", lateral,
        align_abort_tol_);
      enter(kStateSearch);
    } else {
      align_frames_ok_ = 0;
    }
  }

  void stepDescend(double alt)
  {
    if (!markerFresh()) {
      handleMarkerLost("DESCEND");
      return;
    }
    publishHorizontalTracking(-descend_speed_);
    // 每 2 s 打一次实测下降速率：F4 第一次跑时 DESCEND 花了 108 s 才降到 0.5 m，
    // 光看阶段切换日志看不出"指令 -0.30 实际只有 -0.014"，必须打速率才能定位。
    {
      const auto now_t = SteadyClock::now();
      if (!descend_trace_valid_) {
        descend_trace_valid_ = true;
        descend_trace_alt_ = alt;
        descend_trace_time_ = now_t;
      } else if (std::chrono::duration<double>(now_t - descend_trace_time_).count() >= 2.0) {
        const double dt = std::chrono::duration<double>(now_t - descend_trace_time_).count();
        const double rate = (descend_trace_alt_ - alt) / dt;
        RCLCPP_INFO(get_logger(),
          "[LANDING] DESCEND alt=%.3f m  指令 vz=%.2f m/s  实测下降 %.3f m/s",
          alt, -descend_speed_, rate);
        descend_trace_alt_ = alt;
        descend_trace_time_ = now_t;
      }
    }
    if (alt < final_alt_) {
      RCLCPP_INFO(get_logger(), "[LANDING] 高度 %.3f m < %.2f m，进入 FINAL（vz=%.2f）", alt,
        final_alt_, final_speed_);
      enter(kStateFinal);
    }
  }

  void stepFinal(double alt)
  {
    if (!markerFresh()) {
      handleMarkerLost("FINAL");
      return;
    }
    double ex_b = 0.0;
    double ey_b = 0.0;
    double lateral = 0.0;
    lateralErrorBody(ex_b, ey_b, lateral);
    // 容差收紧到 final_tol：偏了就不再往下压，先把水平修正好
    const double vz = (lateral < final_tol_) ? -final_speed_ : 0.0;
    publishHorizontalTracking(vz);
    if (alt < touchdown_alt_) {
      RCLCPP_INFO(get_logger(), "[LANDING] 高度 %.3f m < %.2f m，触地", alt, touchdown_alt_);
      enter(kStateTouchdown);
    }
  }

  void stepTouchdown()
  {
    if (!land_in_progress_) {
      land_in_progress_ = true;
      RCLCPP_INFO(get_logger(), "[LANDING] 请求 AUTO.LAND 并等待上锁");
      const bool ok = fc_->land(fc_->localPose(), control_rate_,
        std::chrono::seconds(static_cast<int>(land_timeout_)));
      land_in_progress_ = false;
      if (ok) {
        RCLCPP_INFO(get_logger(), "[LANDING] 已上锁，降落完成");
      } else {
        RCLCPP_ERROR(get_logger(), "[LANDING] AUTO.LAND 超时未上锁");
      }
      enter(kStateDone);
    }
  }

  void handleMarkerLost(const char * from)
  {
    RCLCPP_WARN(get_logger(), "[LANDING] %s 阶段码丢失，悬停 %.1f s 后回 SEARCH", from,
      relost_hover_);
    const drone_control::EnuPose p = fc_->localPose();
    const auto hold_until = SteadyClock::now() +
      std::chrono::milliseconds(static_cast<int>(relost_hover_ * 1000.0));
    while (rclcpp::ok() && SteadyClock::now() < hold_until) {
      drone_control::EnuPose vel;   // 全零速度 = 原地悬停
      fc_->publishVelocitySetpoint(vel, 0.0);
      fc_->spinOnce();
      rclcpp::sleep_for(std::chrono::milliseconds(20));
      if (markerFresh()) {return;}  // 恢复可见就直接留在原阶段
    }
    search_hold_ = p;
    search_hold_.z = align_alt_;
    enter(kStateSearch);
  }

  void publishStateThrottled()
  {
    const auto now = SteadyClock::now();
    if (std::chrono::duration<double>(now - last_state_pub_).count() >= 0.1) {
      last_state_pub_ = now;
      publishState();
    }
  }

  // --- 参数 -------------------------------------------------------------
  double align_alt_{2.5};
  double descend_speed_{0.30};
  double final_speed_{0.12};
  double lateral_tol_{0.15};
  bool descend_trace_valid_{false};
  double descend_trace_alt_{0.0};
  SteadyClock::time_point descend_trace_time_{};
  double final_tol_{0.08};
  double kp_xy_{0.8};
  double max_vel_xy_{0.6};
  double lost_timeout_{0.5};
  double search_yaw_rate_{0.3};
  double align_abort_tol_{2.50};
  double control_rate_{50.0};
  double kp_z_{0.8};
  double max_vz_{0.30};
  int align_frames_{10};
  double final_alt_{0.5};
  double touchdown_alt_{0.15};
  double safety_alt_{0.12};
  double relost_hover_{1.0};
  double search_timeout_{60.0};
  double land_timeout_{40.0};
  double prestream_{2.0};
  double takeoff_tolerance_{0.15};
  double takeoff_timeout_{30.0};

  // --- 运行时状态 -------------------------------------------------------
  std::unique_ptr<drone_control::FlightControl> fc_;
  const char * state_{kStateIdle};
  SteadyClock::time_point state_since_{SteadyClock::now()};
  SteadyClock::time_point last_state_pub_{SteadyClock::now()};
  drone_control::EnuPose search_hold_{};
  bool requested_start_{false};
  bool requested_stop_{false};
  bool land_in_progress_{false};
  int align_frames_ok_{0};

  geometry_msgs::msg::PoseStamped marker_;
  bool has_marker_{false};
  SteadyClock::time_point last_detection_{};
  mutable std::mutex mutex_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

}  // namespace drone_aruco

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_aruco::LandingController>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("landing_controller"), "异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
