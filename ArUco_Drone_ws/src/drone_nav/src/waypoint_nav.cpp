// 航点导航（需求书 §5.4 / F3）
//
// 职责：按 waypoints.yaml 里的航点顺序飞过去，每个点稳住 steady_hold 秒再走下一个。
// 与 landing_controller 一样，本节点不自作主张切模式：由 mission_manager 用
// /waypoint/start、/waypoint/stop 调度（需求书 §4 职责分离铁律）。
//
// 对外信号（验收脚本只依赖这两个，避免再新增自定义 msg）：
//   /waypoint/state    std_msgs/String          IDLE / TAKEOFF / GOTO / HOLD / DONE
//   /waypoint/progress std_msgs/Float64MultiArray
//       [0] 当前航点(1 起)  [1] 总数  [2] 当前误差 m  [3] 已稳定秒数
//       [4] 最近完成的航点(1 起, 0=无)  [5] 该航点完成时的稳态误差 m
//   —— 后两项是给验收脚本用的：dwell 在"完成"那一拍会被清零，只靠 [3] 抓不到
//      稳态误差（F3 第一次跑就是这么 FAIL 的）。
//
// 实测教训（别改回去）：
//   * 设定点一律 ENU，ENU→NED 由 mavros 做；
//   * 位置模式无法从地面起飞 → 未解锁时先 AUTO.TAKEOFF（§6.4 坑 4）；
//   * 交接 OFFBOARD 前必须先预发 2 s 位置设定点，否则切模式瞬间会掉高。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "drone_control/flight_control.hpp"

namespace drone_nav
{

using SteadyClock = std::chrono::steady_clock;

constexpr const char * kStateIdle = "IDLE";
constexpr const char * kStateTakeoff = "TAKEOFF";
constexpr const char * kStateGoto = "GOTO";
constexpr const char * kStateHold = "HOLD";
constexpr const char * kStateDone = "DONE";

/// 三维欧氏距离（ENU）
double distance3d(const drone_control::EnuPose & a, const drone_control::EnuPose & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

class WaypointNav : public rclcpp::Node
{
public:
  WaypointNav()
  : rclcpp::Node("waypoint_nav")
  {
    // ---- 航点表（唯一来源：drone_bringup/config/waypoints.yaml）-------------
    waypoint_dim_ = declare_parameter<int>("waypoint_dim", 3);
    waypoints_flat_ = declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    settle_tol_ = declare_parameter<double>("settle_tol", 0.30);
    steady_hold_ = declare_parameter<double>("steady_hold", 2.0);

    // ---- 实现细节参数（同样只在 yaml 里出现）------------------------------
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    goto_timeout_ = declare_parameter<double>("goto_timeout", 45.0);
    prestream_ = declare_parameter<double>("prestream", 2.0);
    takeoff_tolerance_ = declare_parameter<double>("takeoff_tolerance", 0.15);
    takeoff_timeout_ = declare_parameter<double>("takeoff_timeout", 30.0);
    finish_action_ = declare_parameter<std::string>("finish_action", "HOVER");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/waypoint/state");
    const std::string progress_topic =
      declare_parameter<std::string>("progress_topic", "/waypoint/progress");

    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 10);
    progress_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(progress_topic, 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/waypoint/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (state_ != kStateIdle && state_ != kStateDone) {
          response->success = false;
          response->message = std::string("航点任务已在运行，当前阶段 ") + state_;
          return;
        }
        if (!parseWaypoints(response->message)) {
          response->success = false;
          return;
        }
        requested_start_ = true;
        response->success = true;
        response->message = "已受理，" + std::to_string(waypoints_.size()) + " 个航点";
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/waypoint/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        requested_stop_ = true;
        response->success = true;
        response->message = "已受理，就地悬停";
      });

    RCLCPP_INFO(get_logger(), "航点导航就绪：settle_tol=%.2f steady_hold=%.1f s finish=%s",
      settle_tol_, steady_hold_, finish_action_.c_str());
  }

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
  /// 把扁平参数数组切成 EnuPose 列表；失败时写清原因
  bool parseWaypoints(std::string & error)
  {
    if (waypoint_dim_ <= 0 || static_cast<int>(waypoints_flat_.size()) % waypoint_dim_ != 0) {
      error = "waypoints 参数长度 " + std::to_string(waypoints_flat_.size()) +
        " 不是 waypoint_dim=" + std::to_string(waypoint_dim_) + " 的整数倍";
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    waypoints_.clear();
    for (std::size_t i = 0; i + static_cast<std::size_t>(waypoint_dim_) <= waypoints_flat_.size();
      i += static_cast<std::size_t>(waypoint_dim_))
    {
      drone_control::EnuPose p;
      p.x = waypoints_flat_[i];
      p.y = waypoints_flat_[i + 1];
      p.z = (waypoint_dim_ >= 3) ? waypoints_flat_[i + 2] : 1.5;
      waypoints_.push_back(p);
    }
    if (waypoints_.empty()) {
      error = "waypoints 参数为空";
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    return true;
  }

  void enter(const char * next)
  {
    RCLCPP_INFO(get_logger(), "[WAYPOINT] %s -> %s", state_, next);
    state_ = next;
    state_since_ = SteadyClock::now();
  }

  void tick()
  {
    if (!fc_) {return;}

    publishStateThrottled();

    if (requested_stop_) {
      requested_stop_ = false;
      requested_start_ = false;
      waypoint_active_ = false;
      if (fc_->armed()) {
        hold_ = fc_->localPose();
        fc_->publishPositionSetpoint(hold_, 0.0);
      }
      if (std::string(state_) != kStateIdle) {
        enter(kStateIdle);
      }
      return;
    }

    if (requested_start_) {
      requested_start_ = false;
      beginMission();
    }

    const std::string state(state_);
    if (state == kStateTakeoff) {
      // takeoffTo 是阻塞式流水，返回即已进 OFFBOARD
      stepTakeoff();
    } else if (state == kStateGoto || state == kStateHold) {
      stepGoto();
    } else if (state == kStateDone) {
      // DONE 之后按 finish_action 保持位置（LAND 交给 mission_manager 调度）
      if (fc_->armed()) {
        fc_->publishPositionSetpoint(hold_, 0.0);
      }
    }
  }

  void beginMission()
  {
    if (!fc_->state().connected) {
      RCLCPP_ERROR(get_logger(), "[WAYPOINT] mavros 未连接，拒绝启动");
      return;
    }
    index_ = 0;
    waypoint_active_ = true;
    hold_ = waypoints_.front();
    enter(kStateTakeoff);
  }

  void stepTakeoff()
  {
    drone_control::OffboardOptions options;
    options.rate_hz = control_rate_;
    options.prestream = std::chrono::milliseconds(static_cast<int>(prestream_ * 1000.0));

    if (!fc_->armed()) {
      RCLCPP_INFO(get_logger(), "[WAYPOINT] 未解锁，AUTO.TAKEOFF 到第一个航点 (%.2f, %.2f, %.2f)",
        hold_.x, hold_.y, hold_.z);
      if (!fc_->takeoffTo(hold_, control_rate_, takeoff_tolerance_,
        std::chrono::seconds(static_cast<int>(takeoff_timeout_)), options))
      {
        RCLCPP_ERROR(get_logger(), "[WAYPOINT] 起飞失败");
        waypoint_active_ = false;
        enter(kStateIdle);
        return;
      }
    } else if (fc_->mode() != "OFFBOARD") {
      if (!fc_->engageOffboard(fc_->localPose(), options)) {
        RCLCPP_ERROR(get_logger(), "[WAYPOINT] OFFBOARD 接管失败");
        waypoint_active_ = false;
        enter(kStateIdle);
        return;
      }
    }
    index_ = 0;
    dwell_ = 0.0;
    waypoint_since_ = SteadyClock::now();
    enter(kStateGoto);
  }

  void stepGoto()
  {
    if (!waypoint_active_) {return;}
    const drone_control::EnuPose target = waypoints_[index_];
    fc_->publishPositionSetpoint(target, 0.0);

    const drone_control::EnuPose p = fc_->localPose();
    const double err = distance3d(p, target);

    const double dt = 1.0 / control_rate_;
    if (err <= settle_tol_) {
      dwell_ += dt;
      if (std::string(state_) != kStateHold) {
        RCLCPP_INFO(get_logger(), "[WAYPOINT] 航点 %zu/%zu 到位（误差 %.3f m），开始稳定计时",
          index_ + 1, waypoints_.size(), err);
        enter(kStateHold);
      }
      if (dwell_ >= steady_hold_) {
        RCLCPP_INFO(get_logger(), "[WAYPOINT] 航点 %zu/%zu 稳态误差 %.3f m（稳定 %.1f s）",
          index_ + 1, waypoints_.size(), err, dwell_);
        settled_index_ = static_cast<int>(index_) + 1;
        settled_error_ = err;
        ++index_;
        dwell_ = 0.0;
        if (index_ >= waypoints_.size()) {
          hold_ = waypoints_.back();
          waypoint_active_ = false;
          enter(kStateDone);
          RCLCPP_INFO(get_logger(), "[WAYPOINT] 全部 %zu 个航点完成", waypoints_.size());
          return;
        }
        waypoint_since_ = SteadyClock::now();
        enter(kStateGoto);
      }
    } else {
      dwell_ = 0.0;
      if (std::string(state_) == kStateHold) {
        enter(kStateGoto);
      }
      const double spent =
        std::chrono::duration<double>(SteadyClock::now() - waypoint_since_).count();
      if (spent > goto_timeout_) {
        RCLCPP_WARN(get_logger(),
          "[WAYPOINT] 航点 %zu/%zu 超时未收敛（%.1f s，当前误差 %.3f m），继续下一个",
          index_ + 1, waypoints_.size(), spent, err);
        settled_index_ = static_cast<int>(index_) + 1;
        settled_error_ = err;
        ++index_;
        if (index_ >= waypoints_.size()) {
          hold_ = waypoints_.back();
          waypoint_active_ = false;
          enter(kStateDone);
          return;
        }
        waypoint_since_ = SteadyClock::now();
      }
    }
  }

  void publishStateThrottled()
  {
    const auto now = SteadyClock::now();
    if (std::chrono::duration<double>(now - last_pub_).count() < 0.1) {
      return;
    }
    last_pub_ = now;

    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);

    std_msgs::msg::Float64MultiArray progress;
    progress.data.resize(6);
    progress.data[0] = static_cast<double>(waypoints_.empty() ? 0 : index_ + 1);
    progress.data[1] = static_cast<double>(waypoints_.size());
    if (waypoint_active_ && index_ < waypoints_.size()) {
      progress.data[2] = distance3d(fc_->localPose(), waypoints_[index_]);
    } else {
      progress.data[2] = -1.0;
    }
    progress.data[3] = dwell_;
    progress.data[4] = static_cast<double>(settled_index_);
    progress.data[5] = settled_error_;
    progress_pub_->publish(progress);
  }

  // --- 参数 -------------------------------------------------------------
  int waypoint_dim_{3};
  std::vector<double> waypoints_flat_;
  std::vector<drone_control::EnuPose> waypoints_;
  double settle_tol_{0.30};
  double steady_hold_{2.0};
  double control_rate_{50.0};
  double goto_timeout_{45.0};
  double prestream_{2.0};
  double takeoff_tolerance_{0.15};
  double takeoff_timeout_{30.0};
  std::string finish_action_{"HOVER"};

  // --- 运行时状态 -------------------------------------------------------
  std::unique_ptr<drone_control::FlightControl> fc_;
  const char * state_{kStateIdle};
  SteadyClock::time_point state_since_{SteadyClock::now()};
  SteadyClock::time_point last_pub_{SteadyClock::now()};
  SteadyClock::time_point waypoint_since_{SteadyClock::now()};
  std::size_t index_{0};
  double dwell_{0.0};
  int settled_index_{0};        // 最近完成的航点（1 起）
  double settled_error_{-1.0};  // 该航点完成时的稳态误差
  bool waypoint_active_{false};
  bool requested_start_{false};
  bool requested_stop_{false};
  drone_control::EnuPose hold_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr progress_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

}  // namespace drone_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_nav::WaypointNav>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_nav"), "异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
