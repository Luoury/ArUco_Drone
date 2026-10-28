// ArUco 检测节点：/camera/image_raw (+ /camera/camera_info) -> /aruco/detection + /aruco/debug_image
//
// 坐标约定（全部由 M0/M1 实测确定，见 需求书 §6.4 与 config/aruco.yaml 注释）：
//   1) solvePnP 在"相机光学系"下给结果：x 右、y 下、z 前（沿光轴）。
//   2) 本机相机装在 base_link 正下方，CameraJoint 的 <pose> 是 rpy=(0, +90°, 0)，
//      即光学系的 z 轴指向机体 -z（向下）、x 轴指向机体 -y（右侧）、y 轴指向机体 +x（前方）。
//      于是 光学->机体系(FLU) 的旋转是固定阵 kOptToFlu（见下）。
//   3) 机体 FLU 再用 /mavros/local_position/pose 的 yaw 旋转到 map(ENU) 并叠加无人机位置。
//   4) 所以 /aruco/detection 的 frame_id 是 map，position 就是码在世界里的绝对坐标。
//
// 所有数值参数都在 drone_bringup/config/aruco.yaml，代码里不写死常数。

#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace drone_aruco
{

using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

// 光学系 -> 机体系(FLU)：x_opt(右) -> -y_flu，y_opt(下) -> +x_flu，z_opt(前) -> -z_flu
// 光学系(OpenCV: x 右, y 下, z 沿光轴) → 机体 FLU(x 前, y 左, z 上)
//   相机朝机体 -z 看，画面"上"对应机体 +x，画面"右"对应机体 -y：
//   x_body = -y_cam, y_body = -x_cam, z_body = -z_cam
// 注意第一行必须是 -1：M1 阶段悬停在码正上方时 x/y 偏移都为 0，这个符号错了也看不出来；
// M3 飞离码上空后实测"估计偏移 (+1.08, -0.87) vs 真实 (-1.09, -0.91)"才发现 x 反了。
const cv::Matx33d kOptToFlu{0.0, -1.0, 0.0,
                            -1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0};

class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector()
  : Node("aruco_detector")
  {
    // ---- 参数（唯一来源：drone_bringup/config/aruco.yaml）----
    dictionary_name_ = declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    marker_id_ = declare_parameter<int>("marker_id", 0);
    marker_size_ = declare_parameter<double>("marker_size", 0.5);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    detection_frame_ = declare_parameter<std::string>("detection_frame", "map");
    debug_axes_length_ = declare_parameter<double>("debug_axes_length", 0.15);
    // 需求书 §5.2：位姿做 EMA 平滑（α≈0.3），否则降落时 x/y 抖
    smooth_alpha_ = declare_parameter<double>("smooth_alpha", 0.3);
    smooth_reset_timeout_ = declare_parameter<double>("smooth_reset_timeout", 1.0);
    camera_pitch_deg_ = declare_parameter<double>("camera_pitch_deg", 90.0);
    camera_yaw_deg_ = declare_parameter<double>("camera_yaw_deg", 0.0);
    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.0);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.0);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.0);
    fx_fallback_ = declare_parameter<double>("camera_fx", 0.0);
    fy_fallback_ = declare_parameter<double>("camera_fy", 0.0);
    cx_fallback_ = declare_parameter<double>("camera_cx", 0.0);
    cy_fallback_ = declare_parameter<double>("camera_cy", 0.0);
    const std::string image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const std::string info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    const std::string pose_topic = declare_parameter<std::string>("drone_pose_topic",
                                                                  "/mavros/local_position/pose");
    const std::string detection_topic = declare_parameter<std::string>("detection_topic", "/aruco/detection");
    const std::string debug_topic = declare_parameter<std::string>("debug_topic", "/aruco/debug_image");

    dictionary_ = makeDictionary(dictionary_name_);

    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    const int win_min = declare_parameter<int>("detector_win_min", 5);
    const int win_max = declare_parameter<int>("detector_win_max", 23);
    const int win_step = declare_parameter<int>("detector_win_step", 10);
    const double min_perimeter = declare_parameter<double>("detector_min_perimeter_rate", 0.03);
    detector_params_->adaptiveThreshWinSizeMin = win_min;
    detector_params_->adaptiveThreshWinSizeMax = win_max;
    detector_params_->adaptiveThreshWinSizeStep = win_step;
    detector_params_->minMarkerPerimeterRate = min_perimeter;

    // 相机与 mavros 都是 BEST_EFFORT（实测：reliable 订户收不到 gz/mavros 的数据）
    rclcpp::QoS sensor_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensor_qos.best_effort();

    pose_sub_ = create_subscription<PoseStamped>(
      pose_topic, sensor_qos,
      [this](const PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        drone_pose_ = *msg;
        has_pose_ = true;
      });

    info_sub_ = create_subscription<CameraInfo>(
      info_topic, sensor_qos,
      [this](const CameraInfo::SharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
          camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            msg->k[0], 0.0, msg->k[2],
            0.0, msg->k[4], msg->k[5],
            0.0, 0.0, 1.0);
          have_intrinsics_ = true;
        }
      });

    image_sub_ = create_subscription<Image>(
      image_topic, sensor_qos,
      [this](const Image::SharedPtr msg) {onImage(msg);});

    detection_pub_ = create_publisher<PoseStamped>(detection_topic, 10);
    if (publish_debug_) {
      debug_pub_ = create_publisher<Image>(debug_topic, rclcpp::QoS(2).best_effort());
    }

    RCLCPP_INFO(get_logger(), "aruco_detector 启动：字典=%s marker_id=%d marker_size=%.3f m 检测系=%s",
                dictionary_name_.c_str(), marker_id_, marker_size_, detection_frame_.c_str());
    RCLCPP_INFO(get_logger(), "订阅 %s + %s + %s；发布 %s%s",
                image_topic.c_str(), info_topic.c_str(), pose_topic.c_str(), detection_topic.c_str(),
                publish_debug_ ? (", " + debug_topic).c_str() : "");
    if (fx_fallback_ > 0.0) {
      RCLCPP_INFO(get_logger(), "camera_info 未到时使用 yaml 内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                  fx_fallback_, fy_fallback_, cx_fallback_, cy_fallback_);
    }
  }

private:
  static cv::Ptr<cv::aruco::Dictionary> makeDictionary(const std::string & name)
  {
    if (name == "DICT_4X4_50") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);}
    if (name == "DICT_4X4_100") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);}
    if (name == "DICT_4X4_250") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);}
    if (name == "DICT_4X4_1000") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);}
    if (name == "DICT_5X5_50") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);}
    if (name == "DICT_5X5_250") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);}
    if (name == "DICT_6X6_50") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);}
    if (name == "DICT_6X6_250") {return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);}
    if (name == "DICT_ARUCO_ORIGINAL") {
      return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    }
    throw std::runtime_error("不支持的字典名: " + name);
  }

  void onImage(const Image::SharedPtr msg)
  {
    cv::Mat gray;
    try {
      if (msg->encoding == "rgb8") {
        cv::Mat rgb = cv_bridge::toCvShare(msg, "rgb8")->image;
        cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
      } else if (msg->encoding == "bgr8") {
        cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
      } else if (msg->encoding == "mono8") {
        gray = cv_bridge::toCvShare(msg, "mono8")->image;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "未处理的编码: %s",
                            msg->encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge 失败: %s", e.what());
      return;
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);
    ++frame_count_;

    // 调试图总是画：即使没检到也发出去（便于排查）
    cv::Mat debug_bgr;
    if (publish_debug_) {
      cv::cvtColor(gray, debug_bgr, cv::COLOR_GRAY2BGR);
      if (!ids.empty()) {cv::aruco::drawDetectedMarkers(debug_bgr, corners, ids);}
    }

    int hit_index = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (marker_id_ < 0 || ids[i] == marker_id_) {
        hit_index = static_cast<int>(i);
        break;
      }
    }

    if (hit_index >= 0 && (have_intrinsics_ || fx_fallback_ > 0.0)) {
      const cv::Mat cam = have_intrinsics_ ? camera_matrix_ : fallbackMatrix();
      const cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
      const auto half = static_cast<float>(marker_size_ * 0.5);
      std::vector<cv::Point3f> object_points = {
        {-half, half, 0.0f}, {half, half, 0.0f}, {half, -half, 0.0f}, {-half, -half, 0.0f}};

      cv::Vec3d rvec;
      cv::Vec3d tvec;
      if (cv::solvePnP(object_points, corners[hit_index], cam, dist, rvec, tvec, false,
                       cv::SOLVEPNP_IPPE_SQUARE))
      {
        publishDetection(msg->header.stamp, rvec, tvec);
        if (publish_debug_) {
          cv::aruco::drawAxis(debug_bgr, cam, dist, rvec, tvec, debug_axes_length_);
        }
        cv::putText(debug_bgr,
                    cv::format("id=%d  z_cam=%.2fm  frame=%zu", marker_id_, tvec[2], frame_count_),
                    cv::Point(12, 34), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
      }
    } else if (publish_debug_ && hit_index < 0) {
      cv::putText(debug_bgr, cv::format("no marker (frame=%zu)", frame_count_),
                  cv::Point(12, 34), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
    }

    if (publish_debug_ && debug_pub_ && !debug_bgr.empty()) {
      cv_bridge::CvImage out(msg->header, "bgr8", debug_bgr);
      debug_pub_->publish(*out.toImageMsg());
    }
  }

  cv::Mat fallbackMatrix() const
  {
    return (cv::Mat_<double>(3, 3) <<
           fx_fallback_, 0.0, cx_fallback_,
           0.0, fy_fallback_, cy_fallback_,
           0.0, 0.0, 1.0);
  }

  void publishDetection(const builtin_interfaces::msg::Time & stamp, const cv::Vec3d & rvec,
                        const cv::Vec3d & tvec)
  {
    // 光学系 -> 机体系：默认按 CameraJoint 的 rpy=(0, 90°, 0)（正下方相机）
    cv::Matx33d r_opt_to_body = kOptToFlu;
    const double yaw_corr = camera_yaw_deg_ * M_PI / 180.0;
    const double pitch_corr = (camera_pitch_deg_ - 90.0) * M_PI / 180.0;
    if (std::abs(yaw_corr) > 1e-12 || std::abs(pitch_corr) > 1e-12) {
      const cv::Matx33d r_z{std::cos(yaw_corr), -std::sin(yaw_corr), 0.0,
        std::sin(yaw_corr), std::cos(yaw_corr), 0.0, 0.0, 0.0, 1.0};
      const cv::Matx33d r_y{std::cos(pitch_corr), 0.0, std::sin(pitch_corr), 0.0, 1.0, 0.0,
        -std::sin(pitch_corr), 0.0, std::cos(pitch_corr)};
      r_opt_to_body = r_z * (r_y * kOptToFlu);
    }

    cv::Mat r_marker;
    cv::Rodrigues(cv::Mat(rvec), r_marker);
    const cv::Matx33d & r = r_opt_to_body;

    // 码中心在光学系下的位置
    const cv::Vec3d p_opt(tvec[0], tvec[1], tvec[2]);
    cv::Vec3d p_body(
      r(0, 0) * p_opt[0] + r(0, 1) * p_opt[1] + r(0, 2) * p_opt[2],
      r(1, 0) * p_opt[0] + r(1, 1) * p_opt[1] + r(1, 2) * p_opt[2],
      r(2, 0) * p_opt[0] + r(2, 1) * p_opt[1] + r(2, 2) * p_opt[2]);
    p_body += cv::Vec3d(camera_offset_x_, camera_offset_y_, camera_offset_z_);

    cv::Vec3d drone_pos(0.0, 0.0, 0.0);
    // 机体 -> ENU 的完整旋转（含 roll/pitch）。悬停时机体有 1~3° 倾斜，
    // 只按 yaw 旋转会把倾斜耦合进 p_body，实测造成 0.05~0.10 m 的定位偏差。
    //
    // 注意：不要用"按时间戳挑最近位姿"来消除滞后——实测 mavros 的
    // /mavros/local_position/pose 头里是墙钟时间（~1.79e9 s），而 gz 图像是仿真时间
    // （几百秒），两个时基差 1.79e9 s，按戳配对会挑到缓冲里最旧的那帧，
    // 反而把爬升阶段的位姿当成当前位姿（实测误差从 0.03 m 恶化到 0.13 m）。
    cv::Matx33d r_map_body{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    PoseStamped drone_pose;
    bool have_pose = false;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      drone_pose = drone_pose_;
      have_pose = has_pose_;
    }
    if (have_pose) {
      tf2::Quaternion q(drone_pose.pose.orientation.x, drone_pose.pose.orientation.y,
                        drone_pose.pose.orientation.z, drone_pose.pose.orientation.w);
      q.normalize();
      tf2::Matrix3x3 m(q);
      r_map_body = cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]);
      drone_pos = cv::Vec3d(drone_pose.pose.position.x, drone_pose.pose.position.y,
                            drone_pose.pose.position.z);
    }

    const cv::Vec3d p_map = drone_pos + cv::Vec3d(
      r_map_body(0, 0) * p_body[0] + r_map_body(0, 1) * p_body[1] + r_map_body(0, 2) * p_body[2],
      r_map_body(1, 0) * p_body[0] + r_map_body(1, 1) * p_body[1] + r_map_body(1, 2) * p_body[2],
      r_map_body(2, 0) * p_body[0] + r_map_body(2, 1) * p_body[1] + r_map_body(2, 2) * p_body[2]);

    // 姿态：R_map_marker = Rz(yaw) * R_opt_to_body * R_opt_marker
    const cv::Matx33d r_body_marker(r(0, 0) * r_marker.at<double>(0, 0) + r(0, 1) *
      r_marker.at<double>(1, 0) + r(0, 2) * r_marker.at<double>(2, 0),
      r(0, 0) * r_marker.at<double>(0, 1) + r(0, 1) * r_marker.at<double>(1, 1) + r(0, 2) *
      r_marker.at<double>(2, 1),
      r(0, 0) * r_marker.at<double>(0, 2) + r(0, 1) * r_marker.at<double>(1, 2) + r(0, 2) *
      r_marker.at<double>(2, 2),
      r(1, 0) * r_marker.at<double>(0, 0) + r(1, 1) * r_marker.at<double>(1, 0) + r(1, 2) *
      r_marker.at<double>(2, 0),
      r(1, 0) * r_marker.at<double>(0, 1) + r(1, 1) * r_marker.at<double>(1, 1) + r(1, 2) *
      r_marker.at<double>(2, 1),
      r(1, 0) * r_marker.at<double>(0, 2) + r(1, 1) * r_marker.at<double>(1, 2) + r(1, 2) *
      r_marker.at<double>(2, 2),
      r(2, 0) * r_marker.at<double>(0, 0) + r(2, 1) * r_marker.at<double>(1, 0) + r(2, 2) *
      r_marker.at<double>(2, 0),
      r(2, 0) * r_marker.at<double>(0, 1) + r(2, 1) * r_marker.at<double>(1, 1) + r(2, 2) *
      r_marker.at<double>(2, 1),
      r(2, 0) * r_marker.at<double>(0, 2) + r(2, 1) * r_marker.at<double>(1, 2) + r(2, 2) *
      r_marker.at<double>(2, 2));

    const cv::Matx33d r_map_marker = r_map_body * r_body_marker;
    tf2::Matrix3x3 m(r_map_marker(0, 0), r_map_marker(0, 1), r_map_marker(0, 2),
                     r_map_marker(1, 0), r_map_marker(1, 1), r_map_marker(1, 2),
                     r_map_marker(2, 0), r_map_marker(2, 1), r_map_marker(2, 2));
    tf2::Quaternion q_map;
    m.getRotation(q_map);

    // EMA 平滑（需求书 §5.2，α≈0.3）：逐帧 PnP 结果的噪声约 2 cm，不滤直接喂给
    // 降落控制器会让下降段的水平速度抖。超过 smooth_reset_timeout 没有检测到就
    // 丢弃历史值，避免重新捕获时被上一次的旧位置拖住。
    cv::Vec3d smooth_p = p_map;
    tf2::Quaternion smooth_q = q_map;
    const double stamp_sec = rclcpp::Time(stamp).seconds();
    if (has_ema_ && (stamp_sec - last_ema_stamp_) <= smooth_reset_timeout_) {
      smooth_p = cv::Vec3d(
        smooth_alpha_ * p_map[0] + (1.0 - smooth_alpha_) * ema_p_[0],
        smooth_alpha_ * p_map[1] + (1.0 - smooth_alpha_) * ema_p_[1],
        smooth_alpha_ * p_map[2] + (1.0 - smooth_alpha_) * ema_p_[2]);
      smooth_q = ema_q_.slerp(q_map, smooth_alpha_);
      smooth_q.normalize();
    } else {
      RCLCPP_INFO(get_logger(), "EMA 复位（距上次检测 %.3f s）", has_ema_ ?
        (stamp_sec - last_ema_stamp_) : -1.0);
    }
    ema_p_ = smooth_p;
    ema_q_ = smooth_q;
    has_ema_ = true;
    last_ema_stamp_ = stamp_sec;

    PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = detection_frame_;
    out.pose.position.x = smooth_p[0];
    out.pose.position.y = smooth_p[1];
    out.pose.position.z = smooth_p[2];
    out.pose.orientation.x = smooth_q.x();
    out.pose.orientation.y = smooth_q.y();
    out.pose.orientation.z = smooth_q.z();
    out.pose.orientation.w = smooth_q.w();
    detection_pub_->publish(out);
    const double lateral = std::hypot(p_body[0], p_body[1]);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "检测到 id=%d：机体系偏移 (%.3f, %.3f, %.3f) m，水平偏 %.3f m，光学距 %.3f m，map=(%.3f, %.3f, %.3f)",
                         marker_id_, p_body[0], p_body[1], p_body[2], lateral, tvec[2],
                         p_map[0], p_map[1], p_map[2]);
  }

  std::string dictionary_name_;
  int marker_id_{0};
  double marker_size_{0.0};
  bool publish_debug_{true};
  std::string detection_frame_{"map"};
  double debug_axes_length_{0.0};
  double camera_pitch_deg_{90.0};
  double camera_yaw_deg_{0.0};
  double camera_offset_x_{0.0};
  double camera_offset_y_{0.0};
  double camera_offset_z_{0.0};
  double fx_fallback_{0.0};
  double fy_fallback_{0.0};
  double cx_fallback_{0.0};
  double cy_fallback_{0.0};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat camera_matrix_;
  bool have_intrinsics_{false};

  PoseStamped drone_pose_;
  bool has_pose_{false};
  std::mutex pose_mutex_;
  size_t frame_count_{0};

  // EMA 平滑状态（smooth_alpha / smooth_reset_timeout 两个参数控制）
  double smooth_alpha_{0.3};
  double smooth_reset_timeout_{1.0};
  bool has_ema_{false};
  double last_ema_stamp_{0.0};
  cv::Vec3d ema_p_{0.0, 0.0, 0.0};
  tf2::Quaternion ema_q_;

  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr detection_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
};

}  // namespace drone_aruco

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_aruco::ArucoDetector>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("aruco_detector"), "异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
