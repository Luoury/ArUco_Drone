// Gazebo Harmonic (gz-transport13) -> ROS 2 桥接节点。
//
// 为什么自己写而不用 ros_gz_bridge：
//   本机 apt 装的 ros-humble-ros-gz-bridge 0.244.20 链接的是
//   libignition-transport11（Fortress ABI），而 Gazebo 8.10 用的是
//   libgz-transport13（Harmonic ABI）。两者发现域不同，桥接进程会正常
//   创建 ROS 发布者，但 gz 侧订阅者根本注册不上：
//     $ gz topic -i -t /clock
//     No subscribers on topic [/clock]
//   结果 /clock、/camera/image_raw 永远没有数据。
//   实测 ldd 证据见 docs/troubleshooting.md。
//
// 这个节点只做三件事：/clock、/camera/image_raw、/camera/camera_info。
// 所有话题名与 QoS 都由 config/bridge.yaml 提供，代码里不写死。

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace drone_bringup
{

namespace
{

constexpr char kLogName[] = "gz_sensor_bridge";

/// 从 gz 的 Header 取 frame_id。
/// gz 的 Header.data 是 map<string, repeated string>，所以 value 是列表。
std::string gzFrameId(const gz::msgs::Header & header, const std::string & fallback)
{
  for (const auto & entry : header.data()) {
    if (entry.key() == "frame_id" && entry.value_size() > 0 && !entry.value(0).empty()) {
      return entry.value(0);
    }
  }
  return fallback;
}

/// gz 像素格式 -> ROS encoding；未知格式返回空串，由调用方跳过该帧。
std::string gzEncoding(int format)
{
  switch (format) {
    case gz::msgs::PixelFormatType::L_INT8:
      return "mono8";
    case gz::msgs::PixelFormatType::L_INT16:
      return "mono16";
    case gz::msgs::PixelFormatType::RGB_INT8:
      return "rgb8";
    case gz::msgs::PixelFormatType::RGBA_INT8:
      return "rgba8";
    case gz::msgs::PixelFormatType::BGRA_INT8:
      return "bgra8";
    case gz::msgs::PixelFormatType::BGR_INT8:
      return "bgr8";
    case gz::msgs::PixelFormatType::BAYER_RGGB8:
      return "bayer_rggb8";
    case gz::msgs::PixelFormatType::BAYER_BGGR8:
      return "bayer_bggr8";
    case gz::msgs::PixelFormatType::BAYER_GBRG8:
      return "bayer_gbrg8";
    case gz::msgs::PixelFormatType::BAYER_GRBG8:
      return "bayer_grbg8";
    default:
      return {};
  }
}

}  // namespace

class GzSensorBridge : public rclcpp::Node
{
public:
  GzSensorBridge()
  : rclcpp::Node(kLogName)
  {
    enable_clock_ = declare_parameter<bool>("enable_clock", true);
    enable_camera_ = declare_parameter<bool>("enable_camera", true);

    const auto gz_clock_topic = declare_parameter<std::string>("gz_clock_topic", "/clock");
    const auto gz_image_topic = declare_parameter<std::string>("gz_image_topic", "");
    const auto gz_info_topic = declare_parameter<std::string>("gz_info_topic", "");
    const auto ros_clock_topic = declare_parameter<std::string>("ros_clock_topic", "/clock");
    const auto ros_image_topic = declare_parameter<std::string>("ros_image_topic", "/camera/image_raw");
    const auto ros_info_topic = declare_parameter<std::string>(
      "ros_info_topic", "/camera/camera_info");
    const auto frame_id = declare_parameter<std::string>("frame_id", "camera_link");
    const auto qos_reliability = declare_parameter<std::string>("qos_reliability", "best_effort");
    const auto qos_depth = declare_parameter<int>("qos_depth", 5);
    const auto rate_log_period = declare_parameter<double>("rate_log_period", 5.0);

    rclcpp::QoS qos(static_cast<size_t>(qos_depth));
    if (qos_reliability == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    if (enable_clock_) {
      // /clock 在 ROS 2 里按惯例用 reliable（use_sim_time 的订户也多按 reliable 请求），
      // 因此时钟通道固定 reliable，不受图像通道 best_effort 设置影响。
      rclcpp::QoS clock_qos(static_cast<size_t>(qos_depth));
      clock_qos.reliable();
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(ros_clock_topic, clock_qos);
      if (!gz_node_.Subscribe(gz_clock_topic, &GzSensorBridge::onClock, this)) {
        RCLCPP_ERROR(get_logger(), "订阅 gz 时钟失败: %s", gz_clock_topic.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "桥接 gz %s -> ROS %s", gz_clock_topic.c_str(), ros_clock_topic.c_str());
      }
    }

    if (enable_camera_) {
      if (gz_image_topic.empty() || gz_info_topic.empty()) {
        RCLCPP_ERROR(get_logger(), "enable_camera 为真但 gz_image_topic/gz_info_topic 为空");
        enable_camera_ = false;
      } else {
        frame_id_ = frame_id;
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(ros_image_topic, qos);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(ros_info_topic, qos);
        if (!gz_node_.Subscribe(gz_image_topic, &GzSensorBridge::onImage, this)) {
          RCLCPP_ERROR(get_logger(), "订阅 gz 图像失败: %s", gz_image_topic.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(), "桥接 gz %s -> ROS %s", gz_image_topic.c_str(),
            ros_image_topic.c_str());
        }
        if (!gz_node_.Subscribe(gz_info_topic, &GzSensorBridge::onCameraInfo, this)) {
          RCLCPP_ERROR(get_logger(), "订阅 gz 相机内参失败: %s", gz_info_topic.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(), "桥接 gz %s -> ROS %s", gz_info_topic.c_str(), ros_info_topic.c_str());
        }

        if (rate_log_period > 0.0) {
          const double period = rate_log_period;
          rate_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            [this, period]() {
              RCLCPP_INFO(
                get_logger(), "相机帧率 %.1f Hz（%zu 帧 / 最近 %.1f s）",
                static_cast<double>(image_count_) / period, image_count_, period);
              image_count_ = 0;
            });
        }
      }
    }

    if (!enable_clock_ && !enable_camera_) {
      RCLCPP_WARN(get_logger(), "时钟与相机都被关闭，本节点不做任何事");
    }
  }

private:
  void onClock(const gz::msgs::Clock & msg)
  {
    if (!clock_pub_) {
      return;
    }
    rosgraph_msgs::msg::Clock out;
    // gz.msgs.Clock 有 system/real/sim 三个时间戳；仿真时间是 system。
    out.clock.sec = msg.system().sec();
    out.clock.nanosec = msg.system().nsec();
    clock_pub_->publish(out);
  }

  void onImage(const gz::msgs::Image & msg)
  {
    if (!image_pub_) {
      return;
    }
    const std::string encoding = gzEncoding(msg.pixel_format_type());
    if (encoding.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "不支持的 gz 像素格式 %d，跳过该帧", static_cast<int>(msg.pixel_format_type()));
      return;
    }

    sensor_msgs::msg::Image out;
    out.header.stamp.sec = msg.header().stamp().sec();
    out.header.stamp.nanosec = msg.header().stamp().nsec();
    out.header.frame_id = gzFrameId(msg.header(), frame_id_);
    out.width = msg.width();
    out.height = msg.height();
    out.step = msg.step() != 0 ? msg.step() : msg.width() * (msg.data().size() / msg.height());
    out.encoding = encoding;
    out.is_bigendian = 0;
    out.data.assign(msg.data().begin(), msg.data().end());
    ++image_count_;
    image_pub_->publish(out);
  }

  void onCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    if (!info_pub_) {
      return;
    }

    sensor_msgs::msg::CameraInfo out;
    out.header.stamp.sec = msg.header().stamp().sec();
    out.header.stamp.nanosec = msg.header().stamp().nsec();
    out.header.frame_id = gzFrameId(msg.header(), frame_id_);
    out.width = msg.width();
    out.height = msg.height();
    out.distortion_model = "plumb_bob";

    const auto & k = msg.intrinsics().k();
    if (k.size() >= 9) {
      // gz 的 Intrinsics.k 直接就是 3x3 内参矩阵 [fx 0 cx; 0 fy cy; 0 0 1]
      for (size_t i = 0; i < 9; ++i) {
        out.k[i] = k[static_cast<int>(i)];
      }
    } else {
      RCLCPP_WARN_ONCE(get_logger(), "gz 相机内参为空，按图像尺寸填中心估计值");
      out.k[0] = static_cast<double>(msg.width()) / 2.0;
      out.k[4] = static_cast<double>(msg.width()) / 2.0;
      out.k[2] = static_cast<double>(msg.width()) / 2.0;
      out.k[5] = static_cast<double>(msg.height()) / 2.0;
      out.k[8] = 1.0;
    }

    out.d.assign(msg.distortion().k().begin(), msg.distortion().k().end());
    out.p[0] = out.k[0];
    out.p[2] = out.k[2];
    out.p[5] = out.k[4];
    out.p[6] = out.k[5];
    out.p[10] = 1.0;
    out.r[0] = 1.0;
    out.r[4] = 1.0;
    out.r[8] = 1.0;
    info_pub_->publish(out);
  }

  bool enable_clock_{false};
  bool enable_camera_{false};
  std::string frame_id_{"camera_link"};

  gz::transport::Node gz_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr rate_timer_;
  size_t image_count_{0};
};

}  // namespace drone_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_bringup::GzSensorBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
