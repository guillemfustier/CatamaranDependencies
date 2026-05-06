#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

class CameraCompressedPublisher : public rclcpp::Node {
public:
  CameraCompressedPublisher() : Node("camera_compressed_publisher") {
    camera_name_ = this->declare_parameter<std::string>("camera_name", "camera");
    video_device_ = this->declare_parameter<std::string>("video_device", "/dev/video0");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/camera/image/compressed");
    frame_id_ = this->declare_parameter<std::string>("frame_id", camera_name_);
    image_width_ = this->declare_parameter<int>("image_width", 1920);
    image_height_ = this->declare_parameter<int>("image_height", 1080);
    fps_ = this->declare_parameter<double>("fps", 30.0);
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 85);
    fourcc_ = this->declare_parameter<std::string>("fourcc", "MJPG");

    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_, 10);

    open_camera();

    auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CameraCompressedPublisher::publish_frame, this));
  }

private:
  std::string camera_name_;
  std::string video_device_;
  std::string topic_name_;
  std::string frame_id_;
  int image_width_{0};
  int image_height_{0};
  double fps_{30.0};
  int jpeg_quality_{85};
  std::string fourcc_{"MJPG"};

  cv::VideoCapture camera_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  static std::string resolve_device_path(const std::string &path) {
    try {
      return std::filesystem::weakly_canonical(path).string();
    } catch (...) {
      return path;
    }
  }

  void open_camera() {
    const std::string resolved_device = resolve_device_path(video_device_);
    constexpr int kMaxAttempts = 30;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
      camera_.open(resolved_device, cv::CAP_V4L2);
      if (camera_.isOpened()) {
        break;
      }

      RCLCPP_WARN(
        this->get_logger(),
        "[%s] unable to open %s (attempt %d/%d), retrying...",
        camera_name_.c_str(),
        resolved_device.c_str(),
        attempt,
        kMaxAttempts);

      if (attempt == kMaxAttempts) {
        throw std::runtime_error("Unable to open camera device: " + resolved_device);
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(image_width_));
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(image_height_));
    camera_.set(cv::CAP_PROP_FPS, fps_);

    if (fourcc_.size() == 4) {
      const int codec = cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
      camera_.set(cv::CAP_PROP_FOURCC, static_cast<double>(codec));
    }

    RCLCPP_INFO(this->get_logger(), "[%s] publishing %s from %s", camera_name_.c_str(), topic_name_.c_str(), resolved_device.c_str());
  }

  void publish_frame() {
    cv::Mat frame;
    camera_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[%s] empty frame from %s", camera_name_.c_str(), video_device_.c_str());
      return;
    }

    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", frame, buffer, params)) {
      RCLCPP_WARN(this->get_logger(), "[%s] failed to encode JPEG", camera_name_.c_str());
      return;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = std::move(buffer);
    publisher_->publish(std::move(msg));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CameraCompressedPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_compressed_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}