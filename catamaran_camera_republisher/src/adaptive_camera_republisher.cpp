#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using CompressedImage = sensor_msgs::msg::CompressedImage;
using Int32MultiArray = std_msgs::msg::Int32MultiArray;

namespace {

template <typename T>
T clamp_value(T value, T min_value, T max_value) {
  return std::max(min_value, std::min(value, max_value));
}

std::string required_string(const YAML::Node &node, const std::string &key) {
  if (!node[key]) {
    throw std::runtime_error("Missing required YAML field: " + key);
  }
  return node[key].as<std::string>();
}

}  // namespace

enum class QualityMode {
  ScaleOnly,
  JpegOnly,
  ScaleAndJpeg,
};

QualityMode parse_quality_mode(const std::string &mode) {
  if (mode == "scale_only") {
    return QualityMode::ScaleOnly;
  }
  if (mode == "jpeg_only") {
    return QualityMode::JpegOnly;
  }
  if (mode == "scale_and_jpeg") {
    return QualityMode::ScaleAndJpeg;
  }
  throw std::runtime_error(
    "Invalid quality_mode '" + mode + "'. Use scale_only, jpeg_only or scale_and_jpeg.");
}

struct CameraConfig {
  std::string name;
  std::string input_topic;
  std::string output_topic;
  std::string control_topic;

  int max_fps{30};
  int default_fps{30};
  int default_quality{60};
  int min_active_fps{1};
  int min_active_quality{5};
  std::string quality_mode_text{"scale_and_jpeg"};
  QualityMode quality_mode{QualityMode::ScaleAndJpeg};
};

struct CameraRuntime {
  explicit CameraRuntime(const CameraConfig &camera_config)
  : config(camera_config),
    target_fps(camera_config.default_fps),
    target_quality(camera_config.default_quality) {}

  CameraConfig config;

  std::mutex mutex;
  CompressedImage::ConstSharedPtr last_image;
  int64_t last_image_time_ns{0};
  int64_t last_publish_time_ns{0};

  int target_fps{0};
  int target_quality{0};

  rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
  rclcpp::Subscription<CompressedImage>::SharedPtr image_subscription;
  rclcpp::Subscription<Int32MultiArray>::SharedPtr control_subscription;
};

class AdaptiveCameraRepublisher : public rclcpp::Node {
public:
  AdaptiveCameraRepublisher()
  : Node("adaptive_camera_republisher") {
    const std::string config_file = this->declare_parameter<std::string>("config_file", "");
    if (config_file.empty()) {
      throw std::runtime_error(
        "Missing parameter 'config_file'. Example: "
        "ros2 run catamaran_camera_republisher adaptive_camera_republisher "
        "--ros-args -p config_file:=$(ros2 pkg prefix --share catamaran_camera_republisher)/config/catamaran_cameras.yaml");
    }

    load_config(config_file);
    create_ros_interfaces();

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / scheduler_hz_));

    scheduler_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AdaptiveCameraRepublisher::on_scheduler_timer, this));

    RCLCPP_INFO(this->get_logger(), "Adaptive camera republisher started");
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file.c_str());
  }

private:
  double scheduler_hz_{50.0};
  double max_stale_sec_{2.0};

  std::vector<std::shared_ptr<CameraRuntime>> cameras_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;

  rclcpp::QoS image_qos_{rclcpp::KeepLast(1)};
  rclcpp::QoS control_qos_{rclcpp::KeepLast(1)};

  void load_config(const std::string &config_file) {
    const YAML::Node root = YAML::LoadFile(config_file);

    const YAML::Node node_cfg = root["node"] ? root["node"] : YAML::Node(YAML::NodeType::Map);
    const YAML::Node defaults = root["defaults"] ? root["defaults"] : YAML::Node(YAML::NodeType::Map);
    const YAML::Node cameras = root["cameras"];

    if (!cameras || !cameras.IsSequence() || cameras.size() == 0) {
      throw std::runtime_error("Config must contain a non-empty 'cameras' list");
    }

    scheduler_hz_ = node_cfg["scheduler_hz"] ? node_cfg["scheduler_hz"].as<double>() : 50.0;
    max_stale_sec_ = node_cfg["max_stale_sec"] ? node_cfg["max_stale_sec"].as<double>() : 2.0;

    if (scheduler_hz_ <= 0.0) {
      throw std::runtime_error("node.scheduler_hz must be greater than 0");
    }

    const int default_max_fps = defaults["max_fps"] ? defaults["max_fps"].as<int>() : 30;
    const int default_fps = defaults["default_fps"] ? defaults["default_fps"].as<int>() : 5;
    const int default_quality = defaults["default_quality"] ? defaults["default_quality"].as<int>() : 60;
    const int min_active_fps = defaults["min_active_fps"] ? defaults["min_active_fps"].as<int>() : 1;
    const int min_active_quality = defaults["min_active_quality"] ? defaults["min_active_quality"].as<int>() : 20;
    const std::string quality_mode_text = defaults["quality_mode"] ?
      defaults["quality_mode"].as<std::string>() : "scale_and_jpeg";

    std::unordered_map<std::string, bool> used_names;

    for (const auto &camera_node : cameras) {
      CameraConfig config;
      config.name = required_string(camera_node, "name");

      if (used_names[config.name]) {
        throw std::runtime_error("Duplicated camera name in config: " + config.name);
      }
      used_names[config.name] = true;

      config.input_topic = required_string(camera_node, "input_topic");
      config.output_topic = required_string(camera_node, "output_topic");
      config.control_topic = required_string(camera_node, "control_topic");

      config.max_fps = camera_node["max_fps"] ? camera_node["max_fps"].as<int>() : default_max_fps;
      config.default_fps = camera_node["default_fps"] ? camera_node["default_fps"].as<int>() : default_fps;
      config.default_quality = camera_node["default_quality"] ? camera_node["default_quality"].as<int>() : default_quality;
      config.min_active_fps = camera_node["min_active_fps"] ? camera_node["min_active_fps"].as<int>() : min_active_fps;
      config.min_active_quality = camera_node["min_active_quality"] ? camera_node["min_active_quality"].as<int>() : min_active_quality;
      config.quality_mode_text = camera_node["quality_mode"] ?
        camera_node["quality_mode"].as<std::string>() : quality_mode_text;
      config.quality_mode = parse_quality_mode(config.quality_mode_text);

      if (config.max_fps <= 0) {
        throw std::runtime_error("Camera '" + config.name + "' must have max_fps > 0");
      }

      config.min_active_fps = clamp_value(config.min_active_fps, 1, config.max_fps);
      config.min_active_quality = clamp_value(config.min_active_quality, 1, 100);
      config.default_fps = clamp_value(config.default_fps, 0, config.max_fps);
      config.default_quality = clamp_value(config.default_quality, 0, 100);

      if (config.default_fps > 0 && config.default_quality > 0) {
        config.default_fps = clamp_value(config.default_fps, config.min_active_fps, config.max_fps);
        config.default_quality = clamp_value(config.default_quality, config.min_active_quality, 100);
      }

      cameras_.push_back(std::make_shared<CameraRuntime>(config));
    }
  }

  void create_ros_interfaces() {
    image_qos_.best_effort();
    image_qos_.durability_volatile();

    control_qos_.reliable();
    control_qos_.durability_volatile();

    for (auto &camera : cameras_) {
      camera->publisher = this->create_publisher<CompressedImage>(
        camera->config.output_topic,
        image_qos_);

      camera->image_subscription = this->create_subscription<CompressedImage>(
        camera->config.input_topic,
        image_qos_,
        [this, camera](CompressedImage::ConstSharedPtr msg) {
          this->on_image(camera, msg);
        });

      camera->control_subscription = this->create_subscription<Int32MultiArray>(
        camera->config.control_topic,
        control_qos_,
        [this, camera](Int32MultiArray::ConstSharedPtr msg) {
          this->on_control(camera, msg);
        });

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] input=%s output=%s control=%s default=[%d fps, %d quality] mode=%s",
        camera->config.name.c_str(),
        camera->config.input_topic.c_str(),
        camera->config.output_topic.c_str(),
        camera->config.control_topic.c_str(),
        camera->config.default_fps,
        camera->config.default_quality,
        camera->config.quality_mode_text.c_str());
    }
  }

  void on_image(
    const std::shared_ptr<CameraRuntime> &camera,
    const CompressedImage::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(camera->mutex);
    camera->last_image = msg;
    camera->last_image_time_ns = this->now().nanoseconds();
  }

  void on_control(
    const std::shared_ptr<CameraRuntime> &camera,
    const Int32MultiArray::ConstSharedPtr &msg) {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] invalid control message. Expected [fps, quality]",
        camera->config.name.c_str());
      return;
    }

    const int requested_fps = msg->data[0];
    const int requested_quality = msg->data[1];

    int new_fps = 0;
    int new_quality = 0;

    if (requested_fps > 0 && requested_quality > 0) {
      new_fps = clamp_value(requested_fps, camera->config.min_active_fps, camera->config.max_fps);
      new_quality = clamp_value(requested_quality, camera->config.min_active_quality, 100);
    }

    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(camera->mutex);
      changed = camera->target_fps != new_fps || camera->target_quality != new_quality;
      camera->target_fps = new_fps;
      camera->target_quality = new_quality;
    }

    if (changed) {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] control -> [%d fps, %d quality]",
        camera->config.name.c_str(),
        new_fps,
        new_quality);
    }
  }

  void on_scheduler_timer() {
    const int64_t now_ns = this->now().nanoseconds();

    for (const auto &camera : cameras_) {
      CompressedImage::ConstSharedPtr image;
      int target_fps = 0;
      int target_quality = 0;
      int64_t last_publish_time_ns = 0;
      int64_t last_image_time_ns = 0;

      {
        std::lock_guard<std::mutex> lock(camera->mutex);
        image = camera->last_image;
        target_fps = camera->target_fps;
        target_quality = camera->target_quality;
        last_publish_time_ns = camera->last_publish_time_ns;
        last_image_time_ns = camera->last_image_time_ns;
      }

      if (!image || target_fps <= 0 || target_quality <= 0) {
        continue;
      }

      const int64_t publish_period_ns = static_cast<int64_t>(1e9 / static_cast<double>(target_fps));
      if (now_ns - last_publish_time_ns < publish_period_ns) {
        continue;
      }

      if (max_stale_sec_ > 0.0) {
        const double image_age_sec = static_cast<double>(now_ns - last_image_time_ns) / 1e9;
        if (image_age_sec > max_stale_sec_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[%s] last image is stale: %.2f s",
            camera->config.name.c_str(),
            image_age_sec);
          continue;
        }
      }

      auto output = process_image(*image, *camera, target_quality);
      if (!output) {
        continue;
      }

      camera->publisher->publish(std::move(*output));

      {
        std::lock_guard<std::mutex> lock(camera->mutex);
        camera->last_publish_time_ns = now_ns;
      }
    }
  }

  std::optional<CompressedImage> process_image(
    const CompressedImage &input,
    const CameraRuntime &camera,
    int quality) {
    try {
      const cv::Mat compressed_mat(
        1,
        static_cast<int>(input.data.size()),
        CV_8UC1,
        const_cast<unsigned char *>(input.data.data()));

      cv::Mat image = cv::imdecode(compressed_mat, cv::IMREAD_COLOR);
      if (image.empty()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "[%s] failed to decode compressed image",
          camera.config.name.c_str());
        return std::nullopt;
      }

      if (camera.config.quality_mode == QualityMode::ScaleOnly ||
          camera.config.quality_mode == QualityMode::ScaleAndJpeg) {
        const double scale = static_cast<double>(quality) / 100.0;
        if (scale < 0.999) {
          const int new_width = std::max(1, static_cast<int>(image.cols * scale));
          const int new_height = std::max(1, static_cast<int>(image.rows * scale));
          cv::resize(image, image, cv::Size(new_width, new_height), 0.0, 0.0, cv::INTER_AREA);
        }
      }

      int jpeg_quality = 95;
      if (camera.config.quality_mode == QualityMode::JpegOnly ||
          camera.config.quality_mode == QualityMode::ScaleAndJpeg) {
        jpeg_quality = clamp_value(quality, 5, 100);
      }

      std::vector<unsigned char> encoded;
      const std::vector<int> encode_params = {
        cv::IMWRITE_JPEG_QUALITY,
        jpeg_quality,
      };

      if (!cv::imencode(".jpg", image, encoded, encode_params)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "[%s] failed to encode output image",
          camera.config.name.c_str());
        return std::nullopt;
      }

      CompressedImage output;
      output.header.stamp = this->get_clock()->now();
      output.header.frame_id = input.header.frame_id;
      output.format = "jpeg";
      output.data.assign(encoded.begin(), encoded.end());
      return output;
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[%s] image processing error: %s",
        camera.config.name.c_str(),
        error.what());
      return std::nullopt;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<AdaptiveCameraRepublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("adaptive_camera_republisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
