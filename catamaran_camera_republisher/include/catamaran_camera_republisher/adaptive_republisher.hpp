#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <yaml-cpp/yaml.h>

namespace catamaran_camera_republisher {

template <typename T>
T clamp_value(T value, T min_value, T max_value) {
  return std::max(min_value, std::min(value, max_value));
}

inline std::string required_string(const YAML::Node &node, const std::string &key) {
  if (!node[key]) {
    throw std::runtime_error("Missing required YAML field: " + key);
  }
  return node[key].as<std::string>();
}

struct StreamConfig {
  std::string name;
  std::string input_topic;
  std::string output_topic;
  std::string control_topic;

  int max_fps{30};
  int default_fps{30};
  int default_quality{60};
  int min_active_fps{1};
  int min_active_quality{5};
  std::string quality_mode{"scale_and_jpeg"};
};

template <typename MessageT>
struct StreamRuntime {
  explicit StreamRuntime(const StreamConfig &stream_config)
  : config(stream_config),
    target_fps(stream_config.default_fps),
    target_quality(stream_config.default_quality) {}

  StreamConfig config;

  std::mutex state_mutex;
  std::mutex processing_mutex;
  typename MessageT::ConstSharedPtr last_message;
  int64_t last_message_time_ns{0};
  int64_t last_publish_time_ns{0};

  int target_fps{0};
  int target_quality{0};

  typename rclcpp::Publisher<MessageT>::SharedPtr publisher;
  typename rclcpp::Subscription<MessageT>::SharedPtr data_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr control_subscription;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

template <typename MessageT>
class AdaptiveRepublisherNode : public rclcpp::Node {
public:
  AdaptiveRepublisherNode(
    const std::string &node_name,
    const std::string &streams_key,
    const std::string &config_hint)
  : Node(node_name),
    streams_key_(streams_key) {
    const std::string config_file = this->declare_parameter<std::string>("config_file", "");
    worker_threads_ = this->declare_parameter<int>("worker_threads", 0);
    if (worker_threads_ < 0) {
      throw std::runtime_error("worker_threads must be >= 0");
    }

    if (config_file.empty()) {
      throw std::runtime_error("Missing parameter 'config_file'. Example: " + config_hint);
    }

    load_config(config_file);
    create_ros_interfaces();

    RCLCPP_INFO(this->get_logger(), "%s started", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Executor threads: %s",
      worker_threads_ == 0 ? "hardware_concurrency" : std::to_string(worker_threads_).c_str());
  }

  size_t worker_threads() const {
    return static_cast<size_t>(worker_threads_);
  }

protected:
  virtual std::optional<MessageT> process_message(
    const MessageT &input,
    const StreamConfig &config,
    int quality) = 0;

private:
  std::string streams_key_;
  double scheduler_hz_{50.0};
  double max_stale_sec_{2.0};
  int worker_threads_{0};

  std::vector<std::shared_ptr<StreamRuntime<MessageT>>> streams_;
  rclcpp::QoS data_qos_{rclcpp::KeepLast(1)};
  rclcpp::QoS control_qos_{rclcpp::KeepLast(1)};

  void load_config(const std::string &config_file) {
    const YAML::Node root = YAML::LoadFile(config_file);

    const YAML::Node node_cfg = root["node"] ? root["node"] : YAML::Node(YAML::NodeType::Map);
    const YAML::Node defaults = root["defaults"] ? root["defaults"] : YAML::Node(YAML::NodeType::Map);
    const YAML::Node streams = root[streams_key_];

    if (!streams || !streams.IsSequence() || streams.size() == 0) {
      throw std::runtime_error("Config must contain a non-empty '" + streams_key_ + "' list");
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
    const std::string quality_mode = defaults["quality_mode"] ?
      defaults["quality_mode"].as<std::string>() : "scale_and_jpeg";

    std::unordered_map<std::string, bool> used_names;

    for (const auto &stream_node : streams) {
      StreamConfig config;
      config.name = required_string(stream_node, "name");

      if (used_names[config.name]) {
        throw std::runtime_error("Duplicated stream name in config: " + config.name);
      }
      used_names[config.name] = true;

      config.input_topic = required_string(stream_node, "input_topic");
      config.output_topic = required_string(stream_node, "output_topic");
      config.control_topic = required_string(stream_node, "control_topic");

      config.max_fps = stream_node["max_fps"] ? stream_node["max_fps"].as<int>() : default_max_fps;
      config.default_fps = stream_node["default_fps"] ? stream_node["default_fps"].as<int>() : default_fps;
      config.default_quality = stream_node["default_quality"] ?
        stream_node["default_quality"].as<int>() : default_quality;
      config.min_active_fps = stream_node["min_active_fps"] ?
        stream_node["min_active_fps"].as<int>() : min_active_fps;
      config.min_active_quality = stream_node["min_active_quality"] ?
        stream_node["min_active_quality"].as<int>() : min_active_quality;
      config.quality_mode = stream_node["quality_mode"] ?
        stream_node["quality_mode"].as<std::string>() : quality_mode;

      if (config.max_fps <= 0) {
        throw std::runtime_error("Stream '" + config.name + "' must have max_fps > 0");
      }

      config.min_active_fps = clamp_value(config.min_active_fps, 1, config.max_fps);
      config.min_active_quality = clamp_value(config.min_active_quality, 1, 100);
      config.default_fps = clamp_value(config.default_fps, 0, config.max_fps);
      config.default_quality = clamp_value(config.default_quality, 0, 100);

      if (config.default_fps > 0 && config.default_quality > 0) {
        config.default_fps = clamp_value(config.default_fps, config.min_active_fps, config.max_fps);
        config.default_quality = clamp_value(config.default_quality, config.min_active_quality, 100);
      }

      streams_.push_back(std::make_shared<StreamRuntime<MessageT>>(config));
    }
  }

  void create_ros_interfaces() {
    data_qos_.best_effort();
    data_qos_.durability_volatile();

    control_qos_.reliable();
    control_qos_.durability_volatile();

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / scheduler_hz_));

    for (auto &stream : streams_) {
      stream->callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      rclcpp::SubscriptionOptions subscription_options;
      subscription_options.callback_group = stream->callback_group;

      stream->publisher = this->create_publisher<MessageT>(
        stream->config.output_topic,
        data_qos_);

      stream->data_subscription = this->create_subscription<MessageT>(
        stream->config.input_topic,
        data_qos_,
        [this, stream](typename MessageT::ConstSharedPtr msg) {
          this->on_message(stream, msg);
        },
        subscription_options);

      stream->control_subscription = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        stream->config.control_topic,
        control_qos_,
        [this, stream](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
          this->on_control(stream, msg);
        },
        subscription_options);

      stream->timer = this->create_wall_timer(
        timer_period,
        [this, stream]() {
          this->on_stream_timer(stream);
        },
        stream->callback_group);

      RCLCPP_INFO(
        this->get_logger(),
        "[%s] input=%s output=%s control=%s default=[%d fps, %d quality]",
        stream->config.name.c_str(),
        stream->config.input_topic.c_str(),
        stream->config.output_topic.c_str(),
        stream->config.control_topic.c_str(),
        stream->config.default_fps,
        stream->config.default_quality);
    }
  }

  void on_message(
    const std::shared_ptr<StreamRuntime<MessageT>> &stream,
    const typename MessageT::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(stream->state_mutex);
    stream->last_message = msg;
    stream->last_message_time_ns = this->now().nanoseconds();
  }

  void on_control(
    const std::shared_ptr<StreamRuntime<MessageT>> &stream,
    const std_msgs::msg::Int32MultiArray::ConstSharedPtr &msg) {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] invalid control message. Expected [fps, quality]",
        stream->config.name.c_str());
      return;
    }

    const int requested_fps = msg->data[0];
    const int requested_quality = msg->data[1];

    int new_fps = 0;
    int new_quality = 0;

    if (requested_fps > 0 && requested_quality > 0) {
      new_fps = clamp_value(requested_fps, stream->config.min_active_fps, stream->config.max_fps);
      new_quality = clamp_value(requested_quality, stream->config.min_active_quality, 100);
    }

    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(stream->state_mutex);
      changed = stream->target_fps != new_fps || stream->target_quality != new_quality;
      stream->target_fps = new_fps;
      stream->target_quality = new_quality;
    }

    if (changed) {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] control -> [%d fps, %d quality]",
        stream->config.name.c_str(),
        new_fps,
        new_quality);
    }
  }

  void on_stream_timer(const std::shared_ptr<StreamRuntime<MessageT>> &stream) {
    const int64_t now_ns = this->now().nanoseconds();

    typename MessageT::ConstSharedPtr message;
    int target_fps = 0;
    int target_quality = 0;
    int64_t last_publish_time_ns = 0;
    int64_t last_message_time_ns = 0;

    {
      std::lock_guard<std::mutex> lock(stream->state_mutex);
      message = stream->last_message;
      target_fps = stream->target_fps;
      target_quality = stream->target_quality;
      last_publish_time_ns = stream->last_publish_time_ns;
      last_message_time_ns = stream->last_message_time_ns;
    }

    if (!message || target_fps <= 0 || target_quality <= 0) {
      return;
    }

    const int64_t publish_period_ns = static_cast<int64_t>(1e9 / static_cast<double>(target_fps));
    if (now_ns - last_publish_time_ns < publish_period_ns) {
      return;
    }

    if (max_stale_sec_ > 0.0) {
      const double message_age_sec = static_cast<double>(now_ns - last_message_time_ns) / 1e9;
      if (message_age_sec > max_stale_sec_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "[%s] last message is stale: %.2f s",
          stream->config.name.c_str(),
          message_age_sec);
        return;
      }
    }

    std::unique_lock<std::mutex> processing_lock(stream->processing_mutex, std::try_to_lock);
    if (!processing_lock.owns_lock()) {
      return;
    }

    auto output = process_message(*message, stream->config, target_quality);
    if (!output) {
      return;
    }

    stream->publisher->publish(std::move(*output));

    {
      std::lock_guard<std::mutex> lock(stream->state_mutex);
      stream->last_publish_time_ns = this->now().nanoseconds();
    }
  }
};

template <typename NodeT>
int run_adaptive_republisher(int argc, char **argv, const std::string &logger_name) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<NodeT>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      node->worker_threads());
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

}  // namespace catamaran_camera_republisher
