#include <cstring>
#include <exception>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "catamaran_camera_republisher/adaptive_republisher.hpp"

using catamaran_camera_republisher::AdaptiveRepublisherNode;
using catamaran_camera_republisher::StreamConfig;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class AdaptiveLidarRepublisher : public AdaptiveRepublisherNode<PointCloud2> {
public:
  AdaptiveLidarRepublisher()
  : AdaptiveRepublisherNode<PointCloud2>(
      "adaptive_lidar_republisher",
      "streams",
      "ros2 run catamaran_camera_republisher adaptive_lidar_republisher "
      "--ros-args -p config_file:=$(ros2 pkg prefix --share catamaran_camera_republisher)/"
      "config/catamaran_lidar.yaml") {}

private:
  std::optional<PointCloud2> process_message(
    const PointCloud2 &input,
    const StreamConfig &config,
    int quality) override {
    try {
      PointCloud2 output = input;
      output.header.stamp = this->get_clock()->now();

      if (quality >= 100) {
        return output;
      }

      if (input.point_step == 0 || input.width == 0 || input.height == 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "[%s] invalid point cloud shape",
          config.name.c_str());
        return std::nullopt;
      }

      const size_t declared_points =
        static_cast<size_t>(input.width) * static_cast<size_t>(input.height);
      const size_t keep_points = std::max<size_t>(
        1,
        (declared_points * static_cast<size_t>(quality) + 99) / 100);

      output.height = 1;
      output.width = static_cast<uint32_t>(keep_points);
      output.row_step = output.width * output.point_step;
      output.data.resize(static_cast<size_t>(output.row_step));

      size_t copied_points = 0;
      for (size_t dst_index = 0; dst_index < keep_points; ++dst_index) {
        const size_t src_index = (dst_index * declared_points) / keep_points;
        const size_t src_row = src_index / input.width;
        const size_t src_col = src_index % input.width;
        const size_t src_offset = src_row * input.row_step + src_col * input.point_step;
        if (src_offset + input.point_step > input.data.size()) {
          continue;
        }

        const size_t dst_offset = copied_points * output.point_step;
        std::memcpy(
          output.data.data() + dst_offset,
          input.data.data() + src_offset,
          input.point_step);
        ++copied_points;
      }

      if (copied_points == 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "[%s] point cloud data is smaller than declared shape",
          config.name.c_str());
        return std::nullopt;
      }

      output.width = static_cast<uint32_t>(copied_points);
      output.row_step = output.width * output.point_step;
      output.data.resize(static_cast<size_t>(output.row_step));
      return output;
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[%s] lidar processing error: %s",
        config.name.c_str(),
        error.what());
      return std::nullopt;
    }
  }
};

int main(int argc, char **argv) {
  return catamaran_camera_republisher::run_adaptive_republisher<AdaptiveLidarRepublisher>(
    argc,
    argv,
    "adaptive_lidar_republisher");
}
