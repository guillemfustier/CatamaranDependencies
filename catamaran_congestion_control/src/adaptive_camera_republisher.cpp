#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "catamaran_congestion_control/adaptive_republisher.hpp"

using catamaran_congestion_control::AdaptiveRepublisherNode;
using catamaran_congestion_control::StreamConfig;
using catamaran_congestion_control::clamp_value;
using CompressedImage = sensor_msgs::msg::CompressedImage;

namespace {

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

}  // namespace

class AdaptiveCameraRepublisher : public AdaptiveRepublisherNode<CompressedImage> {
public:
  AdaptiveCameraRepublisher()
  : AdaptiveRepublisherNode<CompressedImage>(
      "adaptive_camera_republisher",
      "cameras",
      "ros2 run catamaran_congestion_control adaptive_camera_republisher "
      "--ros-args -p config_file:=$(ros2 pkg prefix --share catamaran_congestion_control)/"
      "config/catamaran_cameras.yaml") {}

private:
  std::optional<CompressedImage> process_message(
    const CompressedImage &input,
    const StreamConfig &config,
    int quality) override {
    try {
      const QualityMode quality_mode = parse_quality_mode(config.quality_mode);
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
          config.name.c_str());
        return std::nullopt;
      }

      if (quality_mode == QualityMode::ScaleOnly ||
          quality_mode == QualityMode::ScaleAndJpeg) {
        const double scale = static_cast<double>(quality) / 100.0;
        if (scale < 0.999) {
          const int new_width = std::max(1, static_cast<int>(image.cols * scale));
          const int new_height = std::max(1, static_cast<int>(image.rows * scale));
          cv::resize(image, image, cv::Size(new_width, new_height), 0.0, 0.0, cv::INTER_AREA);
        }
      }

      int jpeg_quality = 95;
      if (quality_mode == QualityMode::JpegOnly ||
          quality_mode == QualityMode::ScaleAndJpeg) {
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
          config.name.c_str());
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
        config.name.c_str(),
        error.what());
      return std::nullopt;
    }
  }
};

int main(int argc, char **argv) {
  return catamaran_congestion_control::run_adaptive_republisher<AdaptiveCameraRepublisher>(
    argc,
    argv,
    "adaptive_camera_republisher");
}
