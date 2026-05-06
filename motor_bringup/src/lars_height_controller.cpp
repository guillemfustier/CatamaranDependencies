///////////////////////////////////////////////////////////////////////////////
//                   *** DYNAMIXEL LARS HEIGHT CONTROLLER ***
//
// Subscribes to an absolute LARS height target in centimeters and drives two
// synchronized linear axes motors (IDs 3 and 4). The node also publishes the
// current estimated LARS height so other components can compare goal vs actual.
///////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>                  
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/float32.hpp"

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11
#define ADDR_VELOCITY_LIMIT 44
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define VELOCITY_UNIT 0.229

// LARS system parameters
constexpr double kInitialHeightCm = 24.0;
constexpr double kMaxHeightCm = 35.0;
constexpr double kMinHeightCm = 0.0;
constexpr double kMotorTurnsPerCm = 5.0;
constexpr double kMaxMotorRpm = 60.0; // default, can be overridden via parameter
constexpr int32_t kVelocityLimit = 1023;

// Two synchronized motors for the linear axes
constexpr uint8_t kMotorIdLeft = 3;
constexpr uint8_t kMotorIdRight = 4;

dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;

int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

class LarsHeightController : public rclcpp::Node {
public:
    LarsHeightController()
    : Node("lars_height_controller"),
      current_height_cm_(kInitialHeightCm),
            max_height_cm_(kMaxHeightCm),
            max_motor_rpm_(kMaxMotorRpm),
            target_height_cm_(kInitialHeightCm),
            motion_active_(false),
            active_velocity_command_(0),
            motion_duration_s_(0.0) {

        RCLCPP_INFO(this->get_logger(), "LARS Height Controller node started");

        this->declare_parameter("qos_depth", 10);
        this->declare_parameter("device_name", std::string(DEFAULT_DEVICE_NAME));
        this->declare_parameter("goal_topic", std::string("/lars_height"));
        this->declare_parameter("actual_topic", std::string("/lars_height_actual"));
        this->declare_parameter("initial_height", kInitialHeightCm);
        this->declare_parameter("max_height", kMaxHeightCm);
        this->declare_parameter("max_motor_rpm", kMaxMotorRpm);

        int8_t qos_depth = 10;
        this->get_parameter("qos_depth", qos_depth);
        this->get_parameter("initial_height", current_height_cm_);
        this->get_parameter("max_height", max_height_cm_);
        this->get_parameter("max_motor_rpm", max_motor_rpm_);

        std::string goal_topic;
        std::string actual_topic;
        this->get_parameter("goal_topic", goal_topic);
        this->get_parameter("actual_topic", actual_topic);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        actual_publisher_ = this->create_publisher<std_msgs::msg::Float32>(actual_topic, qos);
        goal_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            goal_topic,
            qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->handleGoal(msg->data);
            });

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                this->processMotion();
            });

        publishCurrentHeight();

        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &p : params) {
                    if (p.get_name() == "max_motor_rpm") {
                        const double val = p.as_double();
                        if (val <= 0.0) {
                            result.successful = false;
                            result.reason = "max_motor_rpm must be > 0";
                            return result;
                        }
                        max_motor_rpm_ = val;
                    }
                }
                return result;
            });
    }

    void handleGoal(double requested_height_cm) {
        const double goal_height_cm = std::clamp(requested_height_cm, kMinHeightCm, max_height_cm_);

        if (goal_height_cm != requested_height_cm) {
            RCLCPP_WARN(
                this->get_logger(),
                "Requested height %.3f cm was clamped to %.3f cm",
                requested_height_cm,
                goal_height_cm);
        }

        target_height_cm_ = goal_height_cm;
        replanMotion();
    }

private:
    void processMotion() {
        if (!motion_active_) {
            return;
        }

        updateEstimatedHeight();

        if (!motion_active_ && active_velocity_command_ != 0) {
            stopMotors();
        }
    }

    void replanMotion() {
        updateEstimatedHeight();

        const double delta_cm = target_height_cm_ - current_height_cm_;
        if (std::abs(delta_cm) < 1e-6) {
            if (active_velocity_command_ != 0) {
                stopMotors();
            }
            publishCurrentHeight();
            return;
        }

        const double motor_turns = std::abs(delta_cm) * kMotorTurnsPerCm;

        int32_t requested_velocity = static_cast<int32_t>(std::round(max_motor_rpm_ / VELOCITY_UNIT));
        const int32_t safe_velocity_limit = readSafeVelocityLimit();
        requested_velocity = std::max(requested_velocity, 1);
        const int32_t velocity_magnitude = std::clamp(requested_velocity, 1, safe_velocity_limit);
        const double effective_motor_rpm = static_cast<double>(velocity_magnitude) * VELOCITY_UNIT;

        if (velocity_magnitude < requested_velocity) {
            RCLCPP_WARN(
                this->get_logger(),
                "Requested RPM %.3f exceeds motor velocity limit (raw %d). Using %.3f RPM.",
                max_motor_rpm_,
                safe_velocity_limit,
                effective_motor_rpm);
        }

        double motion_time_s = (motor_turns / effective_motor_rpm) * 60.0;
        motion_time_s = std::round(motion_time_s * 100.0) / 100.0;

        int32_t velocity_command = velocity_magnitude;

        const bool should_raise = delta_cm > 0.0;
        if (!should_raise) {
            velocity_command = -velocity_command;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: %.3f cm | Current: %.3f cm | Delta: %.3f cm | Motor turns: %.3f | Time: %.3f s",
            target_height_cm_,
            current_height_cm_,
            delta_cm,
            motor_turns,
            motion_time_s);

        if (velocity_command != active_velocity_command_) {
            if (!writeVelocityBoth(velocity_command)) {
                return;
            }
            active_velocity_command_ = velocity_command;
        }

        motion_active_ = true;
        motion_start_time_ = this->now();
        motion_start_height_cm_ = current_height_cm_;
        motion_goal_height_cm_ = target_height_cm_;
        motion_duration_s_ = motion_time_s;
    }

    void updateEstimatedHeight() {
        if (!motion_active_) {
            return;
        }

        if (motion_duration_s_ <= 0.0) {
            current_height_cm_ = motion_goal_height_cm_;
            motion_active_ = false;
            publishCurrentHeight();
            return;
        }

        const double elapsed_s = (this->now() - motion_start_time_).seconds();
        const double progress = std::clamp(elapsed_s / motion_duration_s_, 0.0, 1.0);

        current_height_cm_ =
            motion_start_height_cm_ + (motion_goal_height_cm_ - motion_start_height_cm_) * progress;

        if (progress >= 1.0) {
            motion_active_ = false;
            publishCurrentHeight();
        }
    }

    void stopMotors() {
        if (active_velocity_command_ == 0) {
            return;
        }

        if (!writeVelocityBoth(0)) {
            return;
        }

        active_velocity_command_ = 0;
    }

    int32_t readMotorVelocityLimit(uint8_t motor_id) {
        uint8_t local_error = 0;
        uint32_t velocity_limit_raw = 0;
        const int comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            motor_id,
            ADDR_VELOCITY_LIMIT,
            &velocity_limit_raw,
            &local_error);

        if (comm_result != COMM_SUCCESS) {
            RCLCPP_WARN(
                this->get_logger(),
                "Motor %d: could not read velocity limit (%s). Falling back to %d.",
                motor_id,
                packetHandler->getTxRxResult(comm_result),
                kVelocityLimit);
            return kVelocityLimit;
        }

        if (local_error != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Motor %d: velocity limit read error (%s). Falling back to %d.",
                motor_id,
                packetHandler->getRxPacketError(local_error),
                kVelocityLimit);
            return kVelocityLimit;
        }

        if (velocity_limit_raw == 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Motor %d: reported velocity limit is 0. Falling back to %d.",
                motor_id,
                kVelocityLimit);
            return kVelocityLimit;
        }

        return std::min(static_cast<int32_t>(velocity_limit_raw), kVelocityLimit);
    }

    int32_t readSafeVelocityLimit() {
        const int32_t left_limit = readMotorVelocityLimit(kMotorIdLeft);
        const int32_t right_limit = readMotorVelocityLimit(kMotorIdRight);
        return std::max(1, std::min(left_limit, right_limit));
    }

    bool writeVelocityBoth(int32_t velocity) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorIdLeft,
            ADDR_GOAL_VELOCITY,
            velocity,
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdLeft, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdLeft, packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorIdRight,
            ADDR_GOAL_VELOCITY,
            velocity,
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        if (velocity == 0) {
            RCLCPP_INFO(this->get_logger(), "Both LARS motors stopped");
        } else {
            RCLCPP_INFO(this->get_logger(), "Both LARS motors running at velocity %d", velocity);
        }

        return true;
    }

    void publishCurrentHeight() {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(current_height_cm_);
        actual_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published current LARS height: %.3f cm", current_height_cm_);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double current_height_cm_;
    double max_height_cm_;
    double max_motor_rpm_;
    double target_height_cm_;
    bool motion_active_;
    int32_t active_velocity_command_;
    rclcpp::Time motion_start_time_;
    double motion_start_height_cm_;
    double motion_goal_height_cm_;
    double motion_duration_s_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

void setupDynamixel(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to disable torque for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to set Velocity Control mode for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("lars_height_controller"), "Velocity Control mode set for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to enable torque for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("lars_height_controller"), "Torque enabled for ID %d.", dxl_id);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto lars_controller = std::make_shared<LarsHeightController>();

    std::string deviceNameString;
    if (!lars_controller->get_parameter("device_name", deviceNameString)) {
        deviceNameString = DEFAULT_DEVICE_NAME;
    }

    if (deviceNameString.empty()) {
        deviceNameString = DEFAULT_DEVICE_NAME;
    }

    const char * deviceName = deviceNameString.c_str();

    RCLCPP_INFO(lars_controller->get_logger(), "Attempting to open Dynamixel port: %s", deviceName);

    portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(lars_controller->get_logger(), "Failed to open the port!");
        return -1;
    }

    RCLCPP_INFO(lars_controller->get_logger(), "Succeeded to open the port.");

    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(lars_controller->get_logger(), "Failed to set the baudrate!");
        return -1;
    }

    RCLCPP_INFO(lars_controller->get_logger(), "Succeeded to set the baudrate.");

    setupDynamixel(kMotorIdLeft);
    setupDynamixel(kMotorIdRight);

    rclcpp::spin(lars_controller);

    rclcpp::shutdown();

    packetHandler->write1ByteTxRx(portHandler, kMotorIdLeft, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, kMotorIdRight, ADDR_TORQUE_ENABLE, 0, &dxl_error);

    portHandler->closePort();
    return 0;
}