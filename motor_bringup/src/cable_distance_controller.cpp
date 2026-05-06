///////////////////////////////////////////////////////////////////////////////
//                 *** DYNAMIXEL CABLE DISTANCE CONTROLLER ***
//
// Subscribes to an absolute cable distance target in meters and drives the
// spool motor (ID 2) until the estimated cable length matches that target.
// The node also publishes the current estimated cable distance so other
// components can compare goal vs actual distance.
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
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64  // 0 for torque off | 1 for torque on
#define ADDR_GOAL_VELOCITY 104

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define VELOCITY_UNIT 0.229

// Cable system parameters
constexpr double kPi = 3.14159265358979323846;
constexpr double kGearRatioMotorToSpool = 5.375;  // motor rotations per one full spool rotation
constexpr double kSpoolDiameterMeters = 0.30;
constexpr double kSpoolCircumferenceMeters = kPi * kSpoolDiameterMeters;
constexpr double kCablePerMotorRotationMeters = kSpoolCircumferenceMeters / kGearRatioMotorToSpool;
constexpr double kMaxCableDistanceMeters = 190.0;
constexpr double kInitialCableDistanceMeters = 1.0;
constexpr double kMaxMotorRpm = 30.0;
constexpr int32_t kVelocityLimit = 1023;

// Dynamixel motor ID for the cable spool
constexpr uint8_t kMotorId = 2;

dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;

int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

class CableDistanceController : public rclcpp::Node {
public:
    CableDistanceController()
        : Node("cable_distance_controller"),
            current_cable_distance_m_(kInitialCableDistanceMeters),
            max_cable_distance_m_(kMaxCableDistanceMeters),
            target_cable_distance_m_(kInitialCableDistanceMeters),
            motion_active_(false),
            active_velocity_command_(0),
            motion_duration_s_(0.0) {

        RCLCPP_INFO(this->get_logger(), "Cable Distance Controller node started");

        this->declare_parameter("qos_depth", 10);
        this->declare_parameter("device_name", std::string(DEFAULT_DEVICE_NAME));
        this->declare_parameter("goal_topic", std::string("/cable_distance"));
        this->declare_parameter("actual_topic", std::string("/cable_distance_actual"));
        this->declare_parameter("initial_cable_distance", kInitialCableDistanceMeters);
        this->declare_parameter("max_cable_distance", kMaxCableDistanceMeters);

        int8_t qos_depth = 10;
        this->get_parameter("qos_depth", qos_depth);
        this->get_parameter("initial_cable_distance", current_cable_distance_m_);
        this->get_parameter("max_cable_distance", max_cable_distance_m_);

        std::string goal_topic;
        std::string actual_topic;
        this->get_parameter("goal_topic", goal_topic);
        this->get_parameter("actual_topic", actual_topic);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        cable_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>(actual_topic, qos);
        cable_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            goal_topic,
            qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->handleCableDistanceCommand(msg->data);
            });

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                this->processMotion();
            });

        publishCurrentDistance();
    }

    void handleCableDistanceCommand(double requested_distance_m) {
        const double goal_distance_m = std::clamp(requested_distance_m, 0.0, max_cable_distance_m_);

        if (goal_distance_m != requested_distance_m) {
            RCLCPP_WARN(
                this->get_logger(),
                "Requested cable distance %.3f m was clamped to %.3f m",
                requested_distance_m,
                goal_distance_m);
        }

        target_cable_distance_m_ = goal_distance_m;
        replanMotion();
    }

private:
    void processMotion() {
        if (!motion_active_) {
            return;
        }

        updateEstimatedDistance();

        if (!motion_active_ && active_velocity_command_ != 0) {
            stopMotor();
        }
    }

    void replanMotion() {
        updateEstimatedDistance();

        const double delta_distance_m = target_cable_distance_m_ - current_cable_distance_m_;
        if (std::abs(delta_distance_m) < 1e-6) {
            if (active_velocity_command_ != 0) {
                stopMotor();
            }
            publishCurrentDistance();
            return;
        }

        int32_t velocity_magnitude = static_cast<int32_t>(std::round(kMaxMotorRpm / VELOCITY_UNIT));
        velocity_magnitude = std::clamp(velocity_magnitude, 1, kVelocityLimit);
        const double effective_motor_rpm = static_cast<double>(velocity_magnitude) * VELOCITY_UNIT;

        const double motor_rotations = std::abs(delta_distance_m) / kCablePerMotorRotationMeters;
        double motion_time_s = (motor_rotations / effective_motor_rpm) * 60.0;
        motion_time_s = std::round(motion_time_s * 100.0) / 100.0;

        int32_t velocity_command = velocity_magnitude;

        const bool should_extend_cable = delta_distance_m > 0.0;
        if (!should_extend_cable) {
            velocity_command = -velocity_command;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: %.3f m | Current: %.3f m | Delta: %.3f m | Motor rotations: %.3f | Time: %.3f s",
            target_cable_distance_m_,
            current_cable_distance_m_,
            delta_distance_m,
            motor_rotations,
            motion_time_s);

        if (velocity_command != active_velocity_command_) {
            if (!writeGoalVelocity(velocity_command)) {
                return;
            }
            active_velocity_command_ = velocity_command;
        }

        motion_active_ = true;
        motion_start_time_ = this->now();
        motion_start_distance_m_ = current_cable_distance_m_;
        motion_goal_distance_m_ = target_cable_distance_m_;
        motion_duration_s_ = motion_time_s;
    }

    void updateEstimatedDistance() {
        if (!motion_active_) {
            return;
        }

        if (motion_duration_s_ <= 0.0) {
            current_cable_distance_m_ = motion_goal_distance_m_;
            motion_active_ = false;
            publishCurrentDistance();
            return;
        }

        const double elapsed_s = (this->now() - motion_start_time_).seconds();
        const double progress = std::clamp(elapsed_s / motion_duration_s_, 0.0, 1.0);

        current_cable_distance_m_ =
            motion_start_distance_m_ + (motion_goal_distance_m_ - motion_start_distance_m_) * progress;

        if (progress >= 1.0) {
            motion_active_ = false;
            publishCurrentDistance();
        }
    }

    void stopMotor() {
        if (active_velocity_command_ == 0) {
            return;
        }

        if (!writeGoalVelocity(0)) {
            return;
        }

        active_velocity_command_ = 0;
    }

    bool writeGoalVelocity(int32_t velocity) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_GOAL_VELOCITY,
            velocity,
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        if (velocity == 0) {
            RCLCPP_INFO(this->get_logger(), "Motor [ID: %d] stopped", kMotorId);
        } else {
            RCLCPP_INFO(this->get_logger(), "Motor [ID: %d] running at velocity %d", kMotorId, velocity);
        }

        return true;
    }

    void publishCurrentDistance() {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(current_cable_distance_m_);
        cable_distance_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published current cable distance: %.3f m", current_cable_distance_m_);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cable_distance_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cable_distance_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double current_cable_distance_m_;
    double max_cable_distance_m_;
    double target_cable_distance_m_;
    bool motion_active_;
    int32_t active_velocity_command_;
    rclcpp::Time motion_start_time_;
    double motion_start_distance_m_;
    double motion_goal_distance_m_;
    double motion_duration_s_;
};

void setupDynamixel(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to disable torque for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to set Velocity Control mode for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("cable_distance_controller"), "Velocity Control mode set for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to enable torque for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("cable_distance_controller"), "Torque enabled for ID %d.", dxl_id);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto cable_controller = std::make_shared<CableDistanceController>();

    std::string deviceNameString;
    if (!cable_controller->get_parameter("device_name", deviceNameString)) {
        deviceNameString = DEFAULT_DEVICE_NAME;
    }

    if (deviceNameString.empty()) {
        deviceNameString = DEFAULT_DEVICE_NAME;
    }

    const char * deviceName = deviceNameString.c_str();

    RCLCPP_INFO(cable_controller->get_logger(), "Attempting to open Dynamixel port: %s", deviceName);

    portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(cable_controller->get_logger(), "Failed to open the port!");
        return -1;
    }

    RCLCPP_INFO(cable_controller->get_logger(), "Succeeded to open the port.");

    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(cable_controller->get_logger(), "Failed to set the baudrate!");
        return -1;
    }

    RCLCPP_INFO(cable_controller->get_logger(), "Succeeded to set the baudrate.");

    setupDynamixel(kMotorId);

    rclcpp::spin(cable_controller);

    rclcpp::shutdown();

    packetHandler->write1ByteTxRx(
        portHandler,
        kMotorId,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    portHandler->closePort();

    return 0;
}