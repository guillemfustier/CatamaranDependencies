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
#define ADDR_OPERATING_MODE 11 // 4 for extended position control
#define ADDR_TORQUE_ENABLE 64  // 0 for torque off | 1 for torque on
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define VELOCITY_UNIT 0.229
#define TICKS_PER_TURN 4096.0

// Cable system parameters
constexpr double kPi = 3.14159265358979323846;
constexpr double kGearRatioMotorToSpool = 5.375;  // motor rotations per one full spool rotation
constexpr double kSpoolDiameterMeters = 0.30;
constexpr double kSpoolCircumferenceMeters = kPi * kSpoolDiameterMeters;
constexpr double kCablePerMotorRotationMeters = kSpoolCircumferenceMeters / kGearRatioMotorToSpool;
constexpr double kMaxCableDistanceMeters = 190.0;
constexpr double kInitialCableDistanceMeters = 1.0;
constexpr double kMaxMotorRpm = 30.0;
constexpr int32_t kProfileVelocityLimit = 32767;

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
            reference_cable_distance_m_(kInitialCableDistanceMeters),
            reference_motor_position_raw_(0),
            reference_initialized_(false),
            profile_velocity_raw_(0) {

        RCLCPP_INFO(this->get_logger(), "Cable Distance Controller node started");

        this->declare_parameter("qos_depth", 10);
        this->declare_parameter("device_name", std::string(DEFAULT_DEVICE_NAME));
        this->declare_parameter("goal_topic", std::string("/cable_distance"));
        this->declare_parameter("actual_topic", std::string("/cable_distance_actual"));
        this->declare_parameter("initial_cable_distance", kInitialCableDistanceMeters);
        this->declare_parameter("max_cable_distance", kMaxCableDistanceMeters);
        this->declare_parameter("max_motor_rpm", kMaxMotorRpm);

        int8_t qos_depth = 10;
        this->get_parameter("qos_depth", qos_depth);
        this->get_parameter("initial_cable_distance", current_cable_distance_m_);
        this->get_parameter("max_cable_distance", max_cable_distance_m_);
        reference_cable_distance_m_ = current_cable_distance_m_;

        double max_motor_rpm = kMaxMotorRpm;
        this->get_parameter("max_motor_rpm", max_motor_rpm);
        profile_velocity_raw_ = rpmToProfileVelocityRaw(max_motor_rpm);

        std::string goal_topic;
        std::string actual_topic;
        this->get_parameter("goal_topic", goal_topic);
        this->get_parameter("actual_topic", actual_topic);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort().durability_volatile();

        cable_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>(actual_topic, qos);
        cable_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            goal_topic,
            qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->handleCableDistanceCommand(msg->data);
            });

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                this->publishActualFromMotor();
            });
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
        sendGoalPosition();
    }

    bool initializeReferenceFromMotor() {
        int32_t present_position_raw = 0;
        if (!readPresentPositionRaw(present_position_raw)) {
            return false;
        }

        reference_motor_position_raw_ = present_position_raw;
        reference_initialized_ = true;
        publishCurrentDistance(current_cable_distance_m_);
        return true;
    }

private:
    int32_t rpmToProfileVelocityRaw(double rpm) const {
        const int32_t raw = static_cast<int32_t>(std::round(rpm / VELOCITY_UNIT));
        return std::clamp(raw, 1, kProfileVelocityLimit);
    }

    bool readPresentPositionRaw(int32_t &position_raw) {
        uint8_t local_error = 0;
        uint32_t raw = 0;
        const int comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_PRESENT_POSITION,
            &raw,
            &local_error);

        if (comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorId, packetHandler->getTxRxResult(comm_result));
            return false;
        }

        if (local_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorId, packetHandler->getRxPacketError(local_error));
            return false;
        }

        position_raw = static_cast<int32_t>(raw);
        return true;
    }

    double rawToCableDistanceMeters(int32_t position_raw) const {
        if (!reference_initialized_) {
            return current_cable_distance_m_;
        }

        const double motor_turns =
            static_cast<double>(position_raw - reference_motor_position_raw_) / TICKS_PER_TURN;
        return reference_cable_distance_m_ + motor_turns * kCablePerMotorRotationMeters;
    }

    bool writeProfileVelocity(int32_t velocity_raw) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_PROFILE_VELOCITY,
            static_cast<uint32_t>(velocity_raw),
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    bool writeGoalPosition(int32_t goal_position_raw) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_GOAL_POSITION,
            static_cast<uint32_t>(goal_position_raw),
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    void sendGoalPosition() {
        int32_t present_position_raw = 0;
        if (!readPresentPositionRaw(present_position_raw)) {
            return;
        }

        if (!reference_initialized_) {
            reference_motor_position_raw_ = present_position_raw;
            reference_initialized_ = true;
        }

        const double present_distance_m = rawToCableDistanceMeters(present_position_raw);
        const double delta_distance_m = target_cable_distance_m_ - present_distance_m;

        if (std::abs(delta_distance_m) < 1e-6) {
            publishCurrentDistance(present_distance_m);
            return;
        }

        const double motor_turn_delta = delta_distance_m / kCablePerMotorRotationMeters;
        const int32_t position_delta_raw =
            static_cast<int32_t>(std::llround(motor_turn_delta * TICKS_PER_TURN));
        const int32_t goal_position_raw = present_position_raw + position_delta_raw;

        if (!writeProfileVelocity(profile_velocity_raw_)) {
            return;
        }

        if (!writeGoalPosition(goal_position_raw)) {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: %.3f m | Current: %.3f m | Delta: %.3f m | Delta turns: %.3f | Goal raw: %d",
            target_cable_distance_m_,
            present_distance_m,
            delta_distance_m,
            motor_turn_delta,
            goal_position_raw);

        current_cable_distance_m_ = target_cable_distance_m_;
    }

    void publishActualFromMotor() {
        int32_t present_position_raw = 0;
        if (!readPresentPositionRaw(present_position_raw)) {
            return;
        }

        if (!reference_initialized_) {
            reference_motor_position_raw_ = present_position_raw;
            reference_initialized_ = true;
        }

        publishCurrentDistance(rawToCableDistanceMeters(present_position_raw));
    }

    void publishCurrentDistance(double distance_m) {
        const double clamped_distance_m = std::clamp(distance_m, 0.0, max_cable_distance_m_);
        current_cable_distance_m_ = clamped_distance_m;
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(clamped_distance_m);
        cable_distance_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published current cable distance: %.3f m", clamped_distance_m);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cable_distance_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cable_distance_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double current_cable_distance_m_;
    double max_cable_distance_m_;
    double target_cable_distance_m_;
    double reference_cable_distance_m_;
    int32_t reference_motor_position_raw_;
    bool reference_initialized_;
    int32_t profile_velocity_raw_;
};

bool setupDynamixel(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to disable torque for ID %d.", dxl_id);
        return false;
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        4,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to set Extended Position mode for ID %d.", dxl_id);
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("cable_distance_controller"), "Extended Position mode set for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("cable_distance_controller"), "Failed to enable torque for ID %d.", dxl_id);
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("cable_distance_controller"), "Torque enabled for ID %d.", dxl_id);
    }

    return true;
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

    if (!setupDynamixel(kMotorId)) {
        packetHandler->write1ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_TORQUE_ENABLE,
            0,
            &dxl_error);
        portHandler->closePort();
        rclcpp::shutdown();
        return -1;
    }

    if (!cable_controller->initializeReferenceFromMotor()) {
        packetHandler->write1ByteTxRx(
            portHandler,
            kMotorId,
            ADDR_TORQUE_ENABLE,
            0,
            &dxl_error);
        portHandler->closePort();
        rclcpp::shutdown();
        return -1;
    }

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