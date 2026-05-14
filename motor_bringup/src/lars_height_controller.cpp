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
#include <glob.h>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/float32.hpp"
#include <bitset>

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
// Hardware error status (read-only) for Protocol 2.0 X series
#define ADDR_HARDWARE_ERROR_STATUS 70

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB1"
#define VELOCITY_UNIT 0.229
#define TICKS_PER_TURN 4096.0

// LARS system parameters
constexpr double kInitialHeightCm = 24.0;
constexpr double kMaxHeightCm = 35.0;
constexpr double kMinHeightCm = 0.0;
constexpr double kMotorTurnsPerCm = 5.0;
constexpr double kMaxMotorRpm = 60.0; // default, can be overridden via parameter
constexpr int32_t kProfileVelocityLimit = 32767;
constexpr int kMaxConnectionRounds = 5;
constexpr auto kConnectionRetryDelay = std::chrono::seconds(1);

// Two synchronized motors for the linear axes
constexpr uint8_t kMotorIdLeft = 3;
constexpr uint8_t kMotorIdRight = 4;

dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;

int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

bool setupDynamixel(uint8_t dxl_id);

void addCandidate(std::vector<std::string> &candidates, const std::string &device_name) {
    if (device_name.empty()) {
        return;
    }

    if (std::find(candidates.begin(), candidates.end(), device_name) == candidates.end()) {
        candidates.push_back(device_name);
    }
}

void addGlobCandidates(std::vector<std::string> &candidates, const std::string &pattern) {
    glob_t glob_result;
    if (glob(pattern.c_str(), 0, nullptr, &glob_result) != 0) {
        globfree(&glob_result);
        return;
    }

    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        addCandidate(candidates, glob_result.gl_pathv[i]);
    }

    globfree(&glob_result);
}

std::vector<std::string> buildDeviceCandidates(const std::string &preferred_device_name) {
    std::vector<std::string> candidates;
    addCandidate(candidates, preferred_device_name.empty() ? DEFAULT_DEVICE_NAME : preferred_device_name);
    addGlobCandidates(candidates, "/dev/ttyUSB*");
    addGlobCandidates(candidates, "/dev/ttyACM*");
    addGlobCandidates(candidates, "/dev/serial/by-id/*");
    return candidates;
}

class LarsHeightController : public rclcpp::Node {
public:
    LarsHeightController()
    : Node("lars_height_controller"),
      current_height_cm_(kInitialHeightCm),
            max_height_cm_(kMaxHeightCm),
            max_motor_rpm_(kMaxMotorRpm),
            target_height_cm_(kInitialHeightCm),
            reference_height_cm_(kInitialHeightCm),
            reference_left_position_raw_(0),
            reference_right_position_raw_(0),
            reference_initialized_(false),
            profile_velocity_raw_(0),
            device_name_(DEFAULT_DEVICE_NAME),
            port_connected_(false),
            connection_permanently_failed_(false) {

        RCLCPP_INFO(this->get_logger(), "LARS Height Controller node started");

        this->declare_parameter("qos_depth", 10);
        this->declare_parameter("device_name", std::string(DEFAULT_DEVICE_NAME));
        this->declare_parameter("goal_topic", std::string("/lars_height"));
        this->declare_parameter("actual_topic", std::string("/lars_height_actual"));
        this->declare_parameter("initial_height", kInitialHeightCm);
        this->declare_parameter("max_height", kMaxHeightCm);
        this->declare_parameter("max_motor_rpm", kMaxMotorRpm);

        this->get_parameter("device_name", device_name_);
        device_candidates_ = buildDeviceCandidates(device_name_);

        int8_t qos_depth = 10;
        this->get_parameter("qos_depth", qos_depth);
        this->get_parameter("initial_height", current_height_cm_);
        this->get_parameter("max_height", max_height_cm_);
        this->get_parameter("max_motor_rpm", max_motor_rpm_);
        reference_height_cm_ = current_height_cm_;
        profile_velocity_raw_ = rpmToProfileVelocityRaw(max_motor_rpm_);

        std::string goal_topic;
        std::string actual_topic;
        this->get_parameter("goal_topic", goal_topic);
        this->get_parameter("actual_topic", actual_topic);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort().durability_volatile();

        actual_publisher_ = this->create_publisher<std_msgs::msg::Float32>(actual_topic, qos);
        goal_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            goal_topic,
            qos,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->handleGoal(msg->data);
            });

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                this->publishActualFromMotors();
            });

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
                        profile_velocity_raw_ = rpmToProfileVelocityRaw(max_motor_rpm_);
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
        if (!ensureConnection()) {
            RCLCPP_ERROR(this->get_logger(), "No Dynamixel connection available; goal ignored.");
            return;
        }
        sendGoalPositions();
    }

    bool initializeReferenceFromMotors() {
        int32_t left_present = 0;
        int32_t right_present = 0;
        if (!readPresentPositionRaw(kMotorIdLeft, left_present)) {
            return false;
        }
        if (!readPresentPositionRaw(kMotorIdRight, right_present)) {
            return false;
        }

        reference_left_position_raw_ = left_present;
        reference_right_position_raw_ = right_present;
        reference_height_cm_ = current_height_cm_;
        reference_initialized_ = true;
        publishCurrentHeight(current_height_cm_);
        return true;
    }

    bool connectWithFallback() {
        disconnectPort();
        connection_permanently_failed_ = false;

        for (int round = 1; round <= kMaxConnectionRounds; ++round) {
            for (const auto &candidate : device_candidates_) {
                RCLCPP_INFO(this->get_logger(), "Attempting to open Dynamixel port: %s (round %d/%d)", candidate.c_str(), round, kMaxConnectionRounds);

                if (openAndConfigurePort(candidate)) {
                    connected_device_name_ = candidate;
                    port_connected_ = true;
                    RCLCPP_INFO(this->get_logger(), "Using Dynamixel port: %s", candidate.c_str());
                    return true;
                }
            }

            RCLCPP_WARN(this->get_logger(), "No usable Dynamixel port found on round %d/%d.", round, kMaxConnectionRounds);

            if (round < kMaxConnectionRounds) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Waiting %ld second(s) before the next reconnection round.",
                    static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(kConnectionRetryDelay).count()));
                std::this_thread::sleep_for(kConnectionRetryDelay);
            }
        }

        connection_permanently_failed_ = true;
        RCLCPP_FATAL(this->get_logger(), "Failed to find a usable Dynamixel port after %d full scan rounds.", kMaxConnectionRounds);
        return false;
    }

    bool ensureConnection() {
        if (connection_permanently_failed_) {
            return false;
        }

        if (port_connected_) {
            return true;
        }

        return connectWithFallback();
    }

    bool recoverConnection(const std::string &context) {
        if (connection_permanently_failed_) {
            return false;
        }

        RCLCPP_WARN(this->get_logger(), "Connection lost during %s. Trying USB fallback.", context.c_str());
        port_connected_ = false;
        disconnectPort();

        if (connectWithFallback()) {
            RCLCPP_INFO(this->get_logger(), "Recovered Dynamixel connection on %s.", connected_device_name_.c_str());
            return true;
        }

        rclcpp::shutdown();
        return false;
    }

    void disconnectPort() {
        if (packetHandler != nullptr && portHandler != nullptr) {
            packetHandler->write1ByteTxRx(portHandler, kMotorIdLeft, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            packetHandler->write1ByteTxRx(portHandler, kMotorIdRight, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            portHandler->closePort();
        }

        port_connected_ = false;
    }

private:
    int32_t rpmToProfileVelocityRaw(double rpm) const {
        const int32_t raw = static_cast<int32_t>(std::round(rpm / VELOCITY_UNIT));
        return std::clamp(raw, 1, kProfileVelocityLimit);
    }

    bool openAndConfigurePort(const std::string &device_name) {
        portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            RCLCPP_WARN(this->get_logger(), "Could not open Dynamixel port: %s", device_name.c_str());
            return false;
        }

        port_connected_ = true;

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_WARN(this->get_logger(), "Could not set baudrate on Dynamixel port: %s", device_name.c_str());
            portHandler->closePort();
            port_connected_ = false;
            return false;
        }

        if (!setupDynamixel(kMotorIdLeft) || !setupDynamixel(kMotorIdRight)) {
            disconnectPort();
            return false;
        }

        if (!initializeReferenceFromMotors()) {
            disconnectPort();
            return false;
        }

        return true;
    }

    bool readPresentPositionRaw(uint8_t motor_id, int32_t &position_raw) {
        uint8_t local_error = 0;
        uint32_t raw = 0;
        const int comm_result = packetHandler->read4ByteTxRx(
            portHandler,
            motor_id,
            ADDR_PRESENT_POSITION,
            &raw,
            &local_error);

        if (comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", motor_id, packetHandler->getTxRxResult(comm_result));
            // Try to read hardware error status for extra diagnostics
            uint8_t hw_err_val = 0;
            int hw_comm = packetHandler->read1ByteTxRx(
                portHandler,
                motor_id,
                ADDR_HARDWARE_ERROR_STATUS,
                &hw_err_val,
                &dxl_error);

            if (hw_comm == COMM_SUCCESS) {
                std::bitset<8> bits(hw_err_val);
                RCLCPP_ERROR(this->get_logger(), "Motor %d: Hardware error status = 0x%02X (%s)", motor_id, hw_err_val, bits.to_string().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motor %d: Could not read hardware error status: %s", motor_id, packetHandler->getTxRxResult(hw_comm));
            }

            return false;
        }

        if (local_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", motor_id, packetHandler->getRxPacketError(local_error));

            uint8_t hw_err_val = 0;
            int hw_comm = packetHandler->read1ByteTxRx(
                portHandler,
                motor_id,
                ADDR_HARDWARE_ERROR_STATUS,
                &hw_err_val,
                &dxl_error);

            if (hw_comm == COMM_SUCCESS) {
                std::bitset<8> bits(hw_err_val);
                RCLCPP_ERROR(this->get_logger(), "Motor %d: Hardware error status = 0x%02X (%s)", motor_id, hw_err_val, bits.to_string().c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motor %d: Could not read hardware error status: %s", motor_id, packetHandler->getTxRxResult(hw_comm));
            }

            return false;
        }

        position_raw = static_cast<int32_t>(raw);
        return true;
    }

    double rawToHeightCm(int32_t position_raw, int32_t reference_position_raw) const {
        if (!reference_initialized_) {
            return current_height_cm_;
        }

        const double motor_turns = static_cast<double>(position_raw - reference_position_raw) / TICKS_PER_TURN;
        return reference_height_cm_ + motor_turns / kMotorTurnsPerCm;
    }

    bool writeProfileVelocityBoth(int32_t velocity_raw) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorIdLeft,
            ADDR_PROFILE_VELOCITY,
            static_cast<uint32_t>(velocity_raw),
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
            ADDR_PROFILE_VELOCITY,
            static_cast<uint32_t>(velocity_raw),
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    bool writeGoalPositionBoth(int32_t left_goal_raw, int32_t right_goal_raw) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            kMotorIdLeft,
            ADDR_GOAL_POSITION,
            static_cast<uint32_t>(left_goal_raw),
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
            ADDR_GOAL_POSITION,
            static_cast<uint32_t>(right_goal_raw),
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "Motor %d: %s", kMotorIdRight, packetHandler->getRxPacketError(dxl_error));
            return false;
        }

        return true;
    }

    void sendGoalPositions() {
        int32_t left_present = 0;
        int32_t right_present = 0;
        if (!readPresentPositionRaw(kMotorIdLeft, left_present)) {
            return;
        }
        if (!readPresentPositionRaw(kMotorIdRight, right_present)) {
            return;
        }

        if (!reference_initialized_) {
            reference_left_position_raw_ = left_present;
            reference_right_position_raw_ = right_present;
            reference_initialized_ = true;
        }

        const double left_height_cm = rawToHeightCm(left_present, reference_left_position_raw_);
        const double right_height_cm = rawToHeightCm(right_present, reference_right_position_raw_);
        const double present_height_cm = (left_height_cm + right_height_cm) * 0.5;

        const double delta_cm = target_height_cm_ - present_height_cm;
        if (std::abs(delta_cm) < 1e-6) {
            publishCurrentHeight(present_height_cm);
            return;
        }

        const double motor_turn_delta = delta_cm * kMotorTurnsPerCm;
        const int32_t delta_raw = static_cast<int32_t>(std::llround(motor_turn_delta * TICKS_PER_TURN));

        const int32_t left_goal_raw = left_present + delta_raw;
        const int32_t right_goal_raw = right_present + delta_raw;

        if (!writeProfileVelocityBoth(profile_velocity_raw_)) {
            recoverConnection("profile velocity write");
            return;
        }

        if (!writeGoalPositionBoth(left_goal_raw, right_goal_raw)) {
            recoverConnection("goal position write");
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: %.3f cm | Current: %.3f cm | Delta: %.3f cm | Delta turns: %.3f | Left goal raw: %d | Right goal raw: %d",
            target_height_cm_,
            present_height_cm,
            delta_cm,
            motor_turn_delta,
            left_goal_raw,
            right_goal_raw);

        current_height_cm_ = target_height_cm_;
    }

    void publishActualFromMotors() {
        if (!ensureConnection()) {
            return;
        }

        int32_t left_present = 0;
        int32_t right_present = 0;
        if (!readPresentPositionRaw(kMotorIdLeft, left_present)) {
            recoverConnection("left axis read");
            return;
        }
        if (!readPresentPositionRaw(kMotorIdRight, right_present)) {
            recoverConnection("right axis read");
            return;
        }

        if (!reference_initialized_) {
            reference_left_position_raw_ = left_present;
            reference_right_position_raw_ = right_present;
            reference_initialized_ = true;
        }

        const double left_height_cm = rawToHeightCm(left_present, reference_left_position_raw_);
        const double right_height_cm = rawToHeightCm(right_present, reference_right_position_raw_);
        const double average_height_cm = (left_height_cm + right_height_cm) * 0.5;

        const double disagreement_cm = std::abs(left_height_cm - right_height_cm);
        if (disagreement_cm > 0.5) {
            RCLCPP_WARN(
                this->get_logger(),
                "LARS axes mismatch detected: left %.3f cm vs right %.3f cm (delta %.3f cm)",
                left_height_cm,
                right_height_cm,
                disagreement_cm);
        }

        publishCurrentHeight(average_height_cm);
    }

    void publishCurrentHeight(double height_cm) {
        const double clamped_height_cm = std::clamp(height_cm, kMinHeightCm, max_height_cm_);
        current_height_cm_ = clamped_height_cm;
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(clamped_height_cm);
        actual_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published current LARS height: %.3f cm", clamped_height_cm);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double current_height_cm_;
    double max_height_cm_;
    double max_motor_rpm_;
    double target_height_cm_;
    double reference_height_cm_;
    int32_t reference_left_position_raw_;
    int32_t reference_right_position_raw_;
    bool reference_initialized_;
    int32_t profile_velocity_raw_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::string device_name_;
    std::vector<std::string> device_candidates_;
    std::string connected_device_name_;
    bool port_connected_;
    bool connection_permanently_failed_;
};

bool setupDynamixel(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to disable torque for ID %d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));

        // Try to read hardware error status for better diagnostics
        uint8_t hw_err_val = 0;
        int hw_comm = packetHandler->read1ByteTxRx(
            portHandler,
            dxl_id,
            ADDR_HARDWARE_ERROR_STATUS,
            &hw_err_val,
            &dxl_error);

        if (hw_comm == COMM_SUCCESS) {
            std::bitset<8> bits(hw_err_val);
            RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Motor %d: Hardware error status = 0x%02X (%s)", dxl_id, hw_err_val, bits.to_string().c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Motor %d: Could not read hardware error status: %s", dxl_id, packetHandler->getTxRxResult(hw_comm));
        }

        return false;
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        4,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to set Extended Position mode for ID %d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("lars_height_controller"), "Extended Position mode set for ID %d.", dxl_id);
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("lars_height_controller"), "Failed to enable torque for ID %d: %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("lars_height_controller"), "Torque enabled for ID %d.", dxl_id);
    }

    return true;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto lars_controller = std::make_shared<LarsHeightController>();

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!lars_controller->connectWithFallback()) {
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(lars_controller);

    rclcpp::shutdown();
    lars_controller->disconnectPort();
    return 0;
}