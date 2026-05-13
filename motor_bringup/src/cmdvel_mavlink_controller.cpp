#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>

// MAVLink IDs y CRC Extras (Standard MAVLink 1.0)
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_CRC_EXTRA_HEARTBEAT 50

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#define MAVLINK_CRC_EXTRA_RC_CHANNELS_OVERRIDE 124

#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_CRC_EXTRA_COMMAND_LONG 152

#define MAV_CMD_COMPONENT_ARM_DISARM 400

// Estructuras empaquetadas para MAVLink v1
struct MAVLinkHeartbeat {
    uint32_t custom_mode;
    uint8_t type;           // 6 = GCS
    uint8_t autopilot;      // 8 = Invalid/Generic
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version; // 3
} __attribute__((__packed__));

struct MAVLinkRCChannelsOverride {
    uint16_t chan1_raw; uint16_t chan2_raw; uint16_t chan3_raw; uint16_t chan4_raw;
    uint16_t chan5_raw; uint16_t chan6_raw; uint16_t chan7_raw; uint16_t chan8_raw;
    uint8_t target_system;
    uint8_t target_component;
} __attribute__((__packed__));

struct MAVLinkCommandLong {
    float param1; float param2; float param3; float param4;
    float param5; float param6; float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
} __attribute__((__packed__));

class CmdVelMavlinkController : public rclcpp::Node {
public:
    CmdVelMavlinkController() : Node("cmd_vel_mavlink_controller"), sequence_(0) {
        this->declare_parameter("udp_target_ip", "127.0.0.1");
        this->declare_parameter("udp_target_port", 14550);
        this->declare_parameter("system_id", 255); // ID de la GCS (nosotros)
        this->declare_parameter("target_system_id", 1);

        udp_ip_ = this->get_parameter("udp_target_ip").as_string();
        udp_port_ = this->get_parameter("udp_target_port").as_int();
        sys_id_ = static_cast<uint8_t>(this->get_parameter("system_id").as_int());
        target_sys_id_ = static_cast<uint8_t>(this->get_parameter("target_system_id").as_int());

        if (!initUDPSocket()) return;

        // Suscripciones y Publicaciones
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelMavlinkController::cmdVelCallback, this, std::placeholders::_1));
        rudder_pub_ = this->create_publisher<std_msgs::msg::Int32>("rudder_angle", 10);

        // Timer para Heartbeat (1Hz) - CRÍTICO para mantener la conexión
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CmdVelMavlinkController::sendHeartbeat, this));

        // Timer para re-intentar armado después de 3 segundos
        arm_timer_ = this->create_wall_timer(
            std::chrono::seconds(3), [this]() { this->armVehicle(); this->arm_timer_->cancel(); });

        RCLCPP_INFO(this->get_logger(), "Nodo iniciado enviando a %s:%d", udp_ip_.c_str(), udp_port_);
    }

    ~CmdVelMavlinkController() { if (udp_socket_ != -1) close(udp_socket_); }

private:
    void sendHeartbeat() {
        MAVLinkHeartbeat hb{};
        hb.type = 6;            // MAV_TYPE_GCS
        hb.autopilot = 8;       // MAV_AUTOPILOT_INVALID
        hb.system_status = 4;   // MAV_STATE_ACTIVE
        hb.mavlink_version = 3;
        sendMavlinkMessage(MAVLINK_MSG_ID_HEARTBEAT, &hb, sizeof(hb), MAVLINK_CRC_EXTRA_HEARTBEAT);
    }

    void armVehicle() {
        MAVLinkCommandLong cmd{};
        cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f; // 1 = Arm
        cmd.target_system = target_sys_id_;
        cmd.target_component = 1;
        sendMavlinkMessage(MAVLINK_MSG_ID_COMMAND_LONG, &cmd, sizeof(cmd), MAVLINK_CRC_EXTRA_COMMAND_LONG);
        RCLCPP_INFO(this->get_logger(), "Comando de armado enviado.");
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 1. Control de timón (Local ROS)
        int rudder_angle = std::clamp(static_cast<int>(msg->angular.z * 40.0), -35, 40);
        std_msgs::msg::Int32 r_msg; r_msg.data = rudder_angle;
        rudder_pub_->publish(r_msg);

        // 2. Mezcla Diferencial (Skid-Steer) para Motores
        // Si quieres que avance recto, ambos motores deben sumar linear.x
        double thrust = std::clamp(msg->linear.x, -1.0, 1.0);
        double turn = std::clamp(msg->angular.z, -1.0, 1.0);

        uint16_t pwm_L = static_cast<uint16_t>(1500 + (thrust + turn) * 400);
        uint16_t pwm_R = static_cast<uint16_t>(1500 + (thrust - turn) * 400);

        MAVLinkRCChannelsOverride rc{};
        rc.target_system = target_sys_id_;
        rc.target_component = 1;
        rc.chan1_raw = pwm_L; // Motor Izquierdo
        rc.chan3_raw = pwm_R; // Motor Derecho
        // Canales 2 y 4 a 0 significa "no sobreescribir" o mantener neutro
        rc.chan2_raw = 0; rc.chan4_raw = 0; 

        sendMavlinkMessage(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, &rc, sizeof(rc), MAVLINK_CRC_EXTRA_RC_CHANNELS_OVERRIDE);
    }

    void sendMavlinkMessage(uint8_t msg_id, void* payload, uint8_t len, uint8_t crc_extra) {
        uint8_t buf[263];
        buf[0] = 0xFE; // STX v1
        buf[1] = len;
        buf[2] = sequence_++;
        buf[3] = sys_id_;
        buf[4] = 1;    // Component ID de la GCS
        buf[5] = msg_id;
        std::memcpy(&buf[6], payload, len);

        uint16_t crc = 0xFFFF;
        for (int i = 1; i < len + 6; i++) accumulateCRC(buf[i], &crc);
        accumulateCRC(crc_extra, &crc);

        buf[len + 6] = (uint8_t)(crc & 0xFF);
        buf[len + 7] = (uint8_t)(crc >> 8);

        sendto(udp_socket_, buf, len + 8, 0, (struct sockaddr*)&target_addr_, sizeof(target_addr_));
    }

    void accumulateCRC(uint8_t data, uint16_t *crc) {
        uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
        tmp ^= (tmp << 4);
        *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    bool initUDPSocket() {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(udp_port_);
        return inet_pton(AF_INET, udp_ip_.c_str(), &target_addr_.sin_addr) > 0;
    }

    int udp_socket_;
    struct sockaddr_in target_addr_;
    uint8_t sequence_, sys_id_, target_sys_id_;
    std::string udp_ip_;
    int udp_port_;
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_, arm_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rudder_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelMavlinkController>());
    rclcpp::shutdown();
    return 0;
}