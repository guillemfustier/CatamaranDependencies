    ///////////////////////////////////////////////////////////////////////////////
    //                     *** DYNAMIXEL MOTOR ROS2 NODE ***
    //
    //
    //                         * Enable USB port access *
    // >> sudo usermod -aG dialout <linux_account>
    //                           (Restart your computer)
    //
    //                               * Start node *
    // >> ros2 run rover_bringup motor_vel_controller
    //
    //
    //             * Send SetVelocity messages to /set_velocity topic *
    // 1 unit = 0.229 rpm
    // >> ros2 topic pub -1 /set_velocity custom_interfaces/msg/SetVelocity "{id: 1, velocity: 500}"
    //
    //
    // if it doesnt work, try:
    // >> ros2 run rover_bringup motor_vel_controller <device_name>
    //         - Optional argument (default: /dev/ttyUSB0)
    //         - Use ls /dev/ttyUSB* to find the correct device name

    //         - Use sudo chmod 777 /dev/ttyUSB0 to give permissions
    //
    //             * Send Twist messages to /cmd_vel topic *
    // >> ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
    // 
    //            * Send SetPosition messages to /tool_pos topic *
    // >> ros2 topic pub -1 /tool_pos custom_interfaces/msg/SetPosition "{position: 90, id: 3}"
    ///////////////////////////////////////////////////////////////////////////////

#include <cstdio> // Dynamixel SDK
#include <memory> // Dynamixel SDK
#include <string> // Dynamixel SDK

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "custom_interfaces/srv/get_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h" // Dynamixel SDK
#include "std_msgs/msg/float32.hpp"


#include "rudder_angle_controller.hpp"

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64 // 0 for torque off | 1 for torque on
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600 // TODO: Es esto verdad?
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0" // ls /dev/ttyUSB* to find the correct device name
#define VELOCITY_UNIT 0.088 // TODO: Poner bien -> rpm | See https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/#velocity-limit for more details

// Robot parameters
#define DIAMETER_ENGRANAJE 0.04 // metros
#define LONGITUD_ALETA 0.47 // metros
#define RPM_MAXIMO 30.0 // velocidad máxima del motor

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Variables for motor control
#define ID_HERRAMIENTA 1 // ID del motor del actuador

// Error handling
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;
int angulo_grados_iniciales = 0;


MotorController::MotorController()
: Node("motor_angle_controller") {

    RCLCPP_INFO(this->get_logger(), "Motor Controller node started");
    
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = // Defines QoS
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // ╔═════════════════════════════╗
    // ║   ANGLE TO MOTOR_MOVEMENT   ║
    // ╚═════════════════════════════╝
    
    rudder_angle_subscriber_ =
    this->create_subscription<std_msgs::msg::Int32>(
        "rudder_angle",
        QOS_RKL10V,
        [this](const std_msgs::msg::Int32::SharedPtr msg) -> void {
            int angulo_grados = msg->data;
            angulo_grados = angulo_grados - angulo_grados_iniciales;
            angulo_grados_iniciales = angulo_grados;

            // Revisar
                double alpha = angulo_grados * M_PI / 180.0;
                if (angulo_grados > 40){
                    RCLCPP_INFO(this->get_logger(), "Se ha superado el valor límite del angulo");
                    angulo_grados = 40;
                } else if  (angulo_grados < -40){
                    RCLCPP_INFO(this->get_logger(), "Se ha superado el valor límite del angulo");
                    angulo_grados = -40;
                }
                double alpha_abs = std::abs(alpha);

                // Constantes del sistema
                const double l = 0.37;
                const double R = 0.3;
                const double x_c = 0.0;    // En teoría es 0.0135
                const double y_c = -0.385;

                // Cálculos auxiliares
                double y = y_c + cos(alpha_abs) * R;
                double x = x_c + sin(alpha_abs) * R;

                // Dos soluciones para d
                double d = sqrt(l * l - y * y);

                double d_final = abs(d - 0.360) + x; // Posición neutra a 0.235 m + 0.125 m del motor descentrado
                d_final = alpha  < 0 ? -d_final : d_final;

                // Cálculo de vueltas y tiempo
                double circunferencia = M_PI * DIAMETER_ENGRANAJE;
                double vueltas = d_final / circunferencia;
                
                //Modificado a partir de aquí
                // Ej queremos pasar de -15 a -20 --> -20 - (-15) = -5
                // Ej queremos pasar de -15 a 20 --> 20 - (-15) = 35
                // Ej queremos pasar de 20 a -15 --> -15 - 20 = -35
                // Ej queremos pasar de 15 a 20 --> 20 - (15) = 5
                
                
                
                double tiempo = std::abs(vueltas / RPM_MAXIMO) * 60.0; // tiempo siempre positivo

                RCLCPP_INFO(this->get_logger(), "Ángulo: %d grados, radianes: %.3f, x: %.3f, y: %.3f, d_final: %.3f m, tiempo: %.3f s", angulo_grados, alpha, x, y, d_final, tiempo);

                // Enviar velocidad máxima al motor (ejemplo para un motor, id_herramienta)
                int32_t velocidad_maxima = static_cast<int32_t>(RPM_MAXIMO / VELOCITY_UNIT);
                int32_t limite = 1023; // Para XW540-T260
                if (velocidad_maxima > limite) velocidad_maxima = limite;
                if (velocidad_maxima < -limite) velocidad_maxima = -limite;
                if (vueltas < 0) {
                    velocidad_maxima = -velocidad_maxima; // invierte el sentido
                }
                dxl_comm_result = packetHandler->write4ByteTxRx(
                    portHandler,
                    ID_HERRAMIENTA,
                    ADDR_GOAL_POSITION,
                    velocidad_maxima,
                    &dxl_error
                );
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
                } else if (dxl_error != 0) {
                    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Motor [ID: %d] girando a velocidad máxima (%d unidades)", ID_HERRAMIENTA, velocidad_maxima);
                }

                // Esperar el tiempo calculado
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(tiempo)));
                // Parar el motor
                dxl_comm_result = packetHandler->write4ByteTxRx(
                    portHandler,
                    ID_HERRAMIENTA,
                    ADDR_GOAL_POSITION,
                    0,
                    &dxl_error
                );
                if (dxl_comm_result != COMM_SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
                } else if (dxl_error != 0) {
                    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Motor [ID: %d] parado", ID_HERRAMIENTA);
                }
            }
        );
    // // ╔═════════════════════════════╗
    // // ║  GET CURRENT VELOCITY       ║
    // // ╚═════════════════════════════╝

    // // Defines a service to get the motor's current velocity
    // auto get_current_velocity =
    //     [this](
    //     const std::shared_ptr<GetVelocity::Request> request,
    //     std::shared_ptr<GetVelocity::Response> response) -> void {
            
    //         // Read current velocity (4 bytes)
    //         dxl_comm_result = packetHandler->read4ByteTxRx(
    //             portHandler, 
    //             (uint8_t) request->id, 
    //             ADDR_PRESENT_VELOCITY, 
    //             reinterpret_cast<uint32_t*>(&current_velocity),
    //             &dxl_error
    //         );

    //         RCLCPP_INFO(
    //             this->get_logger(),
    //             "Get [ID: %d] [Current Velocity: %d]",
    //             request->id,
    //             current_velocity
    //         );

    //         response->velocity = current_velocity;
    //     };
    // get_velocity_server_ = create_service<GetVelocity>("get_velocity", get_current_velocity);

    // // ╔═════════════════════════════╗
    // // ║  GET CURRENT POSITION       ║
    // // ╚═════════════════════════════╝

    // auto get_present_position =
    //     [this](
    //     const std::shared_ptr<GetPosition::Request> request,
    //     std::shared_ptr<GetPosition::Response> response) -> void
    //     {
    //     // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    //     // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    //     dxl_comm_result = packetHandler->read4ByteTxRx(
    //         portHandler,
    //         (uint8_t) request->id,
    //         ADDR_PRESENT_POSITION,
    //         reinterpret_cast<uint32_t *>(&present_position),
    //         &dxl_error
    //     );

    //     RCLCPP_INFO(
    //         this->get_logger(),
    //         "Get [ID: %d] [Present Position: %d]",
    //         request->id,
    //         present_position
    //     );

    //     response->position = present_position;
    //     };

    // get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

int32_t MotorController::getCurrentPosition(uint8_t id) {
    int32_t present_position = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
    );
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    return present_position;
}

int32_t MotorController::getCurrentVelocity(uint8_t id) {
    int32_t present_velocity = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        id,
        ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t *>(&present_velocity),
        &dxl_error
    );
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    return present_velocity;
}

MotorController::~MotorController() { RCLCPP_INFO(this->get_logger(), "Motor Controller node stopped"); }

void setupDynamixel(uint8_t dxl_id) {

    // ----- SET VELOCITY MODE TO ALL MOTORS
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, 
        dxl_id, 
        ADDR_OPERATING_MODE, 
        4, 
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_angle_controller"), "Failed to set Velocity Control mode for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_angle_controller"), "Succeeded to set Velocity Control mode for ID %d.", dxl_id);
    }

    // -------- Enable Torque so the motor can move (EEPROM will be locked)
    // IMPORTANT: Torque must be disabled to change the operating mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, 
        dxl_id, 
        ADDR_TORQUE_ENABLE, 
        1, 
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_angle_controller"), "Failed to enable Torque for ID %d.", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_angle_controller"), "Succeeded to enable Torque for ID %d.", dxl_id);
    }
    }

int main(int argc, char * argv[]) {
    
    // Get custom device name
    const char* deviceName = DEFAULT_DEVICE_NAME;
    if (argc > 1) {
        deviceName = argv[1]; 
    }
    std::cout << "Using device: " << deviceName << std::endl;

    portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motorr_angle_controller"), "Failed to open the port!");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motorr_angle_controller"), "Succeeded to open the port.");
    }
    
    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motorr_angle_controller"), "Failed to set the baudrate!");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_vel_controller"), "Succeeded to set the baudrate.");
    }
    
    // Initialize Motors with the correct operating mode for each one,
    // and enable Torque for all of them
    // uint8_t wheel_ids[4] = {LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID};
    // for(auto id : wheel_ids){
    //     setupDynamixel(id);
    // }
    setupDynamixel(ID_HERRAMIENTA);
    // Keep the node running until closed
    rclcpp::init(argc, argv);
    auto motorcontroller = std::make_shared<MotorController>();
    rclcpp::spin(motorcontroller);
    
    // On shutdown, disable Torque of DYNAMIXEL
    rclcpp::shutdown();

    packetHandler->write1ByteTxRx(
        portHandler,
        ID_HERRAMIENTA,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );

    // // Disable Torque of all wheels
    //     for(auto id : wheel_ids){
    //         packetHandler->write1ByteTxRx(
    //             portHandler,
    //             id,
    //             ADDR_TORQUE_ENABLE,
    //             0,
    //             &dxl_error
    //         );
    //     }

    return 0;
}