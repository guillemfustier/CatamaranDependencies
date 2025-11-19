// Copyright (c) 2025
#ifndef MOTORES_BRINGUP__MOTOR_VEL_CONTROLLER_HPP_
#define MOTORES_BRINGUP__MOTOR_VEL_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class MotorController : public rclcpp::Node {
public:
   MotorController();
   virtual ~MotorController();

private:
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rudder_angle_subscriber_;
   // Helper methods to read encoder values from Dynamixel
   int32_t getCurrentPosition(uint8_t id);
   int32_t getCurrentVelocity(uint8_t id);
   // Puedes añadir aquí otros miembros si los necesitas
};

#endif // MOTORES_BRINGUP__MOTOR_VEL_CONTROLLER_HPP_