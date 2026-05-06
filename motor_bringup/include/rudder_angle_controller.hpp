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
   rclcpp::TimerBase::SharedPtr control_timer_;
   void processMotion();
   void replanMotion();
   void updateEstimatedAngle();
   bool writeGoalVelocity(int32_t velocity);
   void stopMotor();
   // Helper methods to read encoder values from Dynamixel
   int32_t getCurrentPosition(uint8_t id);
   int32_t getCurrentVelocity(uint8_t id);
   double current_angle_deg_;
   double target_angle_deg_;
   double motion_start_angle_deg_;
   double motion_goal_angle_deg_;
   double motion_duration_s_;
   bool motion_active_;
   int32_t active_velocity_command_;
   rclcpp::Time motion_start_time_;
   // Puedes añadir aquí otros miembros si los necesitas
};

#endif // MOTORES_BRINGUP__MOTOR_VEL_CONTROLLER_HPP_