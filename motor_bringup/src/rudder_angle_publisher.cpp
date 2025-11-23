#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rudder_angle_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/rudder_angle", 10);

    std::cout << "Rudder Angle Publisher. Enter an angle (int) and press Enter to publish. Ctrl+C to exit." << std::endl;

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        std::string input;
        std::cout << "Angle (-40 to 40): ";
        std::getline(std::cin, input);
        if (!rclcpp::ok()) break;
        try {
            int angle = std::stoi(input);
            if (angle < -40 || angle > 40) {
                std::cout << "Error: Angle must be between -40 and 40." << std::endl;
                continue;
            }
            auto msg = std_msgs::msg::Int32();
            msg.data = angle;
            publisher->publish(msg);
            std::cout << "Published: " << angle << std::endl;
        } catch (const std::exception &e) {
            std::cout << "Invalid input. Please enter a valid int." << std::endl;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
