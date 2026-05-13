#include <algorithm>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class MavrosMotorController : public rclcpp::Node
{
public:
    explicit MavrosMotorController(const rclcpp::NodeOptions & options)
    : Node("cmd_vel_motor_controller_mavros", options)
    {
        rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");
    }

    bool armVehicle()
    {
        if (!arming_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Servicio /mavros/cmd/arming no disponible");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto future = arming_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Vehículo armado correctamente");
            return true;
        }

        RCLCPP_ERROR(this->get_logger(), "Fallo al armar el vehículo");
        return false;
    }

    void handleTwist(const geometry_msgs::msg::Twist & msg)
    {
        double velocity = std::clamp(msg.linear.x, -1.0, 1.0);
        uint16_t pwm_1 = static_cast<uint16_t>(1500 + velocity * 400);
        uint16_t pwm_2 = static_cast<uint16_t>(1500 - velocity * 400);

        mavros_msgs::msg::OverrideRCIn rc_msg;
        rc_msg.channels = {
            pwm_1,
            0,
            pwm_2,
            0,
            0, 0, 0, 0
        };

        rc_override_pub_->publish(rc_msg);
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
};

class CmdVelInputController : public rclcpp::Node
{
public:
    CmdVelInputController(
        const rclcpp::NodeOptions & options,
        const std::shared_ptr<MavrosMotorController> & mavros_controller)
    : Node("cmd_vel_motor_controller", options), mavros_controller_(mavros_controller)
    {
        rudder_pub_ = this->create_publisher<std_msgs::msg::Int32>("rudder_angle", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelInputController::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CmdVelMotorController iniciado en el dominio por defecto");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        int rudder_angle = static_cast<int>(msg->angular.z * 40.0);
        rudder_angle = std::clamp(rudder_angle, -35, 40);

        std_msgs::msg::Int32 rudder_msg;
        rudder_msg.data = rudder_angle;
        rudder_pub_->publish(rudder_msg);

        if (mavros_controller_)
        {
            mavros_controller_->handleTwist(*msg);
        }
    }

    std::shared_ptr<MavrosMotorController> mavros_controller_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rudder_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char ** argv)
{
    auto default_context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions default_init_options;
    default_context->init(argc, argv, default_init_options);

    auto mavros_context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions mavros_init_options;
    mavros_init_options.shutdown_on_signal = false;
    mavros_init_options.set_domain_id(2);
    mavros_context->init(0, nullptr, mavros_init_options);

    rclcpp::NodeOptions mavros_node_options;
    mavros_node_options.context(mavros_context);
    auto mavros_controller = std::make_shared<MavrosMotorController>(mavros_node_options);

    rclcpp::NodeOptions cmd_vel_node_options;
    cmd_vel_node_options.context(default_context);
    auto cmd_vel_controller = std::make_shared<CmdVelInputController>(
        cmd_vel_node_options, mavros_controller);

    mavros_controller->armVehicle();

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = default_context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(cmd_vel_controller);
    executor.spin();

    default_context->shutdown("cmd_vel controller stopped");
    mavros_context->shutdown("mavros controller stopped");
    return 0;
}
