#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

class CmdVelMotorController : public rclcpp::Node
{
public:
    CmdVelMotorController() : Node("cmd_vel_motor_controller")
    {
        // Subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelMotorController::cmdVelCallback, this, std::placeholders::_1));

        // Publisher for rudder angle
        rudder_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/rudder_angle", 10);

        // Publisher for RC override
        rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);

        // Arm service client
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");

        // Try to arm on startup
        armVehicle();

        RCLCPP_INFO(this->get_logger(), "CmdVelMotorController iniciado");
    }

private:

    void armVehicle()
    {
        if (!arming_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Servicio /mavros/cmd/arming no disponible");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto future = arming_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Vehículo armado correctamente");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Fallo al armar el vehículo");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // === CONTROL DE TIMÓN ===
        // angular.z -> rango -40 a 40 grados
        int rudder_angle = static_cast<int>(msg->angular.z * 40.0);
        rudder_angle = std::clamp(rudder_angle, -35, 40);    // Limitar ángulo entre -35 y 40 grados

        std_msgs::msg::Int32 rudder_msg;
        rudder_msg.data = rudder_angle;
        rudder_pub_->publish(rudder_msg);

        // === CONTROL DE MOTORES ===
        // linear.x -> potencia simétrica para motores 1 y 3
        double velocity = std::clamp(msg->linear.x, -1.0, 1.0);

        // Mapeo -1..1 -> 1100..1900 (1500 neutro)
        uint16_t pwm_1 = static_cast<uint16_t>(1500 + velocity * 400);
        uint16_t pwm_2 = static_cast<uint16_t>(1500 - velocity * 400);


        mavros_msgs::msg::OverrideRCIn rc_msg;
        rc_msg.channels = {
            pwm_1, // RC1 Motor izquierdo
            0,
            pwm_2, // RC3 Motor derecho
            0,
            0, 0, 0, 0
        };

        rc_override_pub_->publish(rc_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rudder_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelMotorController>());
    rclcpp::shutdown();
    return 0;
}