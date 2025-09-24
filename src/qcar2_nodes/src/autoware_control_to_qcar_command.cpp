#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "std_msgs/msg/header.hpp"

#include "qcar2_interfaces/srv/autoware_command_convert.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

class Autoware2Qcar : public rclcpp::Node
{
public:
    Autoware2Qcar()
    : Node("autoware_command_converter_client")
    {
        autoware_cmd_subscriber_ = this->create_subscription<autoware_control_msgs::msg::Control>("control/command/control_cmd",10,std::bind(&Autoware2Qcar::sub_callback,this,_1));
        qcar_cmd_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd",10);
        command_converter_client = this->create_client<qcar2_interfaces::srv::AutowareCommandConvert>("autoware_command_converter");
    }

private:

    void sub_callback(const autoware_control_msgs::msg::Control &motor_control)
    {
        // making a service call to the converter and pull converted motor commands. 
        auto converter_request = std::make_shared<qcar2_interfaces::srv::AutowareCommandConvert::Request>();
        converter_request->autoware_commands = motor_control;
        
        while (!this->command_converter_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // // Wait for the result.
        // // cant spin twice
        // auto result = this->command_converter_client->async_send_request(converter_request);
        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        //     rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     qcar2_interfaces::msg::MotorCommands motor_cmd = result.get()->qcar2_commands;
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response: steering_angle=%f, motor_throttle=%f",
        //         motor_cmd.values[0], motor_cmd.values[1]);
        //     motor_cmd.values[1]=0;
        //     this->qcar_cmd_publisher_->publish(motor_cmd);
        // } else {
        //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        // }

        using ServiceResponseFuture =
            rclcpp::Client<qcar2_interfaces::srv::AutowareCommandConvert>::SharedFuture;
        auto response_received_callback =
            [this](ServiceResponseFuture future) {
                auto motor_cmd = future.get()->qcar2_commands;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response: steering_angle=%f, motor_throttle=%f",
                    motor_cmd.values[0], motor_cmd.values[1]);
                motor_cmd.values[1]=motor_cmd.values[1]/10;
                this->qcar_cmd_publisher_->publish(motor_cmd);
            };

        this->command_converter_client->async_send_request(converter_request, response_received_callback);

    }

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr autoware_cmd_subscriber_;
    rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr qcar_cmd_publisher_;
    rclcpp::Client<qcar2_interfaces::srv::AutowareCommandConvert>::SharedPtr command_converter_client;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS environment
    rclcpp::init(argc, argv);

    // Instantiate the node
    rclcpp::Node::SharedPtr node = std::make_shared<Autoware2Qcar>();

    // Get a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting Autoware2Qcar loop...");
    executor.spin();
    RCLCPP_INFO(node->get_logger(), "qcar2 loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}