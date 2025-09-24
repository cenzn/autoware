#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "std_msgs/msg/header.hpp"

#include "qcar2_interfaces/srv/autoware_command_convert.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"

void convert(std::shared_ptr<qcar2_interfaces::srv::AutowareCommandConvert::Request> request,
             std::shared_ptr<qcar2_interfaces::srv::AutowareCommandConvert::Response> response)
{
    
    
    std::vector<std::string> name;
    std::vector<double> val;
    double speed = 0;
    double steering = 0;
    
    // Convert the Twist message to MotorCommands
    steering = request->autoware_commands.lateral.steering_tire_angle;
    speed = request->autoware_commands.longitudinal.velocity;
    val.push_back(steering);
    val.push_back(speed);
    
    name.push_back("steering_angle");
    name.push_back("motor_throttle");

    response->qcar2_commands.motor_names = name;
    response->qcar2_commands.values = val;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request: lateral=%f, longitudinal=%f",
                steering, speed);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending response: steering_angle=%f, motor_throttle=%f",
                steering, speed);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("autoware_command_convert_server");

    rclcpp::Service<qcar2_interfaces::srv::AutowareCommandConvert>::SharedPtr service =
        node->create_service<qcar2_interfaces::srv::AutowareCommandConvert>("autoware_command_converter", &convert);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to convert Autoware Control to QCar2 MotorCommands.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}