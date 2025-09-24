#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "autoware_control_msgs/msg/control.hpp"

#include "qcar2_interfaces/srv/autoware_command_convert.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::placeholders;

class Qcar2VehicleInterface : public rclcpp::Node
{
public:
    Qcar2VehicleInterface()
    : Node("Qcar2VehicleInterface")
    {
        scan_matcher_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/scan_matcher",10,std::bind(&Qcar2VehicleInterface::scan_sub_callback,this,_1));
        autoware_cmd_subscriber_ = this->create_subscription<autoware_control_msgs::msg::Control>("control/command/control_cmd",10,std::bind(&Qcar2VehicleInterface::ctrl_sub_callback,this,_1));
        qcar_cmd_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd",10);
        // gps_publisher_ this->create_publisher<>;

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Starting x.";
        param_desc.additional_constraints = "This is starting pose x";
        this->declare_parameter("starting_x",starting_x,param_desc);

        param_desc.description = "Starting y.";
        param_desc.additional_constraints = "This is starting pose y";
        this->declare_parameter("starting_y",starting_y,param_desc);

        param_desc.description = "Starting theta.";
        param_desc.additional_constraints = "This is starting pose th";
        this->declare_parameter("starting_th",starting_th,param_desc);

        starting_x = this->get_parameter("starting_x").as_double();
        starting_y = this->get_parameter("starting_y").as_double();
        starting_th = this->get_parameter("starting_th").as_double();


    }

private:

    void scan_sub_callback(const nav_msgs::msg::Odometry &scan_match_pose)
    {
        double x = scan_match_pose.pose.pose.position.x - starting_x;
        double y = scan_match_pose.pose.pose.position.y - starting_y;
        double th = scan_match_pose.pose.pose.orientation.z - starting_th;

        th = fmod(th + M_PI, 2 * M_PI);
        if (th < 0)
        {
            th += 2 * M_PI;
        }
        th =  th - M_PI;

    }

    void ctrl_sub_callback(const autoware_control_msgs::msg::Control &autoware_cmd)
    {
 
        qcar2_interfaces::msg::MotorCommands qcar2_cmd;
        std::vector<std::string> name;
        std::vector<double> val;
        double speed = 0;
        double steering = 0;
        
        // Convert the autoware command to qcar2 command
        steering = autoware_cmd.lateral.steering_tire_angle;
        speed = autoware_cmd.longitudinal.velocity;
        val.push_back(steering);
        val.push_back(speed);
        
        name.push_back("steering_angle");
        name.push_back("motor_throttle");

        qcar2_cmd.motor_names = name;
        qcar2_cmd.values = val;
        qcar2_cmd.values[1]=qcar2_cmd.values[1]/10;
        this->qcar_cmd_publisher_->publish(qcar2_cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr scan_matcher_pose_subscriber_;
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr autoware_cmd_subscriber_;
    rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr qcar_cmd_publisher_;
    double starting_x = 0.0;
    double starting_y = 0.0;
    double starting_th = 0.0;

};

int main(int argc, char * argv[])
{
    // Initialize the ROS environment
    rclcpp::init(argc, argv);

    // Instantiate the node
    rclcpp::Node::SharedPtr node = std::make_shared<Qcar2VehicleInterface>();

    // Get a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting Qcar2 Vehicle Interface loop...");
    executor.spin();
    RCLCPP_INFO(node->get_logger(), "scan_matcher_to_gps loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}