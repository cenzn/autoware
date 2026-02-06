#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"


#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::placeholders;

class Qcar2VehicleInterface : public rclcpp::Node
{
public:
    Qcar2VehicleInterface()
    : Node("Qcar2VehicleInterface"),
      joint_arrived_(false),
      imu_arrived_(false),
      motor_command_arrived_(false),
      set_mode(false)
    {
        rclcpp::QoS init_qos(1);
        init_qos.transient_local();
        init_qos.reliable();
        // subscribers
        autoware_cmd_subscriber_ = this->create_subscription<autoware_control_msgs::msg::Control>("control/command/control_cmd",10,std::bind(&Qcar2VehicleInterface::ctrl_sub_callback,this,_1));
        encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/qcar2_joint", 10, std::bind(&Qcar2VehicleInterface::encoder_callback, this, _1));
        steering_sub_ = this->create_subscription<qcar2_interfaces::msg::MotorCommands>("/qcar2_motor_speed_cmd", 10, std::bind(&Qcar2VehicleInterface::steering_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/qcar2_imu", 10, std::bind(&Qcar2VehicleInterface::imu_callback, this, _1));
    
        // publisher 
        accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 10);
        vehicle_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
        steering_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
        qcar_cmd_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd",10);
        ctrl_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode",10);
        gear_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status",10);
        hazard_light_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status",10);
        turn_light_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status",10);
        // op_mode_pub_ = this->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/api/operation_mode/state", init_qos);
        // Qcar2VehicleInterface::ctrl_mode_init();
        
        double period_s = 0.025;
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period_s),std::bind(&Qcar2VehicleInterface::on_timer, this));

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "steer gain.";
        param_desc.additional_constraints = "Steer gain amplify the steer command from autoware to QCar Motor Control";
        this->declare_parameter("steer_gain",1.0,param_desc);

        steer_gain = this->get_parameter("steer_gain").as_double();

    }

private:
    // variables
    bool joint_arrived_, imu_arrived_, motor_command_arrived_, set_mode;
    double steer_gain = 1.0;
    double scaling_factor = 10.0;
    double front_axle_to_body = 0.13;
    double rear_axle_to_body = 0.13;
    double latest_steering_ang_ = 0;
    double latest_steering_cmd_ = 0;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    std::deque<sensor_msgs::msg::JointState> joint_state_queue_;
    std::deque<sensor_msgs::msg::Imu> imu_queue_;

    // subscribers
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr autoware_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
    rclcpp::Subscription<qcar2_interfaces::msg::MotorCommands>::SharedPtr steering_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    // publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr qcar_cmd_publisher_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr ctrl_mode_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_light_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_light_pub_;
    // rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr op_mode_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    // void ctrl_mode_init()
    // {
    //     autoware_vehicle_msgs::msg::ControlModeReport ctrl_mode_msg;
    //     ctrl_mode_msg.mode = ctrl_mode_msg.AUTONOMOUS 
    //     ctrl_mode_pub_->publish(ctrl_mode_msg)
    // }

    void encoder_callback(const sensor_msgs::msg::JointState::ConstSharedPtr encoder_msg_ptr)
    {
        joint_arrived_ = true;
        joint_state_queue_.push_back(*encoder_msg_ptr);
    }

    void steering_callback(const qcar2_interfaces::msg::MotorCommands &cmd_msg)
    {
        motor_command_arrived_ = true;
        latest_steering_cmd_ = cmd_msg.values.front();
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
    {
        imu_arrived_ = true;
        imu_queue_.push_back(*imu_msg_ptr);
        imu_pub_->publish(*imu_msg_ptr);
    }
    
    void ctrl_sub_callback(const autoware_control_msgs::msg::Control &autoware_cmd)
    {
        rclcpp::Time now = this->now();
        
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
        qcar2_cmd.values[1]=qcar2_cmd.values[1]*0.1;
        qcar_cmd_publisher_->publish(qcar2_cmd);

    }

    void on_timer()
    {
        if (!joint_arrived_ || !imu_arrived_) {
        return;
        }
        // if (!motor_command_arrived_){
        //     latest_steering_ang_ = 0;
        // }
        if (joint_state_queue_.empty()) {
            // not output error and clear queue
            return;
        }
        if (imu_queue_.empty()) {
            // not output error and clear queue
            return;
        }
        // Get timestamp 
        rclcpp::Time now = this->now();
    
        // Process IMU data
        double imu_x_mean = 0;
        double imu_y_mean = 0;
        double imu_th_mean = 0;

        for (const auto & imu_data : imu_queue_) {
            // convert encoder speed to vx
            imu_x_mean += imu_data.linear_acceleration.x;
            imu_y_mean += imu_data.linear_acceleration.y;
            imu_th_mean += imu_data.angular_velocity.z;
        }
        imu_x_mean /= static_cast<double>(imu_queue_.size());
        imu_y_mean /= static_cast<double>(imu_queue_.size());
        imu_th_mean /= static_cast<double>(imu_queue_.size());

        // --- Build Acceleration message ---

        geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
        const auto latest_imu_stamp = rclcpp::Time(imu_queue_.back().header.stamp);
        accel_msg.header.stamp = latest_imu_stamp;
        accel_msg.header.frame_id = "base_link";  // or base frame

        // Linear acceleration from IMU (body frame)
        accel_msg.accel.accel.linear.x = imu_x_mean;
        accel_msg.accel.accel.linear.y = imu_y_mean;
        accel_msg.accel.accel.linear.z = 0.0;
        accel_msg.accel.accel.angular.x = 0.0;
        accel_msg.accel.accel.angular.y = 0.0;
        accel_msg.accel.accel.angular.z = 0.0;

        // Covariance
        for (int i = 0; i < 36; i++) {
            accel_msg.accel.covariance[i] = 0.0;
        }
        accel_msg.accel.covariance[0] = 0.001;
        accel_msg.accel.covariance[7] = 0.001;

        accel_pub_->publish(accel_msg);
        imu_queue_.clear();
        
        // v_mean from encoder speed
        double v_mean = 0;
        for (const auto & joint_state : joint_state_queue_) {
            // convert encoder speed to vx
            v_mean += (joint_state.velocity.front()/(720.0*4.0))*((13.0*19.0)/(70.0*30.0))*(2.0*M_PI)*0.033;
        }
        v_mean /= static_cast<double>(joint_state_queue_.size());
        
        // compute steering angle based on body turn speed (imu_th_mean) and foward speed (v_mean) using bycicle model
        double R_star;
        R_star = v_mean/(imu_th_mean-0.01);
        // latest_steering_ang_ = std::atan2(front_axle_to_body+rear_axle_to_body,R_star);
        // if (R_star==0.0){latest_steering_ang_ = latest_steering_cmd_;}
        // if (R_star<0.0){latest_steering_ang_-=M_PI;}
        // if (latest_steering_ang_>3.141){latest_steering_ang_=0.0;}
        // RCLCPP_INFO(this->get_logger(), "omega: %.2f, speed: %.2f, R: %.2f, Lf+lr: %.2f,steer: %.3f",imu_th_mean-0.01,v_mean,v_mean/(imu_th_mean-0.01),front_axle_to_body+rear_axle_to_body,latest_steering_ang_);
        latest_steering_ang_ = latest_steering_cmd_;


        double beta, v_x, v_y; // , heading_rate;
        beta = std::atan(std::tan(latest_steering_ang_)*rear_axle_to_body/
                                            (front_axle_to_body+rear_axle_to_body));
        v_x = v_mean;
        v_y = std::tan(beta)*v_mean;
        // heading_rate = std::tan(latest_steering_ang_)/((front_axle_to_body+rear_axle_to_body))*v_mean; //w=(1/R*)*v 
        // body speed estimation in 1.10th scale first, then convert
        v_x *= scaling_factor;
        v_y *= scaling_factor;
        joint_state_queue_.clear();

        
        
        // build and publish vehicle report
        autoware_vehicle_msgs::msg::VelocityReport vehicle_report_msg;
        vehicle_report_msg.header.stamp = now;
        vehicle_report_msg.longitudinal_velocity = v_x;
        vehicle_report_msg.lateral_velocity = v_y;
        vehicle_report_msg.heading_rate = imu_th_mean-0.01; // 0.01 bias at 0 movement
        vehicle_report_pub_->publish(vehicle_report_msg);

        // build and publish steering report
        autoware_vehicle_msgs::msg::SteeringReport steer_report_msg;
        steer_report_msg.stamp = now;
        steer_report_msg.steering_tire_angle = latest_steering_ang_*steer_gain;
        steering_report_pub_->publish(steer_report_msg);  

        // control mode
        autoware_vehicle_msgs::msg::ControlModeReport ctrl_mode_msg;
        ctrl_mode_msg.mode = ctrl_mode_msg.AUTONOMOUS;
        ctrl_mode_msg.stamp = now;
        ctrl_mode_pub_->publish(ctrl_mode_msg);
        
        // gear report
        autoware_vehicle_msgs::msg::GearReport gear_msg;
        gear_msg.report = gear_msg.DRIVE;
        gear_msg.stamp = now;
        gear_report_pub_->publish(gear_msg);

        // hazard light
        autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
        hazard_msg.report = hazard_msg.DISABLE ;
        hazard_msg.stamp = now;
        hazard_light_pub_->publish(hazard_msg);

        // turn light
        autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_signal_msg;
        turn_signal_msg.report = turn_signal_msg.DISABLE ;
        turn_signal_msg.stamp = now;
        turn_light_pub_->publish(turn_signal_msg);

        // if (!set_mode)
        // {
        //     // operation mode
        //     autoware_adapi_v1_msgs::msg::OperationModeState op_mode_msg;
        //     op_mode_msg.mode = op_mode_msg.AUTONOMOUS;
        //     op_mode_msg.stamp = now; 
        //     op_mode_msg.is_autoware_control_enabled = true;
        //     op_mode_pub_->publish(op_mode_msg);
        //     set_mode = true;
        // }
    }
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
    RCLCPP_INFO(node->get_logger(), "Vehicle Interface loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}