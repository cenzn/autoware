
#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
// #include "autoware/dummy_diag_publisher/dummy_diag_publisher_core.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class ScanMatcherInterface : public rclcpp::Node
{
public:
  ScanMatcherInterface()
  : Node("scan_matcher_interface"),
    scan_matcher_arrived_(false),
    initialized_(false),
    initialized3d_(false)
  {
    rclcpp::QoS init_qos(1);
    init_qos.transient_local();
    init_qos.reliable();
    
    // Subscribers
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ekf_odom", 10, std::bind(&ScanMatcherInterface::pose_callback, this, _1));
    encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/qcar2_joint", 10, std::bind(&ScanMatcherInterface::encoder_callback, this, _1));
    route_sub_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route",10,std::bind(&ScanMatcherInterface::route_callback, this, _1));
    // Publishers
    kinematic_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose_twist_fusion_filter/pose", 10);
    init_pose3d_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d", init_qos);
    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", init_qos);
    init_state_pub_ = this->create_publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("/localization/initialization_state", init_qos);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
    //dummy diag
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));

    // Timer
    double period_s = 0.025;  // 20 Hz
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_s),std::bind(&ScanMatcherInterface::on_timer, this));
    
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Starting x.";
    param_desc.additional_constraints = "This is starting pose x";
    this->declare_parameter("init_x",0.0,param_desc);

    param_desc.description = "Starting y.";
    param_desc.additional_constraints = "This is starting pose y";
    this->declare_parameter("init_y",0.0,param_desc);

    param_desc.description = "Starting theta.";
    param_desc.additional_constraints = "This is starting pose th";
    this->declare_parameter("init_th",0.0,param_desc);

    init_x_ = this->get_parameter("init_x").as_double();
    init_y_ = this->get_parameter("init_y").as_double();
    init_th_ = this->get_parameter("init_th").as_double();
    RCLCPP_INFO(this->get_logger(),"Initial pose: x = %.3f, y = %.3f, theta = %.3f",
    init_x_, init_y_, init_th_);
  }

private:
  // Initial values
  double init_x_, init_y_, init_th_;
  double scaling_factor = 10.0;
  double front_axle_to_body = 0.13;
  double rear_axle_to_body = 0.13;
  double latest_steering_ang_;
  // for receiving data
  double latest_x_,latest_y_,latest_th_;
  geometry_msgs::msg::Quaternion latest_quat_,init_quat_, odom2map_quat_;
  std::deque<sensor_msgs::msg::JointState> joint_state_queue_;
  bool scan_matcher_arrived_ , initialized_, initialized3d_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose3d_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr init_state_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // dummy diag
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
 
  rclcpp::TimerBase::SharedPtr timer_;


  // For computing angular acceleration
  // rclcpp::Time last_imu_time_;
  // geometry_msgs::msg::Vector3 last_angular_velocity_;
  void route_callback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr /*route_msg_ptr*/)
  {
    if (!initialized3d_){
      // Pose
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
      init_pose_msg.header.stamp = this->now();
      init_pose_msg.header.frame_id = "map"; 

      init_pose_msg.pose.pose.position.x = init_x_;
      init_pose_msg.pose.pose.position.y = init_y_;
      init_pose_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion init_quaternion_tf2;
      init_quaternion_tf2.setRPY(0.0, 0.0, init_th_);
      init_quat_ = tf2::toMsg(init_quaternion_tf2);
      init_pose_msg.pose.pose.orientation = init_quat_;

      // Fill covariance (36 entries)
      for (int i = 0; i < 36; i++) {
      init_pose_msg.pose.covariance[i] = 0.0;
      }
      // Example: small covariance values in x, y, yaw
      init_pose_msg.pose.covariance[0] = 1.0;
      init_pose_msg.pose.covariance[7] = 1.0;
      init_pose_msg.pose.covariance[14] = 0.01;
      init_pose_msg.pose.covariance[21] = 0.01;
      init_pose_msg.pose.covariance[28] = 0.01;
      init_pose_msg.pose.covariance[35] = 0.2;
      init_pose3d_pub_->publish(init_pose_msg);
      RCLCPP_INFO(this->get_logger(),"Publishing /initialpose3d");
      initialized3d_ = true;
    }
  }
  
  void encoder_callback(const sensor_msgs::msg::JointState::ConstSharedPtr encoder_msg_ptr)
  {
      // joint_arrived_ = true;
      joint_state_queue_.push_back(*encoder_msg_ptr);
  }

  void pose_callback(const nav_msgs::msg::Odometry &scan_match_pose)
  {
    scan_matcher_arrived_ = true;
    //map is for real-size cars, hence the x10 gain
    latest_x_ = (scan_match_pose.pose.pose.position.x - init_x_)*10; 
    latest_y_ = (scan_match_pose.pose.pose.position.y - init_y_)*10;

    // Variables to store roll, pitch, and yaw
    double roll, pitch, yaw, quat_x, quat_y, quat_z, quat_w;
    
    // Convert quaternion to roll, pitch, and yaw
    quat_x = scan_match_pose.pose.pose.orientation.x;
    quat_y = scan_match_pose.pose.pose.orientation.y;
    quat_z = scan_match_pose.pose.pose.orientation.z;
    quat_w = scan_match_pose.pose.pose.orientation.w;

    tf2::Quaternion tf_quat(quat_x, quat_y, quat_z, quat_w);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    latest_th_ = yaw - init_th_;

    latest_th_ = fmod(latest_th_ + M_PI, 2 * M_PI);
    if (latest_th_ < 0)
    {
        latest_th_ += 2 * M_PI;
    }
    latest_th_ =  latest_th_ - M_PI;
    // RCLCPP_INFO(this->get_logger(),"Converted orientation: r = %.3f, p = %.3f, y = %.3f",
    // roll, pitch, yaw);
    
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, latest_th_);
    latest_quat_ = tf2::toMsg(quaternion_tf2);

  }

  void on_timer()
  {
    // Need pose + imu at least
    if (!scan_matcher_arrived_) {
        return;
    }
    if (joint_state_queue_.empty()) {
      // not output error and clear queue
      return;
    }
    // Get timestamp 
    rclcpp::Time now = this->now();

    // build and publish pose
    geometry_msgs::msg::PoseStamped pose_stamp_msg;
    pose_stamp_msg.header.stamp = now;
    pose_stamp_msg.header.frame_id = "map";
    pose_stamp_msg.pose.position.x = latest_x_;
    pose_stamp_msg.pose.position.y = latest_y_;
    pose_stamp_msg.pose.position.z = 0.0;
    pose_stamp_msg.pose.orientation = latest_quat_;
    pose_pub_->publish(pose_stamp_msg);

    // build and publish tf state message
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = latest_x_;
    t.transform.translation.y = latest_y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = latest_quat_;
    tf_broadcaster_->sendTransform(t);

    // velocity_status (longitutdina, lateral speed, heading rate)
    double v_mean = 0;
    for (const auto & joint_state : joint_state_queue_) {
        // convert encoder speed to vx
        v_mean += (joint_state.velocity.front()/(720.0*4.0))*((13.0*19.0)/(70.0*30.0))*(2.0*M_PI)*0.033;
    }
    v_mean /= static_cast<double>(joint_state_queue_.size());
    double beta, v_x, v_y , heading_rate;
    beta = std::atan(std::tan(latest_steering_ang_)*rear_axle_to_body/
                                        (front_axle_to_body+rear_axle_to_body));
    v_x = std::cos(beta)*v_mean;
    v_y = std::sin(beta)*v_mean;
    heading_rate = std::tan(latest_steering_ang_)/((front_axle_to_body+rear_axle_to_body))*v_mean; //w=(1/R*)*v 
    // body speed estimation in 1.10th scale first, then convert
    v_x *= scaling_factor;
    v_y *= scaling_factor;
    joint_state_queue_.clear();

    // build kinematic state message
    nav_msgs::msg::Odometry kinematic_state_msg;
    kinematic_state_msg.header.stamp = now;
    kinematic_state_msg.header.frame_id = "map";  // or your frame
    kinematic_state_msg.child_frame_id = "base_link";

    kinematic_state_msg.pose.pose.position.x = latest_x_;
    kinematic_state_msg.pose.pose.position.y = latest_y_;
    kinematic_state_msg.pose.pose.position.z = 0.0;

    kinematic_state_msg.pose.pose.orientation = latest_quat_;

    // Fill covariance (36 entries)
    for (int i = 0; i < 36; i++) {
    kinematic_state_msg.pose.covariance[i] = 0.0;
    }
    // Example: small covariance values in x, y, yaw
    kinematic_state_msg.pose.covariance[0] = 0.0001;
    kinematic_state_msg.pose.covariance[7] = 0.0001;
    kinematic_state_msg.pose.covariance[35] = 0.0001;

    // Twist
    kinematic_state_msg.twist.twist.linear.x = v_x;
    kinematic_state_msg.twist.twist.linear.y = v_y;
    kinematic_state_msg.twist.twist.linear.z = 0.0;

    kinematic_state_msg.twist.twist.angular.x = 0.0;
    kinematic_state_msg.twist.twist.angular.y = 0.0;
    kinematic_state_msg.twist.twist.angular.z = heading_rate;

    for (int i = 0; i < 36; i++) {
        kinematic_state_msg.twist.covariance[i] = 0.0;
    }
    kinematic_state_msg.twist.covariance[0] = 0.0001;
    kinematic_state_msg.twist.covariance[7] = 0.0001;
    kinematic_state_msg.twist.covariance[35] = 0.0001;

    kinematic_state_pub_->publish(kinematic_state_msg);
    
    //build and publish dummy diag
    diagnostic_msgs::msg::DiagnosticArray array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
    status.hardware_id = "dummy_diag";
    status.name = "localization_error_monitor: ellipse_error_status";
    array.status.push_back(status);
    status.name = "localization: ekf_localizer";
    array.status.push_back(status);
    status.name = "ndt_scan_matcher: scan_matching_status";
    array.status.push_back(status);
    array.header.stamp = now;
    diag_pub_->publish(array);

    if (!initialized_)
    {
      now = this->now();
      // construct insital pose 3d
      // build kinematic state message
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;

      // Pose
      init_pose_msg.header.stamp = now;
      init_pose_msg.header.frame_id = "map"; 

      init_pose_msg.pose.pose.position.x = init_x_;
      init_pose_msg.pose.pose.position.y = init_y_;
      init_pose_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion init_quaternion_tf2;
      init_quaternion_tf2.setRPY(0.0, 0.0, init_th_);
      init_quat_ = tf2::toMsg(init_quaternion_tf2);
      init_pose_msg.pose.pose.orientation = init_quat_;

      // Fill covariance (36 entries)
      for (int i = 0; i < 36; i++) {
      init_pose_msg.pose.covariance[i] = 0.0;
      }
      // // Example: small covariance values in x, y, yaw
      // init_pose_msg.pose.covariance[0] = 1.0;
      // init_pose_msg.pose.covariance[7] = 1.0;
      // init_pose_msg.pose.covariance[14] = 0.01;
      // init_pose_msg.pose.covariance[21] = 0.01;
      // init_pose_msg.pose.covariance[28] = 0.01;
      // init_pose_msg.pose.covariance[35] = 0.2;
      // init_pose3d_pub_->publish(init_pose_msg);
      // RCLCPP_INFO(this->get_logger(),"Publishing /initialpose3d");

      init_pose_msg.pose.covariance[0] = 0.25;
      init_pose_msg.pose.covariance[7] = 0.25;
      init_pose_msg.pose.covariance[14] = 0.0;
      init_pose_msg.pose.covariance[21] = 0.0;
      init_pose_msg.pose.covariance[28] = 0.0;
      init_pose_msg.pose.covariance[35] = 0.0685;
      init_pose_pub_->publish(init_pose_msg);
      RCLCPP_INFO(this->get_logger(),"Publishing /initialpose");

      autoware_adapi_v1_msgs::msg::LocalizationInitializationState init_state_msg;
      init_state_msg.state = init_state_msg.INITIALIZED;
      init_state_msg.stamp = now; 
      init_state_pub_->publish(init_state_msg);

      initialized_ = true;
    }

    // acceleration
    // // transform gyro frame
    // for (auto & gyro : gyro_queue_) {
    //     geometry_msgs::msg::Vector3Stamped angular_velocity;
    //     angular_velocity.header = gyro.header;
    //     angular_velocity.vector = gyro.angular_velocity;

    //     geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
    //     transformed_angular_velocity.header = tf_imu2base_ptr->header;
    //     tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

    //     gyro.header.frame_id = output_frame_;
    //     gyro.angular_velocity = transformed_angular_velocity.vector;
    //     gyro.angular_velocity_covariance = transform_covariance(gyro.angular_velocity_covariance);
    // }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanMatcherInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
