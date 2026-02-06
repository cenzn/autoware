
#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_perception_msgs/msg/traffic_light_group_array.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class FakePerception : public rclcpp::Node
{
public:
  FakePerception()
  : Node("fake_perception")
  {
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FakePerception::scan_callback, this, std::placeholders::_1));
    
    // Publishers
    obj_detect_pub_ = this->create_publisher<autoware_perception_msgs::msg::PredictedObjects >("/perception/object_recognition/objects", 10);
    traffic_pub_ =  this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray >("/perception/traffic_light_recognition/traffic_signals", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacle_segmentation/pointcloud", 10);
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/perception/occupancy_grid_map/map", 10);
    
    // Timer
    double period_s = 0.1;  // 20 Hz
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_s),std::bind(&FakePerception::on_timer, this));
    
    RCLCPP_INFO(this->get_logger(),"Starting Fake Perception loop");

  }

private:
  
  // Publishers
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects >::SharedPtr obj_detect_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray >::SharedPtr traffic_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  // fake laser 3d point cloud
  laser_geometry::LaserProjection projector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::msg::LaserScan dummy_scan;
    dummy_scan = *scan;
    dummy_scan.range_min *= 10;
    dummy_scan.range_max *= 10;
    // Scale ranges by 10
    for (auto & r : dummy_scan.ranges)
    {
      if (std::isfinite(r)) {
        r *= 10.0f+100.0;
      }
    }
    // Convert LaserScan → PointCloud2
    projector_.projectLaser(dummy_scan, cloud);

    cloud.header = scan->header;
    cloud.header.frame_id = "base_link";
    cloud_pub_->publish(cloud);
  }

  void on_timer()
  {
    rclcpp::Time now = this->now();

    // build and publish detected object msgs
    autoware_perception_msgs::msg::PredictedObjects obj_msg;
    obj_msg.header.stamp = now;
    obj_msg.header.frame_id = "map";
    // obj_msg.objects = [];
    obj_detect_pub_->publish(obj_msg);

    // build and publish traffic signal msgs
    autoware_perception_msgs::msg::TrafficLightGroupArray traffic_msg;
    traffic_msg.stamp = now;
    // obj_msg.objects = [];
    traffic_pub_->publish(traffic_msg);

    // build and publish dummy occupancy grid msgs
    nav_msgs::msg::OccupancyGrid map_;
    map_.header.frame_id = "map";

    map_.info.resolution = 0.05;   // meters per cell
    map_.info.width = 300;
    map_.info.height = 300;

    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    map_.data.resize(map_.info.width * map_.info.height, 0);
    occupancy_grid_pub_->publish(map_);

  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakePerception>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
