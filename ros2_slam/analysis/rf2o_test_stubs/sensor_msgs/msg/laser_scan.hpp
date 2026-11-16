#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
namespace sensor_msgs { namespace msg {
struct LaserScan {
  struct Header { builtin_interfaces::msg::Time stamp; } header;
  float angle_min=0,angle_max=0,angle_increment=0,range_min=0.05f,range_max=20;
  std::vector<float> ranges;
};
}}
