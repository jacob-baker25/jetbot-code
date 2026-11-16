#pragma once
// Standalone algorithm tests only. Never add this directory to the ROS build.
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
namespace builtin_interfaces { namespace msg {
struct Time { int32_t sec=0; uint32_t nanosec=0; };
}}
namespace rclcpp {
struct Duration { double value; double seconds() const { return value; } };
class Time {
  double value_=0;
public:
  Time() = default;
  Time(const builtin_interfaces::msg::Time &t): value_(t.sec+t.nanosec*1e-9) {}
  explicit Time(double t): value_(t) {}
  double seconds() const { return value_; }
  Duration operator-(const Time &other) const { return {value_-other.value_}; }
};
struct Clock {
  Time now() const { return Time(std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count()); }
};
class Node {
public:
  explicit Node(const std::string &) {}
  std::shared_ptr<Clock> get_clock() const { return std::make_shared<Clock>(); }
  int get_logger() const { return 0; }
};
}
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
