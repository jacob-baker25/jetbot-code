// Offline only: actual RF2O core with message/clock stubs, no robot commands.
#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("Expected scan sequence file");
    std::ifstream in(argv[1]); int count=0;
    if (!(in >> count) || count<2 || count>100000)
      throw std::runtime_error("Invalid sequence length");
    rf2o::CLaserOdometry2D core;
    geometry_msgs::msg::Pose pose; pose.orientation.w=1;
    core.suppress_small_motion=false;
    for (int n=0; n<count; ++n) {
      sensor_msgs::msg::LaserScan scan;
      if (!(in >> scan.header.stamp.sec >> scan.header.stamp.nanosec >> scan.angle_increment))
        throw std::runtime_error("Truncated scan header");
      for (int j=0; j<360; ++j) {
        std::string value;
        if (!(in >> value)) throw std::runtime_error("Truncated ranges");
        scan.ranges.push_back(std::stof(value));
      }
      if (!n) { core.init(scan,pose); continue; }
      bool accepted=core.odometryCalculation(scan);
      const auto &p=core.getPose();
      if (!p.matrix().allFinite()) throw std::runtime_error("Nonfinite replay pose");
      std::cout << std::setprecision(12) << n << ',' << accepted << ','
        << core.failure_reason << ',' << p.translation().x() << ','
        << p.translation().y() << ',' << rf2o::getYaw(p.rotation()) << '\n';
      // Match the live node's terminal reference expiry; never silently reset.
      if (std::string(core.failure_reason)=="reference_expired") break;
    }
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
