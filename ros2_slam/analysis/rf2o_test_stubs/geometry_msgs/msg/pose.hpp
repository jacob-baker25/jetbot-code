#pragma once
#include "geometry_msgs/msg/quaternion.hpp"
namespace geometry_msgs { namespace msg {
struct Point { double x=0,y=0,z=0; };
struct Pose { Point position; Quaternion orientation; };
}}
