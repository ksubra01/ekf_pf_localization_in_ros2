#pragma once
#include <vector>
#include <cmath>
#include <random>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Landmark {
    int id;
    double x;
    double y;
}