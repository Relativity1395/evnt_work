#pragma once

#include <vector>
#include <Eigen/Dense>

// The point of this file is to have basic types includable in one place
// No algorithm step should have to import another step just to use one of its structs
// This file should not depend on anything other than libraries defining data types (E.g Eigen)

struct Event{
    Eigen::Vector2d position; // vector for x and y positions
    double timestamp;
    bool polarity;
};

struct ImuSample{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    double timestamp;
};

// Pose?
// state stuff?