#pragma once
#include <Eigen/Dense>
class StateVector{
    private:
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro; 
        Eigen::Vector3d velocity;
        Eigen::Vector3d position;
        Eigen::Quaterniond q;

    public:
        StateVector();
        
};