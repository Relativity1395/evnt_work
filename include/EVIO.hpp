#pragma once
#include <Eigen/Dense>
#include <DVX_Drivers.hpp>
class StateVector{
    private:
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro; 
        Eigen::Vector3d velocity;
        Eigen::Vector3d position;
        Eigen::Quaterniond q;
        Eigen::Vector3d accelBias;
        Eigen::Vector3d gyroBias;
    public:
        StateVector();
        void propagate(const imu_t& imu, double dt);
        
};