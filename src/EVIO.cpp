#include <../include/EVIO.hpp>

StateVector::StateVector()
{
    accel = Eigen::Vector3d::Zero();
    gyro = Eigen::Vector3d::Zero();

    velocity = Eigen::Vector3d::Zero();
    position = Eigen::Vector3d::Zero();

    accelBias = Eigen::Vector3d::Zero();
    gyroBias = Eigen::Vector3d::Zero();

    q = Eigen::Quaterniond::Identity();
}
void StateVector::propagate(const imu_t& imu, double dt){
    Eigen::Vector3d a(imu.accel_x, imu.accel_y, imu.accel_z);
    Eigen::Vector3d w(imu.gyro_x, imu.gyro_y, imu.gyro_z);
    a -= accelBias;
    w -= gyroBias;
    Eigen::Quaterniond qWorld = q.conjugate();
    Eigen::Vector3d aWorld = qWorld * a;
    Eigen::Vector3d g(0,0,-9.81);
    aWorld += g;
    Eigen::Vector3d vold = velocity;
    velocity += aWorld * dt;
    position += vold * dt + 0.5*aWorld*dt*dt;
    double angularSpeed = w.norm();
    if(angularSpeed >(1e-5)){
        double angle = angularSpeed * dt;
        Eigen::Vector3d axis = w.normalized();
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
        q = (dq.conjugate() * q).normalized();
    }
    //Im creating q as a rotation of angle and axis
    
    
}