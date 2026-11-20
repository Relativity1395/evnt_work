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
    Eigen::Vector3d aWorld = q * a;
    double angularSpeed = w.norm();
    if(angularSpeed >(1e-5)){
        double angle = angularSpeed * dt;
        Eigen::Vector3d axis = w.normalized();
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
        q = (dq * q).normalized();
    }
    Eigen::Vector3d g(0,0,-9.81);
    aWorld += g;
    Eigen::Vector3d vold = velocity;
    velocity += aWorld * dt;
    position += vold * dt + 0.5*aWorld*dt*dt;
    //Im creating q as a rotation of angle and axis
    
    
}

void StateVector::initialize(const std::vector<imu_t>& imu_buffer){
    if(imu_buffer.empty()) return;
    Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    for(const imu_t& m : imu_buffer){
        w_sum += Eigen::Vector3d(m.gyro_x, m.gyro_y, m.gyro_z);
        accel_sum += Eigen::Vector3d(m.accel_x, m.accel_y, m.accel_z);

    }
    gyroBias = w_sum/imu_buffer.size();
    Eigen::Vector3d accel_avg = accel_sum/imu_buffer.size();
    Eigen::Vector3d g_world = (0.0,0.0,1.0);
    Eigen::Vector3d g = (0.0,0.0,-9.81);
    q = Eigen::Quaterniond::FromTwoVectors(accel_avg.normalized(), g_world);
    accelBias = accel_avg + q.conjugate() * g;

}