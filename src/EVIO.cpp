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
    Eigen::Vector4d q0(q.x(),q.y(),q.z(),q.w()); 
    Eigen::Vector3d v0 = velocity;
    Eigen::Vector3d p0 = position;
    // k1 at t
    Eigen::Quaterniond w_quaternion(0.0,w.x(),w.y(),w.z());
    Eigen::Quaterniond dq = q * w_quaternion;
    Eigen::Vector4d q_k1 = 0.5 * Eigen::Vector4d(dq.x(),dq.y(),dq.z(),dq.w());
    //k1 velocity
    Eigen::Vector3d v_k1 = (q*a)+g;
    //k1 position
    Eigen::Vector3d p_k1 = v0;

    //k2 at dt/2
    Eigen::Vector4d q_k2_vector = (q0+ 0.5*dt*q_k1).normalized();
    // Eigen Quaterniond cant be scalered so it has to get converted from 4d vector
    //also eigen::quaterniond starts with (w,x,y,z)
    Eigen::Quaterniond q2(q_k2_vector(3), q_k2_vector(0), q_k2_vector(1),q_k2_vector(2));
    Eigen::Vector3d v2 = v0 + 0.5*dt*v_k1;
    
    Eigen::Quaterniond dq2 = q2 * w_quaternion;
    Eigen::Vector4d q_k2 = 0.5 * Eigen::Vector4d(dq2.x(), dq2.y(), dq2.z(), dq2.w());

    Eigen::Vector3d v_k2 = (q2 * a) + g;

    Eigen::Vector3d p_k2 = v2;
    //k3 qt dt/2
    Eigen::Vector4d q_k3_vector = (q0+ 0.5*dt*q_k2).normalized();
    
    Eigen::Quaterniond q3(q_k3_vector(3), q_k3_vector(0), q_k3_vector(1),q_k3_vector(2));
    Eigen::Vector3d v3 = v0 + 0.5*dt*v_k2;
    
    Eigen::Quaterniond dq3 = q3 * w_quaternion;
    Eigen::Vector4d q_k3 = 0.5 * Eigen::Vector4d(dq3.x(), dq3.y(), dq3.z(), dq3.w());

    Eigen::Vector3d v_k3 = (q3 * a) + g;

    Eigen::Vector3d p_k3 = v3;

    //k4 at dt
    Eigen::Vector4d q_k4_vector = (q0+ dt*q_k3).normalized();
    
    Eigen::Quaterniond q4(q_k4_vector(3), q_k4_vector(0), q_k4_vector(1),q_k4_vector(2));
    Eigen::Vector3d v4 = v0 + dt*v_k3;
    
    Eigen::Quaterniond dq4 = q4 * w_quaternion;
    Eigen::Vector4d q_k4 = 0.5 * Eigen::Vector4d(dq4.x(), dq4.y(), dq4.z(), dq4.w());

    Eigen::Vector3d v_k4 = (q4 * a) + g;

    Eigen::Vector3d p_k4 = v4;
    // predicting next
    Eigen::Vector4d q_next = (q0 +(dt/6.0) * (q_k1 + 2 * q_k2 + 2 * q_k3 + q_k4)).normalized();
    q = Eigen::Quaterniond(q_next(3), q_next(0), q_next(1), q_next(2));
    velocity = ( v0 + (dt/6.0) * (v_k1 + 2*v_k2 + 2*v_k3 + v_k4));
    position = (p0 + (dt/6.0) * (p_k1 + 2*p_k2 + 2*p_k3 + p_k4));
/*
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
    
    */
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
static StateVector::Eigen::Vector4d quad_derivative(const Eigen::Quatterniond& q, const Eigen::Vector3d* w){
    Eigen::Quaterniond w_Quaterniond(0,w.x(),w.y(),w,z());
    Eigen::Quaterniond dq = w_Quaterniond * q;
    return 0.5* Eigen::Vector4d(dq.x(),dq.y(),dq.z(),dq.w());
}
