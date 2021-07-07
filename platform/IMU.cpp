#include "IMU.hpp"

namespace IMUUtils
{
    double accelerationX, accelerationY, accelerationZ;
    double rollRate, pitchRate, yawRate;
    double roll, pitch, yaw;
    
    void IMUCallback(ConstIMUPtr &_msg)
    {
        accelerationX = _msg->linear_acceleration().x();
        accelerationY = _msg->linear_acceleration().y();
        accelerationZ = _msg->linear_acceleration().z();

        rollRate = _msg->angular_velocity().x();
        pitchRate = _msg->angular_velocity().y();
        yawRate = _msg->angular_velocity().z();

        gazebo::msgs::Quaternion orientation = _msg->orientation();
        ignition::math::Quaterniond quaternion = gazebo::msgs::ConvertIgn(orientation);
        roll = (180*quaternion.Euler().X())/3.14159;
        pitch = (180*quaternion.Euler().Y())/3.14159;
        yaw = (180*quaternion.Euler().Z())/3.14159;
    }

    gazebo::transport::SubscriberPtr sub;
}

IMU::IMU()
{
    IMUUtils::sub = node->Subscribe("~/iris_custom/iris/iris/imu_link/imu_sensor/imu", IMUUtils::IMUCallback);
}

double IMU::getAccelerationX()
{
    return IMUUtils::accelerationX;
}

double IMU::getAccelerationY()
{
    return IMUUtils::accelerationY;
}

double IMU::getAccelerationZ()
{
    return IMUUtils::accelerationZ;
}

double IMU::getGyroRollRate()
{
    return IMUUtils::rollRate;
}

double IMU::getGyroPitchRate()
{
    return IMUUtils::pitchRate;
}

double IMU::getGyroYawRate()
{
    return IMUUtils::yawRate;
}

double IMU::getGyroRollAngle()
{
    return IMUUtils::roll;
}

double IMU::getGyroPitchAngle()
{
    return IMUUtils::pitch;
}

double IMU::getGyroYawAngle()
{
    return IMUUtils::yaw;
}
