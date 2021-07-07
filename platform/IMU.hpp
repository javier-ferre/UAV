#pragma once
#include "./Platform.hpp"

class IMU
{
private:

public:
    IMU();
    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();
    double getGyroPitchRate();
    double getGyroRollRate();
    double getGyroYawRate();
    double getGyroRollAngle();
    double getGyroPitchAngle();
    double getGyroYawAngle();
};

extern gazebo::transport::NodePtr node;