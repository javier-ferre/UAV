#pragma once
#include "ComplementaryFilter.hpp"
#include <cmath>
#include <iostream>

class AttitudeEstimator
{
private:
    const double tau {1};

    double accRoll{0};
    double accPitch{0};
    double accYaw{0};
    double gyroRoll{0};
    double gyroPitch{0};
    double gyroYaw{0};
    double estimatedRoll{0};
    double estimatedPitch{0};
    double estimatedYaw{0};
    ComplementaryFilter complementary{tau};

public:
    void compute(double rawAccX, double rawAccY, double rawAccZ, double rawGyroRollRate, double rawGyroPitchRate, double rawGyroYawRate, double elapsedTime);
    double getEstimatedRollAngle();
    double getEstimatedPitchAngle();
    double getEstimatedYawAngle();
};