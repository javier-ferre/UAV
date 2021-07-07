#include "AttitudeEstimator.hpp"

void AttitudeEstimator::compute(double rawAccX, double rawAccY, double rawAccZ, double rawGyroRollRate, double rawGyroPitchRate, double rawGyroYawRate, double elapsedTime)
{
    accRoll = atan2(rawAccX,sqrt(rawAccZ*rawAccZ+rawAccY*rawAccY));
    gyroRoll = estimatedRoll + elapsedTime*rawGyroRollRate;
    gyroRoll = gyroRoll - sin(elapsedTime*rawGyroYawRate)*estimatedPitch;
    estimatedRoll = complementary.compute(gyroRoll, accRoll, elapsedTime);

    accPitch = atan2(rawAccY,sqrt(rawAccX*rawAccX+rawAccZ*rawAccZ));
    gyroPitch = estimatedPitch + elapsedTime*rawGyroPitchRate;
    gyroPitch = gyroPitch + sin(elapsedTime*rawGyroYawRate)*estimatedRoll;
    estimatedPitch = complementary.compute(gyroPitch, accPitch, elapsedTime);

    accYaw = atan2(sqrt(rawAccX*rawAccX+rawAccY*rawAccY),-rawAccZ);
    gyroYaw = estimatedYaw + elapsedTime*rawGyroYawRate;
    estimatedYaw = complementary.compute(gyroYaw, accYaw, elapsedTime);

    std::cout << "AccRoll: " << accRoll << ", GyroRoll: " << gyroRoll << std::endl;
}

double AttitudeEstimator::getEstimatedRollAngle()
{
    return estimatedRoll;
}
double AttitudeEstimator::getEstimatedPitchAngle()
{
    return estimatedPitch;
}
double AttitudeEstimator::getEstimatedYawAngle()
{
    return estimatedYaw;
}