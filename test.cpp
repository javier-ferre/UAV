#include "./platform/GPS.hpp"
#include "./platform/Motors.hpp"
#include "./platform/IMU.hpp"
#include "./controller/PositionController.hpp"
#include "./controller/AttitudeController.hpp"
#include "./estimator/KalmanFilter.hpp"
#include "./estimator/AttitudeEstimator.cpp"

#include <chrono>
#include <iostream>

int main(int _argc, char **_argv)
{
  Platform platform(_argc,_argv);
  GPS gpsSensor;
  IMU imuSensor;
  Motors motors;

  double minThrottle = 350;
  double throttle = 0;

  double xSetpoint = 3;
  double ySetpoint = 5;
  double zSetpoint = 3;

  double rollSetpoint = 0;
  double pitchSetpoint = 0;
  double yawSetpoint = 0;

  double throttleBackLeft = 0;
  double throttleBackRight = 0;
  double throttleFrontLeft = 0;
  double throttleFrontRight = 0;

  const double RAD2DEG = 180/3.14159;

  AttitudeEstimator attitudeEstimator;
  PositionController positionController;
  AttitudeController attitudeController;

  boost::qvm::mat<float,6,1> gpsMeasurements;
  boost::qvm::mat<float,3,1> imuMeasurements;

  boost::qvm::A00(gpsMeasurements) = gpsSensor.getX();
  boost::qvm::A10(gpsMeasurements) = gpsSensor.getY();
  boost::qvm::A20(gpsMeasurements) = gpsSensor.getZ();
  boost::qvm::A30(gpsMeasurements) = gpsSensor.getVelocityEast();
  boost::qvm::A40(gpsMeasurements) = gpsSensor.getVelocityNorth();
  boost::qvm::A50(gpsMeasurements) = gpsSensor.getVelocityUp();

  KalmanFilter EKF(gpsMeasurements);
  auto end = std::chrono::high_resolution_clock::now();
  auto begin = std::chrono::high_resolution_clock::now();

  while (true)
  {
    boost::qvm::A00(gpsMeasurements) = gpsSensor.getX();
    boost::qvm::A10(gpsMeasurements) = gpsSensor.getY();
    boost::qvm::A20(gpsMeasurements) = gpsSensor.getZ();
    boost::qvm::A30(gpsMeasurements) = gpsSensor.getVelocityEast();
    boost::qvm::A40(gpsMeasurements) = gpsSensor.getVelocityNorth();
    boost::qvm::A50(gpsMeasurements) = gpsSensor.getVelocityUp();

    boost::qvm::A00(imuMeasurements) = imuSensor.getAccelerationX();
    boost::qvm::A10(imuMeasurements) = imuSensor.getAccelerationY();
    boost::qvm::A20(imuMeasurements) = imuSensor.getAccelerationZ();

    end = std::chrono::high_resolution_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
    attitudeEstimator.compute(imuSensor.getAccelerationX(),imuSensor.getAccelerationY(),imuSensor.getAccelerationZ(),imuSensor.getGyroRollRate(),imuSensor.getGyroPitchRate(),imuSensor.getGyroYawRate(), elapsedTime*0.000001);
    //EKF.compute(imuMeasurements, gpsMeasurements, elapsedTime*0.000001, imuSensor.getGyroRollAngle(), imuSensor.getGyroPitchAngle(), imuSensor.getGyroYawAngle());
    EKF.compute(imuMeasurements, gpsMeasurements, elapsedTime*0.000001, attitudeEstimator.getEstimatedRollAngle(), attitudeEstimator.getEstimatedPitchAngle(), attitudeEstimator.getEstimatedYawAngle());
    
    begin = std::chrono::high_resolution_clock::now();

    std::cout << "EKF = " << EKF.getEstimatedX() << ", " << EKF.getEstimatedY() << ", " << EKF.getEstimatedZ() << std::endl;
    std::cout << "GPS = " << gpsSensor.getX() << ", " << gpsSensor.getY() << ", " << gpsSensor.getZ() << std::endl;
    std::cout << "IMU = " << (imuSensor.getGyroRollAngle())*RAD2DEG << ", " << (imuSensor.getGyroPitchAngle())*RAD2DEG << ", " << (imuSensor.getGyroYawAngle())*RAD2DEG << std::endl;
    std::cout << "IMUe= " << (attitudeEstimator.getEstimatedRollAngle())*RAD2DEG << ", " << (attitudeEstimator.getEstimatedPitchAngle())*RAD2DEG << ", " << (attitudeEstimator.getEstimatedYawAngle())*RAD2DEG << std::endl;
    std::cout << "Elapsed: " << elapsedTime*0.000001 << " seconds" << std::endl;

    positionController.compute(xSetpoint, ySetpoint, zSetpoint, EKF.getEstimatedX(), EKF.getEstimatedY(), EKF.getEstimatedZ(), rollSetpoint, pitchSetpoint, yawSetpoint, throttle);
    attitudeController.compute(rollSetpoint, pitchSetpoint, yawSetpoint, (imuSensor.getGyroRollAngle())*RAD2DEG, (imuSensor.getGyroPitchAngle())*RAD2DEG, (imuSensor.getGyroYawAngle())*RAD2DEG, throttleBackLeft, throttleBackRight, throttleFrontLeft, throttleFrontRight);

    motors.setMotorBackLeft(minThrottle+throttle+throttleBackLeft);
    motors.setMotorBackRight(minThrottle+throttle+throttleBackRight);
    motors.setMotorFrontLeft(minThrottle+throttle+throttleFrontLeft);
    motors.setMotorFrontRight(minThrottle+throttle+throttleFrontRight);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}