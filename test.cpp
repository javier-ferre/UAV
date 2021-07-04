#include "./platform/GPS.hpp"
#include "./platform/Motors.hpp"
#include "./platform/IMU.hpp"
#include "./controller/PID.hpp"
#include "./estimator/KalmanFilter.hpp"

#include <chrono>
#include <iostream>

const double zKp = 25;
const double zKi = 0.008;
const double zKd = 0;
const double zSatMax = 50;
const double zSatMin = -50;

const double rollKp = 0.1;
const double rollKi = 0;
const double rollKd = 0;
const double rollSatMax = 10;
const double rollSatMin = -10;

const double pitchKp = 0.1;
const double pitchKi = 0;
const double pitchKd = 0;
const double pitchSatMax = 10;
const double pitchSatMin = -10;

const double yawKp = 0.03;
const double yawKi = 0.000002;
const double yawKd = 0;
const double yawSatMax = 10;
const double yawSatMin = -10;

int main(int _argc, char **_argv)
{
  Platform platform(_argc,_argv);
  GPS gpsSensor;
  IMU imuSensor;
  Motors motors;

  double throttle = 0;
  double setpoint = 3;
  double pitchSetpoint = 0;
  double rollSetpoint = 0;
  double yawSetpoint = 0;

  PID rollPid(rollKp, rollKi, rollKd, rollSatMax, rollSatMin);
  PID pitchPid(pitchKp, pitchKi, pitchKd, pitchSatMax, pitchSatMin);
  PID yawPid(yawKp, yawKi, yawKd, yawSatMax, yawSatMin);

  PID altitudePid(zKp, zKi, zKd, zSatMax, zSatMin);

  boost::qvm::mat<float,6,1> gpsMeasurements;
  boost::qvm::mat<float,3,1> imuMeasurements;

  boost::qvm::A00(gpsMeasurements) = gpsSensor.getX();
  boost::qvm::A10(gpsMeasurements) = gpsSensor.getY();
  boost::qvm::A20(gpsMeasurements) = gpsSensor.getZ();
  boost::qvm::A30(gpsMeasurements) = gpsSensor.getVelocityEast();
  boost::qvm::A40(gpsMeasurements) = gpsSensor.getVelocityNorth();
  boost::qvm::A50(gpsMeasurements) = gpsSensor.getVelocityUp();

  KalmanFilter uavEKF(gpsMeasurements);
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
    uavEKF.compute(imuMeasurements, gpsMeasurements, elapsedTime*0.000001, (imuSensor.getGyroRollAngle())*3.14159/180, (imuSensor.getGyroPitchAngle())*3.14159/180, (imuSensor.getGyroYawAngle())*3.14159/180);
    begin = std::chrono::high_resolution_clock::now();

    // std::cout << "Current altitude = " << gpsSensor.getAltitude() << std::endl;
    // std::cout << "Current Yaw = " << imuSensor.getGyroYawAngle() << std::endl;
    std::cout << "Esimated = " << uavEKF.getEstimatedX() << ", " << uavEKF.getEstimatedY() << ", " << uavEKF.getEstimatedZ() << std::endl;
    std::cout << "  True   = " << gpsSensor.getX() << ", " << gpsSensor.getY() << ", " << gpsSensor.getZ() << std::endl;

    throttle = 350 + altitudePid.compute(setpoint, gpsSensor.getAltitude());
    motors.setMotorBackLeft(throttle+rollPid.compute(rollSetpoint,imuSensor.getGyroRollAngle())-pitchPid.compute(pitchSetpoint,imuSensor.getGyroPitchAngle())+yawPid.compute(yawSetpoint,imuSensor.getGyroYawAngle()));
    motors.setMotorBackRight(throttle-rollPid.compute(rollSetpoint,imuSensor.getGyroRollAngle())-pitchPid.compute(pitchSetpoint,imuSensor.getGyroPitchAngle())-yawPid.compute(yawSetpoint,imuSensor.getGyroYawAngle()));
    motors.setMotorFrontLeft(throttle+rollPid.compute(rollSetpoint,imuSensor.getGyroRollAngle())+pitchPid.compute(pitchSetpoint,imuSensor.getGyroPitchAngle())-yawPid.compute(yawSetpoint,imuSensor.getGyroYawAngle()));
    motors.setMotorFrontRight(throttle-rollPid.compute(rollSetpoint,imuSensor.getGyroRollAngle())+pitchPid.compute(pitchSetpoint,imuSensor.getGyroPitchAngle())+yawPid.compute(yawSetpoint,imuSensor.getGyroYawAngle()));

    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}