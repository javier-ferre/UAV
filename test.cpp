#include "./platform/GPS.hpp"
#include "./platform/Motors.hpp"
#include "./platform/IMU.hpp"
#include "./controller/PID.hpp"

#include <iostream>

const double Kp = 25;
const double Ki = 0.008;
const double Kd = 0;

const double targetAltitude = 2;

int main(int _argc, char **_argv)
{
  Platform platform(_argc,_argv);
  GPS gpsSensor;
  IMU imuSensor;
  Motors motors;

  double throttle = 0;
  double setpoint = 3;

  PID rollPid(Kp, Ki, Kd);
  PID pitchPid(Kp, Ki, Kd);
  PID yawPid(Kp, Ki, Kd);

  PID altitudePid(Kp, Ki, Kd);

  while (true)
  {
    std::cout << "Current altitude = " << gpsSensor.getAltitude() << std::endl;
    throttle = 350 + altitudePid.compute(setpoint, gpsSensor.getAltitude());
    motors.setMotorBackLeft(throttle);
    motors.setMotorBackRight(throttle);
    motors.setMotorFrontLeft(throttle);
    motors.setMotorFrontRight(throttle);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}