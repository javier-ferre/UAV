#pragma once
#include "PID.hpp"
#include <iostream>

class AttitudeController
{
private:
    const double rollKp = 10;
    const double rollKi = 0;
    const double rollKd = 0;
    const double rollSatMax = 50;
    const double rollSatMin = -50;

    const double pitchKp = 10;
    const double pitchKi = 0;
    const double pitchKd = 0;
    const double pitchSatMax = 50;
    const double pitchSatMin = -50;

    const double yawKp = 1;
    const double yawKi = 0.001;
    const double yawKd = 0;
    const double yawSatMax = 50;
    const double yawSatMin = -50;

    PID rollPid{rollKp, rollKi, rollKd, rollSatMax, rollSatMin};
    PID pitchPid{pitchKp, pitchKi, pitchKd, pitchSatMax, pitchSatMin};
    PID yawPid{yawKp, yawKi, yawKd, yawSatMax, yawSatMin};

public:
    void compute(double rollSetpoint, double pitchSetpoint, double yawSetpoint, double roll, double pitch, double yaw, double& motorBackLeft, double& motorBackRight, double& motorFrontLeft, double& motorFrontRight);
};