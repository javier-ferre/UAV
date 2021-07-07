#pragma once
#include "PID.hpp"
#include <iostream>

class PositionController
{
private:
    const double xKp = 10;
    const double xKi = 0;
    const double xKd = 0;
    const double xSatMax = 5;
    const double xSatMin = -5;

    const double yKp = 10;
    const double yKi = 0;
    const double yKd = 0;
    const double ySatMax = 5;
    const double ySatMin = -5;

    const double zKp = 25;
    const double zKi = 0.01;
    const double zKd = 0;
    const double zSatMax = 100;
    const double zSatMin = -100;

    PID xPid{xKp, yKi, zKd, xSatMax, xSatMin};
    PID yPid{yKp, yKi, yKd, ySatMax, ySatMin};
    PID zPid{zKp, zKi, zKd, zSatMax, zSatMin};

public:
    void compute(double xSetpoint, double ySetpoint, double zSetpoint, double x, double y, double z, double& rollSetpoint, double& pitchSetpoint, double& yawSetpoint, double& throttle);
};