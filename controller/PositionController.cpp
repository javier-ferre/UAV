#include "PositionController.hpp"

void PositionController::compute(double xSetpoint, double ySetpoint, double zSetpoint, double x, double y, double z, double& rollSetpoint, double& pitchSetpoint, double& yawSetpoint, double& throttle)
{
    rollSetpoint = yPid.compute(-ySetpoint,y);
    pitchSetpoint = xPid.compute(-xSetpoint,x);
    yawSetpoint = 0;
    throttle = zPid.compute(zSetpoint,z);
}
