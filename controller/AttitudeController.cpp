#include "AttitudeController.hpp"

void AttitudeController::compute(double rollSetpoint, double pitchSetpoint, double yawSetpoint, double roll, double pitch, double yaw, double& motorBackLeft, double& motorBackRight, double& motorFrontLeft, double& motorFrontRight)
{
    motorBackLeft = (rollPid.compute(rollSetpoint,roll)-pitchPid.compute(pitchSetpoint,pitch)+yawPid.compute(yawSetpoint,yaw));
    motorBackRight = (-rollPid.compute(rollSetpoint,roll)-pitchPid.compute(pitchSetpoint,pitch)-yawPid.compute(yawSetpoint,yaw));
    motorFrontLeft = (rollPid.compute(rollSetpoint,roll)+pitchPid.compute(pitchSetpoint,pitch)-yawPid.compute(yawSetpoint,yaw));
    motorFrontRight = (-rollPid.compute(rollSetpoint,roll)+pitchPid.compute(pitchSetpoint,pitch)+yawPid.compute(yawSetpoint,yaw));
}