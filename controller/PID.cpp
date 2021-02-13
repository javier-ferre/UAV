#include "PID.hpp"

double PID::compute(double setpoint, double current)
{
    diff = setpoint - current;
    sum += diff;

    return (Kp*diff+Ki*sum+Kd*(diff-prev));
}