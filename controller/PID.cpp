#include "PID.hpp"

double PID::compute(double setpoint, double current)
{
    diff = setpoint - current;
    sum += diff;
    double result = (Kp*diff+Ki*sum+Kd*(diff-prev));
    if (result > saturationMax) return saturationMax;
    if (result < saturationMin) return saturationMin;
    return result;
}