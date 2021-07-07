#include "PID.hpp"

double PID::compute(double setpoint, double current)
{
    diff = setpoint - current;
    sum += diff;
    double result = (Kp*diff+Ki*sum+Kd*(diff-prev));

    if (result > saturationMax)
    {
        sum -= diff;
        return saturationMax;
    }
    if (result < saturationMin)
    {
        sum-=diff;
        return saturationMin;
    }

    prev = diff;
    
    return result;
}