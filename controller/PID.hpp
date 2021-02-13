#pragma once

class PID
{
private:
    double Kp;
    double Ki;
    double Kd;
    double diff;
    double sum;
    double prev;

public:
    PID(double Kp, double Ki, double Kd) : Kp{Kp}, Ki{Ki}, Kd{Kd}, sum{0}, diff{0}, prev{0} {};
    double compute(double setpoint, double current);
};