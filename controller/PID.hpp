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
    double saturationMin;
    double saturationMax;

public:
    PID(double Kp, double Ki, double Kd, double saturationMax, double saturationMin) : Kp{Kp}, Ki{Ki}, Kd{Kd}, sum{0}, diff{0}, prev{0}, saturationMin{saturationMin}, saturationMax{saturationMax} {};
    double compute(double setpoint, double current);
};