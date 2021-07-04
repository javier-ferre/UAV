#pragma once

class ComplementaryFilter
{
    private:
        double t;
        double T;
    public:
        ComplementaryFilter(double tau, double deltat) : t{tau}, T{deltat} {}
        double compute(double input1, double input2);
};