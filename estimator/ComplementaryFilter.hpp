#pragma once

class ComplementaryFilter
{
    private:
        double tau;
    public:
        ComplementaryFilter(double tau) : tau{tau} {}
        double compute(double input1, double input2, double deltaT);
};