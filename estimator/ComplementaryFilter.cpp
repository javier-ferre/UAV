#include "ComplementaryFilter.hpp"

double ComplementaryFilter::compute(double input1, double input2)
{
    return (input1*T/(T+t))+(input2*t/(T+t));
}