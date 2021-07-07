#include "ComplementaryFilter.hpp"

double ComplementaryFilter::compute(double input1, double input2, double deltaT)
{
    double result = (input1*tau/(tau+deltaT))+(input2*deltaT/(tau+deltaT));
    return result;
}