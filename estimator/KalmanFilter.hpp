#pragma once
#include <boost/qvm/mat.hpp>
#include <boost/qvm/mat_operations.hpp>
#include <boost/qvm/mat_access.hpp>
#include <boost/qvm/map_mat_mat.hpp>
#include <cmath>
#include <iostream>

class KalmanFilter 
{
private:
    float deltaT{1};
    boost::qvm::mat<float,6,6> A;
    boost::qvm::mat<float,6,3> B;
    boost::qvm::mat<float,6,6> C;
    boost::qvm::mat<float,6,6> Qt;
    boost::qvm::mat<float,6,6> Rt;
    boost::qvm::mat<float,6,1> mu;
    boost::qvm::mat<float,6,6> sigma;

public:
    KalmanFilter(boost::qvm::mat<float,6,1>& firstMeasurement);
    void compute(boost::qvm::mat<float,3,1>& control, boost::qvm::mat<float,6,1>& measurement, float elapsedTime, float roll, float pitch, float yaw);
    float getEstimatedX();
    float getEstimatedY();
    float getEstimatedZ();
    float getEstimatedVx();
    float getEstimatedVy();
    float getEstimatedVz();
};