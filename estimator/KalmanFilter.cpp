#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(boost::qvm::mat<float,6,1>& firstMeasurement)
{
    boost::qvm::set_identity(A);
    boost::qvm::set_identity(A);
    boost::qvm::A14(A) = deltaT;
    boost::qvm::A25(A) = deltaT;

    boost::qvm::set_zero(B);

    boost::qvm::set_identity(C);

    boost::qvm::A00(mu) = boost::qvm::A00(firstMeasurement);
    boost::qvm::A10(mu) = boost::qvm::A10(firstMeasurement);
    boost::qvm::A20(mu) = boost::qvm::A20(firstMeasurement);
    boost::qvm::A30(mu) = boost::qvm::A30(firstMeasurement);
    boost::qvm::A40(mu) = boost::qvm::A40(firstMeasurement);
    boost::qvm::A50(mu) = boost::qvm::A50(firstMeasurement);

    boost::qvm::set_zero(sigma);
    boost::qvm::A00(sigma) = 1;
    boost::qvm::A11(sigma) = 1;
    boost::qvm::A22(sigma) = 1;
    boost::qvm::A33(sigma) = 1;
    boost::qvm::A44(sigma) = 1;
    boost::qvm::A55(sigma) = 1;

    boost::qvm::set_zero(Rt);
    boost::qvm::A00(Rt) = 1;
    boost::qvm::A11(Rt) = 1;
    boost::qvm::A22(Rt) = 1;
    boost::qvm::A33(Rt) = 1;
    boost::qvm::A44(Rt) = 1;
    boost::qvm::A55(Rt) = 1;

    boost::qvm::set_zero(Qt);
    boost::qvm::A00(Qt) = 1;
    boost::qvm::A11(Qt) = 1;
    boost::qvm::A22(Qt) = 1;
    boost::qvm::A33(Qt) = 1;
    boost::qvm::A44(Qt) = 1;
    boost::qvm::A55(Qt) = 1;
}

void KalmanFilter::compute(boost::qvm::mat<float,3,1>& control, boost::qvm::mat<float,6,1>& measurement, float elapsedTime, float roll, float pitch, float yaw)
{
    boost::qvm::A14(A) = elapsedTime;
    boost::qvm::A25(A) = elapsedTime;

    boost::qvm::A30(B) = cos(pitch)*cos(yaw)+ (sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))*(boost::qvm::A10(control)) + (cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*(boost::qvm::A20(control));
    boost::qvm::A31(B) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    boost::qvm::A32(B) = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
    boost::qvm::A40(B) = cos(pitch)*sin(yaw);
    boost::qvm::A41(B) = sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw);
    boost::qvm::A42(B) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    boost::qvm::A50(B) = -sin(pitch);
    boost::qvm::A51(B) = sin(roll)*cos(pitch);
    boost::qvm::A52(B) = cos(pitch)*cos(roll);

    boost::qvm::mat<float,6,1> mu_estimated = ( A*mu )+( B*control );
    boost::qvm::mat<float,6,6> Atr = boost::qvm::transposed(A);
    boost::qvm::mat<float,6,6> sigma_estimated = ( A*sigma*Atr ) + Qt;
    boost::qvm::mat<float,6,6> Ctr = boost::qvm::transposed(C);

    boost::qvm::mat<float,6,6> Kt = sigma_estimated * Ctr * (boost::qvm::inverse( ( C*sigma_estimated*Ctr ) + Rt));
    
    mu = mu_estimated + Kt*( measurement - (C*mu_estimated) );
    sigma = (boost::qvm::identity_mat<float,6>() - (Kt*C))*sigma_estimated;
}

float KalmanFilter::getEstimatedX()
{
    return boost::qvm::A00(mu);
}

float KalmanFilter::getEstimatedY()
{
    return boost::qvm::A10(mu);
}

float KalmanFilter::getEstimatedZ()
{
    return boost::qvm::A20(mu);
}

float KalmanFilter::getEstimatedVx()
{
    return boost::qvm::A30(mu);
}

float KalmanFilter::getEstimatedVy()
{
    return boost::qvm::A40(mu);
}

float KalmanFilter::getEstimatedVz()
{
    return boost::qvm::A50(mu);
}