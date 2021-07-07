add_library(complementary ComplementaryFilter.hpp ComplementaryFilter.cpp)
add_library(kalman KalmanFilter.hpp KalmanFilter.cpp)
add_library(attitudeEstimator AttitudeEstimator.hpp AttitudeEstimator.cpp)

target_link_libraries(attitudeEstimator PUBLIC complementary)
