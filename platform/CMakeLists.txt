add_library(platform Platform.hpp Platform.cpp)
add_library(gps GPS.hpp GPS.cpp)
add_library(imu IMU.hpp IMU.cpp)
add_library(motors Motors.hpp Motors.cpp)

target_link_libraries(gps PUBLIC platform)
target_link_libraries(imu PUBLIC platform)
target_link_libraries(motors PUBLIC platform)
target_link_libraries(platform ${GAZEBO_LIBRARIES})