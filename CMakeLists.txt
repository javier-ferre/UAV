cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find Boost
find_package(Boost 1.36.0)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})

    # Build our plugin
    add_library(custom_plugin SHARED custom_plugin.cc)
    target_link_libraries(custom_plugin ${GAZEBO_LIBRARIES})

    # Build the stand-alone test program
    # add_executable(publisher publisher.cc)
    #target_link_libraries(publisher ${GAZEBO_LIBRARIES})

    # Build the stand-alone test program
    add_executable(app test.cpp)
    target_link_libraries(app ${GAZEBO_LIBRARIES})

    add_subdirectory(platform ./platform)
    target_link_libraries(app gps motors imu)
    
    add_subdirectory(controller ./controller)

    target_link_libraries(app positionController attitudeController)

    add_library(complementary ./estimator/ComplementaryFilter.hpp ../estimator/ComplementaryFilter.cpp)
    target_link_libraries(app complementary)

    add_library(kalman ./estimator/KalmanFilter.hpp ../estimator/KalmanFilter.cpp)
    target_link_libraries(app kalman)

endif()
