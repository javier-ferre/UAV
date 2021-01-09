cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(custom_plugin SHARED custom_plugin.cc)
target_link_libraries(custom_plugin ${GAZEBO_LIBRARIES})

# Build the stand-alone test program
add_executable(publisher publisher.cc)
target_link_libraries(publisher ${GAZEBO_LIBRARIES})