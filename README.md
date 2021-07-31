# UAV project

## Introduction
This project is a (very) simple implementation of a quadrotor-type UAV software abstration layer, as well as algorithms for navigation and control. The Gazebo simulator is used for validation and testing. 

## Structure
The project is structured using different folders:
* platform: This folder holds the platform-abstracting code, which at the moment only supports the Gazebo simulator. It has GPS, IMU, and Motors modules that abstract the main sensors and actuators needed for the platform.
* worlds: This folder holds the Gazebo world file to represent the drone, based on the default Iris model created by the OSRF.
* estimator: This folder holds the implementation of a few useful algorithms for quadrotor navigation and estimation, such as a simple Kalman filter that expects GPS and IMU readings, and a Complementary Filter to process IMU data.
* controller: This folder holds the implmentation of simple control algorithms for the quadrotor, based on cascaded PID control (one loop for attitude control, and another for x,y,z position control).
* build: This folder contains the build artifacts created using CMake.

Also, on the project root there is a custom_plugin.cc file, that implements a simple Gazebo plugin to add to the model, thereby making the sensor readings visible for the platform modules. The publisher.cc file is there to debug the plugin itself. The test.cpp file is a simple program that uses all the modules described previously, integrating them into a full flight control program.

## User Guide
* To use this plugin, first clone the repository, and after create a build directory on the project root. Then switch to the build directory and run these commands:

        cmake ..
        make

* Always run this command before running the simulation, on the build directory:

        cd build/
        export  GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH

* To run the simulation, run from the build directory the following:
 
        gazebo --verbose ../worlds/modified.world

* Then, you can run the test program from the build directory:

        ./test

* The quadrotor should hold the position specified in the test.cpp code
