# DDS UAV project

* To use this plugin, first clone the repository, and after create a build directory on the project root. Then switch to the build directory and run these commands:

        cmake ..
        make

* Always run this command before running the simulation, on the build directory:

        cd build/
        export  GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH

* To run the simulation, run from the build directory the following:
 
        gazebo --verbose ../worlds/modified.world

