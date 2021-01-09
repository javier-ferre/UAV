# IRIS UAV SIMULATOR
* To use this plugin, first clone the repository, and after create a build directory on the project root. Then switch to the build directory and run 'cmake ..' and 'make'
* Always run this command before running the simulation, on the build directory: export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
* To run the simulation, run 'gazebo --verbose ../worlds/custom.world'