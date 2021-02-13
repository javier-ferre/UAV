#include "Platform.hpp"

// Global node handle for communication with Gazebo
gazebo::transport::NodePtr node(new gazebo::transport::Node());

Platform::Platform(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Initialize our node for communication
    node->Init();

}

Platform::~Platform()
{
    // Make sure to shut everything down.
    gazebo::client::shutdown();
}