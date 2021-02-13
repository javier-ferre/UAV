#pragma once
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <csignal>
#include <gazebo/gazebo_client.hh>
#include <chrono>
#include <thread>

class Platform
{
private:
public:
    Platform(int _argc, char **_argv);
    ~Platform();
};