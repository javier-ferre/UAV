#pragma once
#include "Platform.hpp"
#include <cmath>

class GPS
{
public:
    GPS();
    double getLatitude();
    double getLongitude();
    double getAltitude();
    double getVelocityEast();
    double getVelocityNorth();
    double getVelocityUp();
    double getX();
    double getY();
    double getZ();
};

extern gazebo::transport::NodePtr node;