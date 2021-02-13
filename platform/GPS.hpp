#pragma once
#include "Platform.hpp"

class GPS
{
public:
    GPS();
    double getLatitude();
    double getLongitude();
    double getAltitude();
};

extern gazebo::transport::NodePtr node;