#pragma once
#include "Platform.hpp"

class Motors 
{
private:
    // Create a the neccesary messages
    gazebo::msgs::Vector3d front_right_msg;
    gazebo::msgs::Vector3d front_left_msg;
    gazebo::msgs::Vector3d back_right_msg;
    gazebo::msgs::Vector3d back_left_msg;

    // Create the motor control topics
    gazebo::transport::PublisherPtr pub_front_right; 
    gazebo::transport::PublisherPtr pub_front_left;
    gazebo::transport::PublisherPtr pub_back_right;
    gazebo::transport::PublisherPtr pub_back_left;
public:
    Motors();
    void setMotorFrontLeft(double speed);
    void setMotorFrontRight(double speed);
    void setMotorBackLeft(double speed);
    void setMotorBackRight(double speed);
};

extern gazebo::transport::NodePtr node;