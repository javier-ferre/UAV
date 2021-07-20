#include "Motors.hpp"

Motors::Motors()
{
    // Publish to the motor control topics
    pub_front_right = node->Advertise<gazebo::msgs::Vector3d>("~/iris/speed_command_front_right");
    pub_front_left = node->Advertise<gazebo::msgs::Vector3d>("~/iris/speed_command_front_left");
    pub_back_right = node->Advertise<gazebo::msgs::Vector3d>("~/iris/speed_command_back_right");
    pub_back_left = node->Advertise<gazebo::msgs::Vector3d>("~/iris/speed_command_back_left");
    
    // Wait for a subscriber to connect to this publisher
    //std::cout << "Waiting for connections..." << std::endl;
    pub_front_right->WaitForConnection();
    pub_front_left->WaitForConnection();
    pub_back_right->WaitForConnection();
    pub_back_left->WaitForConnection();
    //std::cout << "Done!" << std::endl;
}

void Motors::setMotorFrontLeft(double speed)
{
    // Set the motor speed in rpm
    gazebo::msgs::Set(&front_left_msg, ignition::math::Vector3d((double)speed, 0, 0));

    // Send the messages
    pub_front_left->Publish(front_left_msg);
}

void Motors::setMotorFrontRight(double speed)
{
    // Set the motor speed in rpm
    gazebo::msgs::Set(&front_right_msg, ignition::math::Vector3d((double)speed, 0, 0));

    // Send the messages
    pub_front_right->Publish(front_right_msg);
}

void Motors::setMotorBackLeft(double speed)
{
    // Set the motor speed in rpm
    gazebo::msgs::Set(&back_left_msg, ignition::math::Vector3d((double)speed, 0, 0));

    // Send the messages
    pub_back_left->Publish(back_left_msg);
}

void Motors::setMotorBackRight(double speed)
{
    // Set the motor speed in rpm
    gazebo::msgs::Set(&back_right_msg, ignition::math::Vector3d((double)speed, 0, 0));

    // Send the messages
    pub_back_right->Publish(back_right_msg);
}