#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <csignal>
#include <gazebo/gazebo_client.hh>
#include <chrono>
#include <thread>

const unsigned int MAX_SPEED = 400;

/////////////////////////////////////////////////
sig_atomic_t signaled = 0;
void my_handler (int param)
{
  signaled = 1;
}
void cb(ConstPosesStampedPtr &_msg)
{
  gazebo::msgs::Vector3d position = _msg->pose(0).position();
  // gazebo::msgs::Vector3d orientation = _msg->orientation();
  std::cout << "The position is: " << position.x() << " " << position.y() << " " << position.z() << std::endl;
  // std::cout << "The orientation is: " << orientation.x() << " " << orientation().y() << " " << orientation().z() << std::endl;
}
/////////////////////////////////////////////////

int main(int _argc, char **_argv)
{
    void (*prev_handler)(int);
    prev_handler = signal(SIGINT, my_handler);

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the motor control topics
    gazebo::transport::PublisherPtr pub_front_right = node->Advertise<gazebo::msgs::Vector3d>("~/iris_custom/speed_command_front_right");
    gazebo::transport::PublisherPtr pub_front_left = node->Advertise<gazebo::msgs::Vector3d>("~/iris_custom/speed_command_front_left");
    gazebo::transport::PublisherPtr pub_back_right = node->Advertise<gazebo::msgs::Vector3d>("~/iris_custom/speed_command_back_right");
    gazebo::transport::PublisherPtr pub_back_left = node->Advertise<gazebo::msgs::Vector3d>("~/iris_custom/speed_command_back_left");
    
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", cb);

    // Wait for a subscriber to connect to this publisher
    std::cout << "Waiting for connections..." << std::endl;
    pub_front_right->WaitForConnection();
    pub_front_left->WaitForConnection();
    pub_back_right->WaitForConnection();
    pub_back_left->WaitForConnection();
    std::cout << "Done!" << std::endl;

    // Create a a vector3 message
    gazebo::msgs::Vector3d front_right_msg;
    gazebo::msgs::Vector3d front_left_msg;
    gazebo::msgs::Vector3d back_right_msg;
    gazebo::msgs::Vector3d back_left_msg;
    unsigned int count = 0;
    bool flag = false;

    while (signaled == 0)
    {
      if ((count < MAX_SPEED) && (flag == false))
      {
          count++;
      }
      else
      {
          if ((flag == false) && (count >= MAX_SPEED)) flag = true;
          else count--;

          if (count == 0) flag = false;
      }
      // Set the motor speeds in rpm
      gazebo::msgs::Set(&front_right_msg, ignition::math::Vector3d((double)count, 0, 0));
      gazebo::msgs::Set(&front_left_msg, ignition::math::Vector3d((double)count, 0, 0));
      gazebo::msgs::Set(&back_right_msg, ignition::math::Vector3d((double)count, 0, 0));
      gazebo::msgs::Set(&back_left_msg, ignition::math::Vector3d((double)count, 0, 0));

      // Send the messages
      pub_front_right->Publish(front_right_msg);
      pub_front_left->Publish(front_left_msg);
      pub_back_right->Publish(back_right_msg);
      pub_back_left->Publish(back_left_msg);

      // Sleep for 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();

    std::cout << "El valor es: " << signaled << std::endl;
    return 0;

}