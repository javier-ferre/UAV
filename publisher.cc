#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <csignal>
#include <gazebo/gazebo_client.hh>

/////////////////////////////////////////////////
sig_atomic_t signaled = 0;
void my_handler (int param)
{
  signaled = 1;
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

    // Publish to the control topic
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/iris_custom/speed_command_front_right");

    // Wait for a subscriber to connect to this publisher
    std::cout << "Waiting for connection..." << std::endl;
    pub->WaitForConnection();
    std::cout << "Done!" << std::endl;

    // Create a a vector3 message
    gazebo::msgs::Vector3d msg;

    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));

    // Send the message
    pub->Publish(msg);

    // Make sure to shut everything down.
    gazebo::client::shutdown();

    raise(SIGINT);
    std::cout << "El valor es: " << signaled << std::endl;
    return 0;

}