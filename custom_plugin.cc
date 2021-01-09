#ifndef _CUSTOM_PLUGIN_HH_
#define _CUSTOM_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#define SPEED 300

namespace gazebo
{
    /**
     * \brief A custom plugin to control the Iris UAV
     */

    class CustomPlugin : public ModelPlugin
    {
        // Constructor
    public:
        CustomPlugin() {}

        // Load function
    public:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Just output a message for now
            std::cerr << "\nThe Custom Control plugin is attached to the model: " << _model->GetName() << "\n";

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            //this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CustomPlugin::GetPose, this));

            // Control of one of the rotors
            if(_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid Joint count, Control Plugin not loaded\n";
                return;
            }

            this->model = _model;
            this->joint1 = _model->GetJoints()[2];
            this->joint2 = _model->GetJoints()[3];
            this->joint3 = _model->GetJoints()[4];
            this->joint4 = _model->GetJoints()[5];
            this->pid1 = common::PID(0.1,0,0);
            this->pid2 = common::PID(0.1,0,0);
            this->pid3 = common::PID(0.1,0,0);
            this->pid4 = common::PID(0.1,0,0);

            this->model->GetJointController()->SetVelocityPID(this->joint1->GetScopedName(), this->pid1);
            this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), SPEED);
            this->model->GetJointController()->SetVelocityPID(this->joint2->GetScopedName(), this->pid2);
            this->model->GetJointController()->SetVelocityTarget(this->joint2->GetScopedName(), SPEED);            
            this->model->GetJointController()->SetVelocityPID(this->joint3->GetScopedName(), this->pid3);
            this->model->GetJointController()->SetVelocityTarget(this->joint3->GetScopedName(), -SPEED);            
            this->model->GetJointController()->SetVelocityPID(this->joint4->GetScopedName(), this->pid4);
            this->model->GetJointController()->SetVelocityTarget(this->joint4->GetScopedName(), -SPEED);

            // Node creation
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());
            this->world = this->model->GetWorld();

            // Topic and subscriber creation
            std::string topicName1 = "~/" + this->model->GetName() + "/speed_command_front_right";
            this->sub1 = this->node->Subscribe(topicName1, &CustomPlugin::OnMsg1, this);

            std::string topicName2 = "~/" + this->model->GetName() + "/speed_command_back_left";
            this->sub2 = this->node->Subscribe(topicName2, &CustomPlugin::OnMsg2, this);

            std::string topicName3 = "~/" + this->model->GetName() + "/speed_command_front_left";
            this->sub3 = this->node->Subscribe(topicName3, &CustomPlugin::OnMsg3, this);

            std::string topicName4 = "~/" + this->model->GetName() + "/speed_command_back_right";
            this->sub4 = this->node->Subscribe(topicName4, &CustomPlugin::OnMsg4, this);
        }
    private:
        physics::ModelPtr model;
        physics::WorldPtr world;
        physics::JointPtr joint1;
        physics::JointPtr joint2;
        physics::JointPtr joint3;
        physics::JointPtr joint4;

        common::PID pid1;
        common::PID pid2;
        common::PID pid3;
        common::PID pid4;

        transport::NodePtr node;
        transport::SubscriberPtr sub1;
        transport::SubscriberPtr sub2;
        transport::SubscriberPtr sub3;
        transport::SubscriberPtr sub4;
        // event::ConnectionPtr updateConnection;

    public:
        void SetVelocity(int motor, const double &_velocity)
        {
            switch (motor)
            {
                case 1: this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), _velocity); break;
                case 2: this->model->GetJointController()->SetVelocityTarget(this->joint2->GetScopedName(), _velocity); break;
                case 3: this->model->GetJointController()->SetVelocityTarget(this->joint3->GetScopedName(), _velocity); break;
                case 4: this->model->GetJointController()->SetVelocityTarget(this->joint4->GetScopedName(), _velocity); break;
                default: break;
            }
        }

        void GetPose()
        {
            ignition::math::Pose3d pose = model->WorldPose();
            std::cerr << "Position is: " << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() << "\n";
        }

    private:
        void OnMsg1(ConstVector3dPtr &_msg)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), _msg->x()); // Publisher will send here the speed commands
        }
        void OnMsg2(ConstVector3dPtr &_msg)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint2->GetScopedName(), _msg->x()); // Publisher will send here the speed commands
        }
        void OnMsg3(ConstVector3dPtr &_msg)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint3->GetScopedName(), -(_msg->x())); // Publisher will send here the speed commands
        }
        void OnMsg4(ConstVector3dPtr &_msg)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint4->GetScopedName(), -(_msg->x())); // Publisher will send here the speed commands
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(CustomPlugin)
} // namespace gazebo
#endif