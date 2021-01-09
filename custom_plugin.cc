#ifndef _CUSTOM_PLUGIN_HH_
#define _CUSTOM_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#define SPEED 400

namespace gazebo
{
    /**
     * \brief A custom pluging to control the Iris UAV
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
            std::cerr << "\nThe Custom Control plugin is attached to the model" << _model->GetName() << "]\n";

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CustomPlugin::GetPose, this));

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
            std::string topicName = "~/" + this->model->GetName() + "/speed_command";
            this->sub = this->node->Subscribe(topicName, &CustomPlugin::OnMsg, this);
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
        transport::SubscriberPtr sub;

        event::ConnectionPtr updateConnection;

    public:
        void SetVelocity(const double &_velocity)
        {
            this->model->GetJointController()->SetVelocityTarget(this->joint1->GetScopedName(), _velocity);
        }

        void GetPose()
        {
            ignition::math::Pose3d pose = model->WorldPose();
            std::cerr << "Position is: " << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() << "\n";
        }

    private:
        void OnMsg(ConstVector3dPtr &_msg)
        {
            this->SetVelocity(_msg->x()); // Publisher will send here the speed commands
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(CustomPlugin)
} // namespace gazebo
#endif