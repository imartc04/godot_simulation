
#include "hinge_joint3D_controller.hpp"
#include "ros_manager/ros_manager.hpp"


using namespace godot;


CHingeJoint3DController::CHingeJoint3DController(){};

CHingeJoint3DController::~CHingeJoint3DController(){};

void CHingeJoint3DController::_bind_methods()
{
}

// Godot init
void CHingeJoint3DController::_init()
{
}

void CHingeJoint3DController::_ready()
{
}

// Godot ready
// void _ready()
// {

// }

void CHingeJoint3DController::_physics_process(float delta)
{

    if (ros_manager_initialized())
    {

        if (!m_initialized)
        {
            // Set initialized
            m_initialized = true;


            //Initialize control 
            if(m_control_type == EControlTye::ROS_CONTROL_PACKAGE)
            {



            }
            else
            {
                //ROS 1 topic based control 


            }


        }

        // Obtain controller command
        auto l_cmd = m_control_if->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel = this->get_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        m_control_if->set_new_joint_reads({0.f, l_joint_vel, 0.f});
    }
}

