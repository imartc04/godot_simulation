
#include "hinge_joint3D_controller.hpp"

using namespace godot;

// Set config
void CHingeJoint3DController::set_config(CHingeJoint3DControllerConfig &&f_config)
{
    m_config = std::move(f_config);
}

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

    // Obtain controller command
    auto l_cmd = m_config.ros1_control_if->get_joint_commands()[1];

    // Obtain joint velocity value
    float l_joint_vel = this->get_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY);

    // Set joint velocity with command
    this->set_param(HingeJoint3D::PARAM_MOTOR_TARGET_VELOCITY, l_cmd);

    // Update controller velocity
    m_config.ros1_control_if->set_new_joint_reads({0.f, l_joint_vel, 0.f});
}

// // Godot process
// void _process(float delta)
// {

// }
