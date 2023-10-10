
#include "generic6_dof_joint_controller.hpp"

using namespace godot;

CGeneric6DOFController::CGeneric6DOFController(){};

// Destructor
CGeneric6DOFController::~CGeneric6DOFController(){};

// Set config
void CGeneric6DOFController::set_config(CGeneric6DOFControllerConfig &&f_config)
{
    m_config = std::move(f_config);
}

void CGeneric6DOFController::_bind_methods()
{
}

void CGeneric6DOFController::_ready()
{
}

// Godot init
void CGeneric6DOFController::_init()
{
}

// Godot ready
// void _ready()
// {

// }

void CGeneric6DOFController::_physics_process(float delta)
{

    {
        // Get angular x controller
        auto &l_controller_ang_x = m_config.ros1_control_if[0];

        // Obtain controller command
        auto l_cmd = l_controller_ang_x->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_x(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_x(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_ang_x->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }

    {
        // Get angular y controller
        auto &l_controller_ang_y = m_config.ros1_control_if[1];

        // Obtain controller command
        auto l_cmd = l_controller_ang_y->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_y(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_y(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_ang_y->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }

    {
        // Get angular z controller
        auto &l_controller_ang_z = m_config.ros1_control_if[2];

        // Obtain controller command
        auto l_cmd = l_controller_ang_z->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_z(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_z(CGeneric6DOFController::PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_ang_z->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }

    // linear x controller
    {
        // Get linear x controller
        auto &l_controller_lin_x = m_config.ros1_control_if[3];

        // Obtain controller command
        auto l_cmd = l_controller_lin_x->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_x(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_x(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_lin_x->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }

    // linear y controller
    {
        // Get linear y controller
        auto &l_controller_lin_y = m_config.ros1_control_if[4];

        // Obtain controller command
        auto l_cmd = l_controller_lin_y->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_y(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_y(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_lin_y->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }

    // linear z controller
    {
        // Get linear z controller
        auto &l_controller_lin_z = m_config.ros1_control_if[5];

        // Obtain controller command
        auto l_cmd = l_controller_lin_z->get_joint_commands()[1];

        // Obtain joint velocity value
        float l_joint_vel_measure = this->get_param_z(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY);

        // Set joint velocity with command
        this->set_param_z(CGeneric6DOFController::PARAM_LINEAR_MOTOR_TARGET_VELOCITY, l_cmd);

        // Update controller velocity
        l_controller_lin_z->set_new_joint_reads({0.f, l_joint_vel_measure, 0.f});
    }
}

// // Godot process
// void _process(float delta)
// {

// }
