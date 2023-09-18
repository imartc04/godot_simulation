
#include "ros1_vel_scalar_hw_if.hpp"

using namespace godot;

// Implement init
void CRos1VelocityHWinterface::init()
{
    // connect and register the joint state interface

    hardware_interface::JointStateHandle state_handle_a(m_config.control_var_name, &pos, &vel, &eff);
    jnt_state_interface.registerHandle(state_handle_a);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(m_config.control_var_name), &cmd);
    m_vel_joint_if.registerHandle(vel_handle);

    registerInterface(&m_vel_joint_if);
}

void CRos1VelocityHWinterface::set_config(CRos1VelocityHWinterfaceConfig const &f_config)
{
    m_config = f_config;
}

// Get config
CRos1VelocityHWinterfaceConfig &CRos1VelocityHWinterface::get_config()
{
    return m_config;
}

void CRos1VelocityHWinterface::set_new_joint_reads(std::vector<double> const & f_vaues)
{
    vel = f_vaues[1];
}

std::vector<double> CRos1VelocityHWinterface::get_joint_commands() const
{
    return {0.f, cmd};
}



