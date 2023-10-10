
#include "joint_control_manager.hpp"

using namespace godot;

// Implement init
void CJointControlManager::init()
{

    // Create server
    m_server = std::make_unique<CJointControlServerImpl>();

    // Create gRPC server builder
    ::grpc::ServerBuilder builder;
    builder.AddListeningPort(m_config.grpc_config.server_address + ":" + m_config.grpc_config.server_port, grpc::InsecureServerCredentials());

    builder.RegisterService(m_server.get());

    m_grpc_server = builder.BuildAndStart();

    // Launch new joint interface process that will comunicate with the server
    m_process.init();

    // Launch server in its own thread
    m_server->init();
}

void CJointControlManager::set_config(CJointControlManagerConfig const &f_config)
{
    m_config = f_config;

    // Configure process manager
    m_process.config(m_config.grpc_config);
}

// Get config
CJointControlManagerConfig &CJointControlManager::get_config()
{
    return m_config;
}

void CJointControlManager::set_new_joint_reads(t_joint_values const &f_vaues)
{
    m_joint_control_server->set_joint_sensor_data(f_vaues);
}

t_joint_values CJointControlManager::get_new_commands()
{
    return std::move(m_joint_control_server->get_joint_commands());
}