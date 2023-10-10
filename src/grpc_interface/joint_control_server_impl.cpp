
#include "joint_control_server_impl.hpp"

::grpc::Status readJointSensorData(::grpc::ServerContext *context, const ::godot_grpc::emptyMsg *request, ::godot_grpc::joint_control_service::jointVaulesMsg *response)
{
    m_data_mtx.lock();
    *response = m_sensor_data;
    m_data_mtx.unlock();

    return ::grpc::Status::OK;
}

//implement init
void CJointControlServerImpl::init()
{
    CServerThreadManager::init();
}

::grpc::Status writeJointCommand(::grpc::ServerContext *context, const ::godot_grpc::joint_control_service::jointVaulesMsg *request, ::godot_grpc::emptyMsg *response)
{
    m_data_mtx.lock();
    m_commands = *request;
    m_data_mtx.unlock();

    return ::grpc::Status::OK;
}

//Get commands
::godot_grpc::joint_control_service::jointVaulesMsg CJointControlServerImpl::get_joint_commands()
{
    std::shared_lock<std::shared_mutex> lock(m_data_mtx);
    return m_commands;
}

//Set sensor data
void CJointControlServerImpl::set_joint_sensor_data(t_joint_values const &f_sensor_data)
{
    std::unique_lock<std::shared_mutex> lock(m_data_mtx);
    m_sensor_data = f_sensor_data;
}

void run_server()
{
    this->Wait();
}