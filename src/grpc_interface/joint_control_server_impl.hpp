#pragma once

#include "grpc_interface/gen_protoc/joint_control_service.grpc.pb.h"
#include <server_thread_manager.hpp>


typedef ::godot_grpc::joint_control_service::jointVaulesMsg t_joint_values;

class ControlJointServerImpl final : public ::godot_grpc::joint_control_service::JointControlService::Service, public CServerThreadManager
{

public:

    void init();

    ::grpc::Status readJointSensorData(::grpc::ServerContext *context, const ::godot_grpc::emptyMsg *request, ::godot_grpc::joint_control_service::jointVaulesMsg *response) override;

    ::grpc::Status writeJointCommand(::grpc::ServerContext *context, const ::godot_grpc::joint_control_service::jointVaulesMsg *request, ::godot_grpc::emptyMsg *response) override;

    //Get commands
    t_joint_values get_joint_commands();

    //Set sensor data
    void set_joint_sensor_data(t_joint_values const &f_sensor_data);

 

protected:
    void run_server() override;

private:

    t_joint_values m_commands;

    t_joint_values m_sensor_data;

    std::shared_mutex m_data_mtx;



    // t_set_client_status_callback m_set_client_status_callback;
}