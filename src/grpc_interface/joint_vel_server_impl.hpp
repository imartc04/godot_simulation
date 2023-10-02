

#pragma once

#include "grpc_interface/gen_protoc/joint_vel.grpc.pb.h"

#include <functional>
#include "ros_interface/ros1/joint_controllers/ros1_control_if.hpp"
#include <vector>

#include <shared_mutex>
#include <mutex>
#include <iostream>
#include <thread>


class JointVelServerImpl final : public ::godot_grpc::joint_vel::JointVelService::Service, public ::godot::Ros1ControlIf
{

public:
    JointVelServerImpl();
    ~JointVelServerImpl();

    void set_new_joint_reads(std::vector<double> const &) override;
    std::vector<double> get_joint_commands() override;


    ::grpc::Status setJointCommand(::grpc::ServerContext *context, const ::godot_grpc::joint_vel::jointVelocitiesMsg *request, ::godot_grpc::emptyMsg *response) override;

    ::grpc::Status readJointData(::grpc::ServerContext *context, const ::godot_grpc::emptyMsg *request, ::godot_grpc::joint_vel::jointVelocitiesMsg *response) override;

private:
    /**
     * Current status of joint velocities
     */
    std::vector<double> m_joint_vels_read;

    /**
     * Current command of joint velocities
     */
    std::vector<double> m_joint_vels_cmd;

    std::shared_mutex m_mtx;
    // std::mutex m_mtx;
};