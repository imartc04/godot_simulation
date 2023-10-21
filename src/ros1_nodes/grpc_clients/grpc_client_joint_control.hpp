#pragma once
#include "grpc_base_client.hpp"
#include <memory>
#include "grpc_interface/gen_protoc/joint_control_service.grpc.pb.h"
#include "ros_interface/ros1/ros1_pub_if.hpp"
#include "ros_interface/ros1/ros1_subscriber_if.hpp"


struct CGRPCClientJointControlConfig
{
    CGRPCBaseCilentConfig base_config;

};

class CGRPCClientJointControl
{

public:
    CGRPCClientJointControl(/* args */);
    ~CGRPCClientJointControl();

    void init();

    void set_config(CGRPCClientJointControlConfig const &f_config);

    // get ros1_pub
    std::shared_ptr<CRos1PublisherInterface> get_ros1_pub();

private:
    void set_joint_commands(::godot_grpc::joint_control_service::jointVaulesMsg &f_joint_control);

    void get_joint_reads(::godot_grpc::joint_control_service::jointVaulesMsg &f_joint_reads);

    //ros1 publisher
    std::shared_ptr<CRos1PublisherInterface> m_ros1_pub;

    //ros 1 subscriber
    std::shared_ptr<CROS1SubscriberIf> m_ros1_sub;

    //grpc stub
    std::unique_ptr<::godot_grpc::simple_camera_service::ImageService::Stub> m_stub;

    // Config
    CGRPCClientJointControlConfig m_config;
};
