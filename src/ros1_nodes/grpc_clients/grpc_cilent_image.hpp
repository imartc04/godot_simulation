#pragma once

#include "grpc_base_client.hpp"
#include <memory>
#include "grpc_interface/gen_protoc/simple_camera_service.grpc.pb.h"

struct CGRPCClientImageConfig
{
    CGRPCBaseCilentConfig base_config;

    ::godot::CRos1PublisherConfig<::godot_grpc::simple_camera_service::imageMsg> ros1_pub_config;
}

class CGRPCClientImage : public CGRPCBaseCilent
{

public:
    CGRPCClientImage(/* args */);
    ~CGRPCClientImage();

    void init();

    void set_config(CGRPCClientImageConfig const &f_config);

    // get ros1_pub
    std::shared_ptr<CRos1PublisherInterface> get_ros1_pub();

private:
    void get_image(::godot_grpc::simple_camera_service::imageMsg &f_img);

    std::unique_ptr<::godot_grpc::simple_camera_service::ImageService::Stub> m_stub;

    std::shared_ptr<CRos1PublisherInterface> m_ros1_pub;

    CGRPCClientImageConfig m_config;
};

CGRPCClientImage::CGRPCClientImage(/* args */)
{
}

CGRPCClientImage::~CGRPCClientImage()
{
}
