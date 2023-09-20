
#include "simple_camera_server_impl.hpp"

// Define methods
::grpc::Status SimpleCameraServerImpl::getImage(::grpc::ClientContext *context, const ::godot_grpc::simple_camera_service::imageRequestMsg &request, ::godot_grpc::simple_camera_service::imageMsg *response)
{
    

    
    return ::grpc::Status::OK;
}

::grpc::Status SimpleCameraServerImpl::getROSConfig(::grpc::ClientContext *context, const ::godot_grpc::emptyMsg &request, ::godot_grpc::ros1::ROS1PublisherConfig *response)
{
    // std::cout << "SimpleCameraServerImpl::getROSConfig" << std::endl;

    // Set ROS1 publisher config to gRPC response
    *response = m_ros1_config;

    return ::grpc::Status::OK;
}

::grpc::Status SimpleCameraServerImpl::setClientStatus(::grpc::ClientContext *context, const ::godot_grpc::uint32Msg &request, ::godot_grpc::emptyMsg *response)
{

    m_set_client_status_callback(request.value());
    return ::grpc::Status::OK;
}


void SimpleCameraServerImpl::set_callback(std::function<::sensors::Image()> f_callback)
{
    m_gen_image_callback = f_callback;
}


void SimpleCameraServerImpl::set_ros1_config(::godot_grpc::ros1::ROS1PublisherConfig f_ros1_config)
{
    m_ros1_config = f_ros1_config;
}

void SimpleCameraServerImpl::set_client_status_callback(std::function<void(uint16_t)> f_callback)
{
    m_set_client_status_callback = f_callback;
}
