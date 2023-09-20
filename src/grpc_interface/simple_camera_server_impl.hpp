

#pragma once

#include "gen_protoc/simple_camera_service.grpc.pb.h"
#include <functional>
 //Include godot type for image


/**
 * @brief SimpleCameraServerImpl class
 * 
 * Implementation of the gRPC service for the simple camera
*/
class SimpleCameraServerImpl final : public godot_grpc::SimpleCameraService::Service
{
    /**
     * @brief getImage
     * Sends to the requester the current image generated through the camera callback
    */
    ::grpc::Status getImage(::grpc::ClientContext *context, const ::godot_grpc::simple_camera_service::imageRequestMsg &request, ::godot_grpc::simple_camera_service::imageMsg *response) override;

    ::grpc::Status getROSConfig(::grpc::ClientContext *context, const ::godot_grpc::emptyMsg &request, ::godot_grpc::ros1::ROS1PublisherConfig *response) override;

    ::grpc::Status setClientStatus(::grpc::ClientContext *context, const ::godot_grpc::uint32Msg &request, ::godot_grpc::emptyMsg *response) override;

    /**
     * Method set image callback
    */
    void set_callback(std::function<::sensors::Image ()> f_callback);

    /**
     * Method set ROS1 publisher config
    */
    void set_ros1_config(::godot_grpc::ros1::ROS1PublisherConfig f_ros1_config);

    /**
     * Method set client status callback
    */
    void set_client_status_callback(std::function<void (uint16_t)> f_callback);

private:


    /**
     * @brief ROS1 publisher config
     * Configuration set by the class user used to pass
     * to the clients when they request it
    */
    ::godot_grpc::ros1::ROS1PublisherConfig m_ros1_config;

    /**
     * @brief Callback function to set client status
     * Callback function that delegates the management of the client in function of its status
    */
    std::function<void (uint16_t)> m_set_client_status_callback;

    /**
     * @brief Callback function to generate new image from godot camera
    */
    
    std::function<:: ()> m_gen_image_callback;

};