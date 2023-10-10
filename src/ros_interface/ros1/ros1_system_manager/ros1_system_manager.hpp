
#pragma once
/**
 * Shared library to manage all ROS 1 interaction isolated from 
 * the rest of the system. This way we can also use ROS 2
*/

#include "ros_interface/ros1/ros1_pub.hpp"

#include "grpc_interface/gen_protoc/ros1.pb.h"
#include "grpc_interface/gen_protoc/joint_control_service.pb.h"
#include <memory>



/**
 * Extern C function to create ROS 1 publisher interface for image
*/
extern "C" std::shared_ptr<CRos1PublisherInterface> create_ros1_publisher_interface_image(::godot::CRos1PublisherConfig<::godot_grpc::simple_camera_service::imageMsg> const & f_config );

/**
 * Extern C function to create ROS 1 publisher interface for joint control
*/
extern "C" std::shared_ptr<CRos1PublisherInterface> create_ros1_publisher_interface_joint_control(::godot::CRos1PublisherConfig<::godot_grpc::joint_control_service::jointVaulesMsg> const &f_config);