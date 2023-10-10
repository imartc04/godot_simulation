
#pragma once

#include "ros/ros.h"
#include "grpc_interface/gen_protoc/simple_camera_service.pb.h"
#include <functional>


struct CGRPCImageToRos1Config
{

    std::function<void (::godot_grpc::simple_camera_service::imageMsg & )> gen_img;
}


/**
 * Class to manage obtain and parsing processes 
 * of camera images in the isolated ROS1 library
 * 
 * As ROS types cannot be exposed outside the shared library
 * the grpc type of image is used as interface. 
*/
class CGRPCImageToRos1
{

public:
    CGRPCImageToRos1();
    ~CGRPCImageToRos1();

    void parse_grpc_msg_to_ros1_msg(const ::godot_grpc::simple_camera_service::imageMsg &grpc_msg, sensor_msgs::Image &ros1_msg);

    void gen_ros1_img_data(::sensor_msgs::Image &f_ros1_msg);

private:
    uint64_t m_seq_num = 0u;

    CGRPCImageToRos1Config m_config;
};

