
#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grpc_interface/gen_protoc/joint_control_service.grpc.pb.h"
#include <functional>


struct CGRPCJointControlToStringConfig
{
    /**
     * Function to be called when gen_data is called  to obtain the jointVaulesMsg
     * to be parsed into string format
    */
    std::function<void (::godot_grpc::jointControlService::jointVaulesMsg & )> gen_img;
}


/**
 * Class to manage obtain and parsing processes 
 * of camera images in the isolated ROS1 library
 * 
 * As ROS types cannot be exposed outside the shared library
 * the grpc type of image is used as interface. 
*/
class CGRPCJointControlToString
{

public:
    CGRPCJointControlToString();
    ~CGRPCJointControlToString();

    void parse_joint_msg_to_string(const ::godot_grpc::jointControlService::jointVaulesMsg &grpc_msg, ::std_msgs::String &ros1_msg);

    void gen_data(::std_msgs::String &f_ros1_msg);

    //set configuration
    void set_config(CGRPCJointControlToStringConfig const &f_config);

private:

    CGRPCJointControlToStringConfig m_config;

    ::godot_grpc::jointControlService::jointVaulesMsg m_msg;
};