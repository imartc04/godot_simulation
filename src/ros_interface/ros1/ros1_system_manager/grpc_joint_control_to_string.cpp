
#include "grpc_joint_control_to_string.hpp"


//constructor
CGRPCJointControlToString::CGRPCJointControlToString()
{
}

//destructor
CGRPCJointControlToString::~CGRPCJointControlToString()
{
}

//parse joint control message to string
void CGRPCJointControlToString::parse_joint_msg_to_string(const ::godot_grpc::jointControlService::jointVaulesMsg &grpc_msg, ::std_msgs::String &ros1_msg)
{

    /*
    Parse jointValuesMsg to string containing python dictionary 
    only the the fields whose has_value is true are included in the dictionary
    */

    //create python dictionary string
    std::string l_dict_str = "{";

    if(grpc_msg.has_rot_x())
    {
        l_dict_str += "'rot_x': " + std::to_string(grpc_msg.rot_x()) + ", ";
    }

    if(grpc_msg.has_rot_y())
    {
        l_dict_str += "'rot_y': " + std::to_string(grpc_msg.rot_y()) + ", ";
    }

    if(grpc_msg.has_rot_z())
    {
        l_dict_str += "'rot_z': " + std::to_string(grpc_msg.rot_z()) + ", ";
    }

    if(grpc_msg.has_linear_x())
    {
        l_dict_str += "'linear_x': " + std::to_string(grpc_msg.linear_x()) + ", ";
    }

    if(grpc_msg.has_linear_y())
    {
        l_dict_str += "'linear_y': " + std::to_string(grpc_msg.linear_y()) + ", ";
    }

    if(grpc_msg.has_linear_z())
    {
        l_dict_str += "'linear_z': " + std::to_string(grpc_msg.linear_z()) + ", ";
    }

}


//generate ros1 joint control message
void CGRPCJointControlToString::gen_data(::std_msgs::String &f_ros1_msg)
{
    //Generate image interface from callback 
    m_config.gen_img(m_msg);

    //parse joint control message to string
    parse_joint_msg_to_string(m_msg, f_ros1_msg);
}

//set configuration
void CGRPCJointControlToString::set_config(CGRPCJointControlToStringConfig const &f_config)
{
    m_config = f_config;
}

