#include "grpc_image_to_ros1.hpp"

#include "ros/ros.h"

CGRPCImageToRos1::CGRPCImageToRos1(/* args */)
{

    // Initialize ROS
    if (!ros::isInitialized())
    {
        int l_argc = 0;
        char **l_argv = NULL;

        ros::init(l_argc, l_argv, l_ros_config.node_name());
    }
}

CGRPCImageToRos1::~CGRPCImageToRos1()
{
}

// Function to parse grpc message image to ros1 message image
void CGRPCImageToRos1::parse_grpc_msg_to_ros1_msg(const ::godot_grpc::simple_camera_service::imageMsg &grpc_msg, sensor_msgs::Image &ros1_msg)
{
    // Set ros1 message sequence checkingfor overflow
    ros1_msg.header.seq = m_seq_num;

    if (m_seq_num == UINT64_MAX)
    {
        m_seq_num = 0;
    }
    else
    {
        m_seq_num++;
    }

    // Set ros1 message stamp with ros time
    ros1_msg.header.stamp = ros::Time::now();

    ros1_msg.header.frame_id = grpc_msg.frame_id();

    // Set ros1 message height
    ros1_msg.height = grpc_msg.height();

    // Set ros1 message width
    ros1_msg.width = grpc_msg.width();

    // Set ros1 message encoding
    ros1_msg.encoding = grpc_msg.encoding();

    // Set ros1 message is_bigendian
    ros1_msg.is_bigendian = grpc_msg.is_bigendian();

    // Set ros1 message step
    ros1_msg.step = grpc_msg.step();

    // Set ros1 message data
    auto &l_data = grpc_msg.uint8_data();

    auto l_num_data = l_data.size();

    ros1_msg.data.clear();
    ros1_msg.data.reserve(l_num_data);

    for (auto &i_byte_data : l_data)
    {
        ros1_msg.data.push_back(static_cast<uint8_t>(i_byte_data));
    }
}

// Callback function to generate ros1 message from shared memory passed data
void CGRPCImageToRos1::gen_data(::sensor_msgs::Image &f_ros1_msg)
{

    // Get new image from callback
    m_config.gen_img(m_msg);

    if (l_status.ok())
    {
        // Parse gRPC message to ros1 message
        parse_grpc_msg_to_ros1_msg(l_reply, m_msg);
    }
    else
    {
        cout << "Status return for get image from gRPC server is not ok" << endl;
    }
}