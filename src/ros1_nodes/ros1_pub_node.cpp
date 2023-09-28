#include "ros_interface/ros1/ros1_pub.hpp"

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <map>
#include <variant>
#include <string>
#include <memory>
#include <exception>
#include <chrono>

#include <grpc/grpc.h>
#include <grpcpp/client_context.h>
#include <grpcpp/server_context.h>
#include <grpcpp/server.h>
#include <grpcpp/create_channel.h>

#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

#include "grpc_interface/gen_protoc/ros1.pb.h"
#include "grpc_interface/gen_protoc/simple_camera_service.grpc.pb.h"
#include "grpc_interface/gen_protoc/simple_camera_service.pb.h"
#include "grpc_interface/gen_protoc/ros1.pb.h"
#include "grpc_interface/gen_protoc/commonMessages.pb.h"

using namespace std;
using namespace grpc;
using namespace godot;

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

string g_ros_msg_image = "sensor_msgs/Image";

namespace gcamera = ::godot_grpc::simple_camera_service;

static std::unique_ptr<gcamera::SimpleCameraService::Stub> g_rpc_stub;

uint64_t g_seq = 0;

// Help method on how to use application
void help()
{
    std::cout << "Usage: ros1_pub_node <grpc_server_address> <grpc_server_port>" << std::endl;
}

// Function to parse grpc message image to ros1 message image
void parse_grpc_msg_to_ros1_msg(const gcamera::imageMsg &grpc_msg, sensor_msgs::Image &ros1_msg)
{
    // Set ros1 message sequence checkingfor overflow
    ros1_msg.header.seq = g_seq;

    if (g_seq == UINT64_MAX)
    {
        g_seq = 0;
    }
    else
    {
        g_seq++;
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

    ros1_msg.data.reserve(l_data.size());

    for (auto &i_byte_data : l_data)
    {
        ros1_msg.data.push_back(static_cast<uint8_t>(i_byte_data));
    }
}

// Callback function to generate ros1 message from shared memory passed data
template <typename t_message>
t_message gen_ros1_msg_from_shm_data()
{
    // Create ros1 message object
    t_message ros1_msg;

    ClientContext l_context;

    gcamera::imageRequestMsg l_request;
    gcamera::imageMsg l_reply;

    // Get new image from gRPC server
    auto l_status = g_rpc_stub->getImage(&l_context, l_request, &l_reply);

    if (l_status.ok())
    {
        // Parse gRPC message to ros1 message
        parse_grpc_msg_to_ros1_msg(l_reply, ros1_msg);
    }
    else
    {
        cout << "Status return for get image from gRPC server is not ok" << endl;
    }

    return std::move(ros1_msg);
}

// Function to send status message to gRPC server
int send_status_to_grpc_server(int i_status)
{
    int l_ret = 0;
    ::grpc::ClientContext l_context;
    ::godot_grpc::uint32Msg l_request;
    ::godot_grpc::emptyMsg l_reply;

    l_request.set_value(i_status);

    auto l_status = g_rpc_stub->setClientStatus(&l_context, l_request, &l_reply);

    if (!l_status.ok())
    {
        cout << "Could not send status " << endl;
        l_ret = 1;
    }

    return l_ret;
}

/**
 * Application that creates a ros1 publisher for a given ros1 message type
 *
 * The application is intented to work with interprocess shared memory communication
 * to share the next para meters listed by the order of the arguments:
 *
 *  - grpc_server_address: Address of the grpc server
 *  - grpc_server_port: Port of the grpc server
 *
 *
 * Return values :
 *
 *  - 0: Success
 *  - 1: General config error
 *  - 2: Error in ros1 publisher, relaunch node
 *
 */
int main(int argc, char **argv)
{

    { // Aux debug wait for debugge to attach

        bool l_aux = true;

        while (l_aux)
        {
            std::this_thread::sleep_for(10s);
        }
    }

    int l_ret = 0;

    // Check passed arguments are correct and print help when needed
    if (argc != 3)
    {
        help();
        return 1;
    }

    // Create gRPC client from passed arguments
    std::string grpc_server_address = argv[1];
    std::string grpc_server_port = argv[2];
    std::shared_ptr<Channel> l_grpc_client_channel = grpc::CreateChannel(grpc_server_address + ":" + grpc_server_port, grpc::InsecureChannelCredentials());

    g_rpc_stub = ::godot_grpc::simple_camera_service::SimpleCameraService::NewStub(l_grpc_client_channel);

    // Get ROS config from server
    ::grpc::ClientContext l_context;
    ::godot_grpc::ros1::ROS1PublisherConfig l_ros_config;
    ::godot_grpc::emptyMsg l_req;
    auto l_status = g_rpc_stub->getROSConfig(&l_context, l_req, &l_ros_config);

    if (l_status.ok())
    {

        // Initialize ROS
        if (!ros::isInitialized())
        {
            int l_argc = 0;
            char **l_argv = NULL;

            ros::init(l_argc, l_argv, l_ros_config.node_name());
        }

        // Create storage for ROS1 publisher object as a variant
        shared_ptr<CRos1PublisherInterface> m_ros_if;

        // Create ros1 publisher object depending on the type of ros1 message
        if (l_ros_config.topic_type() == g_ros_msg_image)
        {
            std::shared_ptr<CRos1Publisher<::sensor_msgs::Image>> ros1_pub = make_shared<CRos1Publisher<::sensor_msgs::Image>>();

            // Create ROS1 publisher config data
            auto ros1_pub_config = make_shared<::godot::CRos1PublisherConfig<::sensor_msgs::Image>>();

            // Set callback functino in ros1 publisher config data

            ros1_pub_config->f_get_message = gen_ros1_msg_from_shm_data<::sensor_msgs::Image>;
            ros1_pub_config->proto_config = std::move(l_ros_config);

            // Set ros1 publisher config data
            ros1_pub->set_config(*ros1_pub_config);

            m_ros_if = std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
        }
        else
        {
            std::cout << "Error: ros1 message type not supported" << std::endl;
            send_status_to_grpc_server(1);
            l_ret = 1;
        }

        // Initialize ros1 publisher
        m_ros_if->init();

        // Publish ros1 messages until some error occurred
        m_ros_if->publish_loop();

        // Check if there is some error in ros1
        if (m_ros_if->ros_error())
        {
            std::cout << "Error: ros1 publisher error" << std::endl;
            send_status_to_grpc_server(2);
            l_ret = 2;
        }
    }
    else
    {
        send_status_to_grpc_server(1);
        return 1;
    }

    return l_ret;
}
