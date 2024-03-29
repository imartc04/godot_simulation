#include "ros_interface/ros1/ros1_pub.hpp"

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
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
#include "grpc_interface/gen_protoc/joint_control_service.grpc.pb.h"

#include "grpc_interface/gen_protoc/ros1.pb.h"
#include "grpc_interface/gen_protoc/commonMessages.pb.h"

#include "node_utils.hpp"

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

// static std::unique_ptr<gcamera::SimpleCameraService::Stub> g_rpc_stub;

uint64_t g_seq = 0;

// Help method on how to use application
void help()
{
    std::cout << "Usage: ros1_pub_node <grpc_server_address> <grpc_server_port>" << std::endl;
}



std::shared_ptr<CRos1PublisherInterface> create_ros1_joint_control(Channel &f_channel)
{

    auto l_stub = ::godot_grpc::joint_control_service::jointControlService::NewStub(f_channel);

    auto l_ros1_pub = make_shared<CRos1Publisher<::std_msgs::String>>();

    // Get ROS config from server
    ::grpc::ClientContext l_context;
    ::godot_grpc::ros1::ROS1PublisherConfig l_ros_config;
    ::godot_grpc::emptyMsg l_req;
    auto l_status = l_stub->getROSConfig(&l_context, l_req, &l_ros_config);

    // Create ROS1 publisher config data
    auto ros1_pub_config = make_shared<::godot::CRos1PublisherConfig<::std_msgs::String>>();

    // Set callback functino in ros1 publisher config data
    ros1_pub_config->f_get_message = std::bind(gen_joint_control_msg, std::placeholders::_1);
    ros1_pub_config->proto_config = std::move(l_ros_config);

    // Set ros1 publisher config data
    ros1_pub->set_config(*ros1_pub_config);

    return std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
}

/**
 * Application that creates a ros1 publisher for a given ros1 message type
 *
 * The application is intented to work with interprocess shared memory communication
 * to share the next para meters listed by the order of the arguments:
 *
 *  - grpc_server_address: Address of the grpc server
 *  - grpc_server_port: Port of the grpc server
 *  - server_type : Server for sensor image data, joint control datata, altimeter etc.
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

    // { // Aux debug wait for debugge to attach

    //     bool l_aux = true;

    //     while (l_aux)
    //     {
    //         std::this_thread::sleep_for(10s);
    //     }
    // }

    int l_ret = 0;

    // Check passed arguments are correct and print help when needed
    if (argc != 4)
    {
        help();
        return 1;
    }

    // Create gRPC client from passed arguments
    std::string grpc_server_address = argv[1];
    std::string grpc_server_port = argv[2];
    string grpc_server_type = argv[3];

    std::shared_ptr<Channel> l_grpc_client_channel = grpc::CreateChannel(grpc_server_address + ":" + grpc_server_port, grpc::InsecureChannelCredentials());

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
        shared_ptr<CRos1PublisherInterface> l_ros_if;

        // Create ros1 publisher object depending on the type of ros1 message
        if (grpc_server_type == "image")
        {
            l_ros_if = create_ros1_pub_image(l_grpc_client_channel);
        }
        else if (grpc_server_type == "joint_control")
        {
            l_ros_if = create_ros1_joint_control(l_grpc_client_channel);
        }
        else
        {
            std::cout << "Error: ros1 message type not supported" << std::endl;
            send_status_to_grpc_server(1, g_rpc_stub.get());
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
            send_status_to_grpc_server(2, g_rpc_stub.get());
            l_ret = 2;
        }
    }
    else
    {
        send_status_to_grpc_server(1, g_rpc_stub.get());
        return 1;
    }

    return l_ret;
}
