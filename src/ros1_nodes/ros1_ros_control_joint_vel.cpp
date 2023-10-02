

#include "ros/ros.h"
#include <grpc/grpc.h>
#include <grpcpp/client_context.h>
#include <grpcpp/server_context.h>
#include <grpcpp/server.h>
#include <grpcpp/create_channel.h>

#include "grpc_interface/gen_protoc/joint_vel.grpc.pb.h"

#include "ros_interface/ros1/joint_controllers/ros_control_based/ros1_vel_scalar_hw_if.hpp"

#include "node_utils.hpp"

#include <string>
#include <thread>
#include <chrono>

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using namespace godot;
using namespace std;
using namespace ::godot_grpc::joint_vel;

int main(int argc, char **argv)
{

    int l_ret = 0;

    // Create gRPC client from passed arguments
    std::string grpc_server_address = argv[1];
    std::string grpc_server_port = argv[2];
    std::shared_ptr<Channel> l_grpc_client_channel = grpc::CreateChannel(grpc_server_address + ":" + grpc_server_port, grpc::InsecureChannelCredentials());

    auto l_grpc_stub = ::godot_grpc::joint_vel::JointVelService::NewStub(l_grpc_client_channel);

    // Get joint controller config from server
    ::grpc::ClientContext l_context;
    ::godot_grpc::joint_vel::jointVelConfigMsg l_grpc_config;
    ::godot_grpc::emptyMsg l_req;

    auto l_status = l_grpc_stub->getConfig(&l_context, l_req, &l_grpc_config);

    CRos1VelocityHWinterface l_ros_control_if;

    if (l_status.ok())
    {
        string l_node_name = "ros1_ros_control_joint_vel" + std::to_string(l_grpc_config.control_var_index()) + "_" + l_grpc_config.control_var_name();

        // Init ros
        ros::init(argc, argv, l_node_name.c_str());

        CRos1VelocityHWinterfaceConfig l_config{l_grpc_config.control_var_name(), l_grpc_config.control_var_index(), l_grpc_config.update_period_ms()};

        l_ros_control_if.set_config(l_config);
        l_ros_control_if.init();

        // Aux data vec
        std::vector<double> l_aux_joint_reads;

        // Loop to read and write joint data
        while (ros::ok())
        {

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint64_t>(l_config.update_period_ms)));

            {
                // Send new command values to server
                auto l_commands = l_ros_control_if.get_joint_commands();
                ClientContext l_context;
                jointVelocitiesMsg l_req;
                auto l_num_data = l_commands.size();
                l_req.mutable_vels()->Resize(l_num_data, 0.f);

                ::godot_grpc::emptyMsg l_response;

                for (int i = 0u; i < l_num_data; i++)
                {
                    l_req.set_vels(i, l_commands[i]);
                }

                l_grpc_stub->setJointCommand(&l_context, l_req, &l_response);
            }

            {
                // Read joint values
                ClientContext l_context;
                jointVelocitiesMsg l_response;
                ::godot_grpc::emptyMsg l_req;

                auto l_status = l_grpc_stub->readJointData(&l_context, l_req, &l_response);

                if (l_status.ok())
                {

                    // Set inferface data new reads
                    auto l_commands = l_response.vels();


                    int l_num_data = l_commands.size();

                    l_aux_joint_reads.resize(l_num_data);

                    for (int i = 0u; i < l_num_data; i++)
                    {
                        l_aux_joint_reads[i] = l_commands[i];
                    }

                    l_ros_control_if.set_new_joint_reads(l_aux_joint_reads);
                }
                else
                {
                    send_status_to_grpc_server(1, l_grpc_stub.get());
                    return 1;
                }
            }

            ros::spinOnce();
        }

        send_status_to_grpc_server(2, l_grpc_stub.get());
        return 2;
    }
    else
    {
        send_status_to_grpc_server(1, l_grpc_stub.get());
        return 1;
    }

    return l_ret;
}