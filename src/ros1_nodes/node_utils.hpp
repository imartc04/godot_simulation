
#pragma once

#include <iostream>
#include <grpcpp/client_context.h>
#include "grpc_interface/gen_protoc/commonMessages.pb.h"


// Function to send status message to gRPC server
template<typename t_stub>
static int send_status_to_grpc_server(int i_status, t_stub* f_tub)
{
    int l_ret = 0;
    ::grpc::ClientContext l_context;
    ::godot_grpc::uint32Msg l_request;
    ::godot_grpc::emptyMsg l_reply;

    l_request.set_value(i_status);

    auto l_status = f_tub->setClientStatus(&l_context, l_request, &l_reply);

    if (!l_status.ok())
    {
        std::cout << "Could not send status " << std::endl;
        l_ret = 1;
    }

    return l_ret;
}