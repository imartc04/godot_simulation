#pragma once

#include <string>
#include <grpcpp/create_channel.h>

struct CGRPCBaseCilentConfig
{

    /**
     * String of type server
     */
    std::string server_address;

    /**
     * String of type port
     */
    std::string server_port;
};

/**
 * Base class to manage gRPC clients
 * that interact with the Godot system
 */
class CGRPCBaseCilent
{

public:
    CGRPCBaseCilent(/* args */);
    ~CGRPCBaseCilent();

    void init();

    void set_config(CGRPCBaseCilentConfig const &f_config);

protected:
    std::shared_ptr<::grpc::Channel> m_channel;

private:
    CGRPCBaseCilentConfig m_config;
};

CGRPCBaseCilent::CGRPCBaseCilent(/* args */)
{
}

CGRPCBaseCilent::~CGRPCBaseCilent()
{
}
