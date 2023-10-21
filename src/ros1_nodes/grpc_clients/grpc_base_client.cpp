#include "grpc_base_client.hpp"



//constructor 
CGRPCBaseCilent::CGRPCBaseCilent()
{
}

//destructor
CGRPCBaseCilent::~CGRPCBaseCilent()
{
}

//init
void CGRPCBaseCilent::init()
{
    //create grpc channel
    m_channel = ::grpc::CreateChannel(m_config.server_address + ":" + m_config.server_port, ::grpc::InsecureChannelCredentials());

}

//set config
void CGRPCBaseCilent::set_config(CGRPCBaseCilentConfig const &f_config)
{
    m_config = f_config;
}