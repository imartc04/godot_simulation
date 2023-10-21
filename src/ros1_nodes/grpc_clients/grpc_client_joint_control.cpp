
#include "grpc_client_joint_control.hpp"
#include "ros_interface/ros1/ros1_system_manager/ros1_system_manager.hpp"
#include "ros1_nodes/exception/exception.hpp"

// constructor
CGRPCClientJointControl::CGRPCClientJointControl()
{
}

// destructor
CGRPCClientJointControl::~CGRPCClientJointControl()
{
}

// init
void CGRPCClientJointControl::init()
{

    GRPCBaseCilent::init();

    // create grpc stub
    m_stub = simple_camera_service::ImageService::NewStub(m_channel);

    // create stub
    m_stub = ::godot_grpc::joint_control_service::JointControlService::NewStub(m_channel);

    // Get ROS 1 joint reads publisher config from server
    ::grpc::ClientContext l_context;
    ::godot_grpc::ros1::ROS1PublisherConfig l_ros_config;
    ::godot_grpc::emptyMsg l_req;
    auto l_status = m_stub->getROSConfig(&l_context, l_req, &l_ros_config);

    // Check status is ok
    if (l_status.ok())
    {

        //Create grpc joint control parser config
        CGRPCJointControlToStringConfig l_joint_to_string_config = {std::bind(&CGRPCClientJointControl::get_joint_reads, this, std::placeholders::_1)};

        //Create grpc joint control parser object
        auto l_grpc_joint_to_string = make_shared<CGRPCJointControlToString>();

        // Set configuration
        l_grpc_joint_to_string->set_config(l_joint_to_string_config);


        CRos1PublisherConfig l_ros1_pub_config = {l_ros_config, std::bind(&CGRPCJointControlToString::gen_data, l_grpc_joint_to_string.get(), std::placeholders::_1)};


        // create ros1 publisher
        m_ros1_pub = std::make_shared<CRos1Publisher<::godot_grpc::joint_control_service::jointVaulesMsg>>(m_config.ros1_pub_config);

        //Set ros1 publisher config data
        m_ros1_pub->set_config(m_config.ros1_pub_config);

        // crate ros1 subscriber for joint commands
        m_ros1_sub = create_ros1_subscriber_interface_joint_control(m_config.ros1_sub_config);
    }
    else
    {
        std::cout << "Error: " << l_status.error_code() << ": " << l_status.error_message() << std::endl;
        CGRPCClientException l_exception(l_status.error_code(), l_status.error_message());
        l_exception.m_error_code = CGRPCClientException::EGRPCClientExceptionType::CONFIGURATION_ERROR;
        throw l_exception;
    }
}

// set config
void CGRPCClientJointControl::set_config(CGRPCClientJointControlConfig const &f_config)
{
    m_config = f_config;
    m_config.ros1_pub_config.f_get_message = std::bind(&CGRPCClientJointControl::get_joint_control, this, std::placeholders::_1);
}

// get ros1_pub
std::shared_ptr<CRos1PublisherInterface> CGRPCClientJointControl::get_ros1_pub()
{
    return m_ros1_pub;
}

void CGRPCClientJointControl::set_joint_commands(::godot_grpc::joint_control_service::jointVaulesMsg &f_joint_control)
{

    // Set new joint commands into server
    ::grpc::ClientContext l_context;

    // Create request
    ::godot_grpc::joint_control_service::jointVaulesMsg l_req;
    auto l_status = m_stub->setJointCommands(&l_context, f_joint_control, &l_req);
}

void CGRPCClientJointControl::get_joint_reads(::godot_grpc::joint_control_service::jointVaulesMsg &f_joint_reads)
{
    // Get joint reads from gRPC server
    ::grpc::ClientContext l_context;
    ::godot_grpc::emptyMsg l_req;
    auto l_status = m_stub->getJointReads(&l_context, l_req, &f_joint_reads);

    // Check status is ok
    if (!l_status.ok())
    {
        std::cout << "Error: " << l_status.error_code() << ": " << l_status.error_message() << std::endl;
        CGRPCClientException l_exception(l_status.error_code(), l_status.error_message());
        l_exception.m_error_code = CGRPCClientException::EGRPCClientExceptionType::CONFIGURATION_ERROR;
        throw l_exception;
    }
}
