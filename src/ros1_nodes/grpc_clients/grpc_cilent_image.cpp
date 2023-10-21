#include "grpc_cilent_image.hpp"

#include "ros_interface/ros1/ros1_system_manager/ros1_system_manager.hpp"
#include "ros1_nodes/exception/exception.hpp"


//constructor
CGRPCClientImage::CGRPCClientImage()
{
}

//destructor
CGRPCClientImage::~CGRPCClientImage()
{
}

//init
void CGRPCClientImage::init()
{
  
    GRPCBaseCilent::init();

    //create grpc stub
    m_stub = simple_camera_service::ImageService::NewStub(m_channel);

    // Get ROS config from server
    ::grpc::ClientContext l_context;
    ::godot_grpc::ros1::ROS1PublisherConfig l_ros_config;
    ::godot_grpc::emptyMsg l_req;
    auto l_status = l_stub->getROSConfig(&l_context, l_req, &l_ros_config);

    //Check status is ok 
    if (l_status.ok())
    {
        //Create ros publisher
        m_ros1_pub = create_ros1_publisher_interface_image(m_config.ros1_pub_config);

    }
    error
    {
        std::cout << "Error: " << l_status.error_code() << ": " << l_status.error_message() << std::endl;
        CGRPCClientException l_exception(l_status.error_code(), l_status.error_message());
        l_exception.m_error_code = CGRPCClientException::EGRPCClientExceptionType::CONFIGURATION_ERROR;
        throw l_exception;
    }

    auto ros1_pub
}

//set config
void CGRPCClientImage::set_config(CGRPCClientImageConfig const &f_config)
{
    m_config = f_config;
    m_config.f_get_message = std::bind(&CGRPCClientImage::get_image, this, std::placeholders::_1);
}



void get_image(::godot_grpc::simple_camera_service::imageMsg &f_img);
{

     = make_shared<CRos1Publisher<::sensor_msgs::Image>>();

    // Create ROS1 publisher config data
    auto ros1_pub_config = make_shared<::godot::CRos1PublisherConfig<::sensor_msgs::Image>>();

    // Set callback functino in ros1 publisher config data

    ros1_pub_config->f_get_message = std::bind(gen_ros1_img_data, std::placeholders::_1);
    ros1_pub_config->proto_config = std::move(l_ros_config);

    // Set ros1 publisher config data
    ros1_pub->set_config(*ros1_pub_config);

    return std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
}