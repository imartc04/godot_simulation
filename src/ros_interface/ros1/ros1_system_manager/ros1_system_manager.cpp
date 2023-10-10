

#include "ros1_system_manager.hpp"
#include "grpc_image_to_ros1.hpp"
#include <std_msgs/String.h>

#include "grpc_interface/gen_protoc/simple_camera_service.pb.h"

std::shared_ptr<CRos1PublisherInterface> create_ros1_publisher_interface_image(::godot::CRos1PublisherConfig<::godot_grpc::simple_camera_service::imageMsg> const &f_config)
{
    std::shared_ptr<CRos1PublisherInterface> l_ret;

    // Create publisher object
    auto l_ros1_pub = make_shared<CRos1Publisher<::sensor_msgs::Image>>();

    // Create grpc image to ros1 image object
    auto l_grpc_img_to_ros1 = make_shared<CGRPCImageToRos1>();

    // Set configuration
    auto l_config = CRos1PublisherConfig{f_config, std::bind(&CGRPCImageToRos1::gen_ros1_img_data, l_grpc_img_to_ros1.get(), std::placeholders::_1)};

    // Set ros1 publisher config data
    ros1_pub->set_config(f_config);

    /*Set reference to te config object to avoid destruction of it
    when exiting this function
    */
    ros1_pub->add_obj_dep(static_pointer_cast<void>(l_grpc_img_to_ros1));

    l_ret = std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);

    return l_ret;
}


std::shared_ptr<CRos1PublisherInterface> create_ros1_publisher_interface_joint_control(::godot::CRos1PublisherConfig<::godot_grpc::joint_control_service::jointVaulesMsg> const &f_config)
{

    auto l_ros1_pub = make_shared<CRos1Publisher<::std_msgs::String>>();

   
    // Create ROS1 publisher config data
    auto ros1_pub_config = make_shared<::godot::CRos1PublisherConfig<::std_msgs::String>>();


    // Set ros1 publisher config data
    ros1_pub->set_config(*ros1_pub_config);

    return std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
}