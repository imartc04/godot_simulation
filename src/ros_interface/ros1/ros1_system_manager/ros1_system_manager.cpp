

#include "ros1_system_manager.hpp"
#include "grpc_image_to_ros1.hpp"
#include "grpc_joint_control_to_string.hpp"


#include "grpc_interface/gen_protoc/simple_camera_service.pb.h"

std::shared_ptr<CRos1PublisherInterface> create_ros1_publisher_interface_image(::godot::CRos1PublisherConfig<::godot_grpc::simple_camera_service::imageMsg> const &f_config)
{
    std::shared_ptr<CRos1PublisherInterface> l_ret;

    // Create publisher object
    auto l_ros1_pub = make_shared<CRos1Publisher<::sensor_msgs::Image>>();

    // Create grpc image to ros1 image object

    CGRPCImageToRos1Config l_img_to_ros1_config = {f_config.f_get_message};
    auto l_grpc_img_to_ros1 = make_shared<CGRPCImageToRos1>();
    l_grpc_img_to_ros1->set_config(l_img_to_ros1_config);


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

    //Create parser object 
    CGRPCJointControlToStringConfig l_joint_to_string_config = {f_config.f_get_message};
    auto l_grpc_joint_to_string = make_shared<CGRPCJointControlToString>();

    //Create ros1 publisher object
    auto l_ros1_pub = make_shared<CRos1Publisher<::std_msgs::String>>();

    // Set configuration
    auto l_config = CRos1PublisherConfig{f_config, std::bind(&CGRPCJointControlToString::gen_data, l_grpc_joint_to_string.get(), std::placeholders::_1)};

    // Set ros1 publisher config data
    ros1_pub->set_config(*ros1_pub_config);

    /*Set reference to te config object to avoid destruction of it
    when exiting this function
    */
    ros1_pub->add_obj_dep(static_pointer_cast<void>(l_grpc_joint_to_string));

    return std::static_pointer_cast<CRos1PublisherInterface>(ros1_pub);
}

std::shared_ptr<CROS1Subscriber> create_ros1_subscriber_interface_joint_control(CROS1SubscriberConfig<::godot_grpc::joint_control_service::jointVaulesMsg> const &f_config)
{
    std::shared_ptr<CROS1SubscriberIf> l_ret;

    ///Create parser object
    CGRPCStringToJointControlConfig l_string_to_joint_config = {f_config.f_set_message};
    auto l_grpc_string_to_joint = make_shared<CGRPCStringToJointControl>();
    l_grpc_string_to_joint->set_config(l_string_to_joint_config);
    
    // Create subscriber object
    auto l_ros1_sub = make_shared<CROS1Subscriber<::std_msgs::String>>();

    //Create subscriber config
    auto l_config = CROS1SubscriberConfig<::std_msgs::String>{f_config.topic_name, f_config.queue_size, std::bind(&CGRPCStringToJointControl::parse_string_to_joint, l_grpc_string_to_joint.get(), std::placeholders::_1), f_config.spin_thread};

    //set subscriber config
    l_ros1_sub->set_config(l_config);

    /*Set reference to te config object to avoid destruction of it
    when exiting this function
    */
    l_ros1_sub->add_obj_dep(static_pointer_cast<void>(l_grpc_string_to_joint));


    //return data
    l_ret = std::static_pointer_cast<CROS1SubscriberIf>(l_ros1_sub);
}