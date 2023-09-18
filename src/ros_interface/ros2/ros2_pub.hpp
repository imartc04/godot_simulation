#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cstdint>

namespace godot_ros
{

    struct CRos2TopicPubConfig
    {
        std::string topic_name;
        std::string topic_type;
        uint16_t queue_size;
        bool is_configured = false;
    };

    class CRos2TopicPub
    {

        // Constructor
    public:
        CRos2TopicPub();

        // Destructor

        ~CRos2TopicPub();

        // Config method
        void config(CRosPublisherConfig &const f_config);

        // Init method
        void init();

        // Publish method
        void publish(void *f_msg);

    private :

        // ROS2 node
        rclcpp::Node::SharedPtr m_node;

        // ROS2 publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;

        // Config
        CRos2TopicPubConfig m_config;


    }

}