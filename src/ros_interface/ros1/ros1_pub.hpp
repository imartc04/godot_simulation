#pragma once

/** \file ros1_pub.hpp
 * @brief Contains ROS1 publisher class, it config struct and macros to allow faster godot bind method creation
 *
 * This file define macros because ros publisher needs config parameters that have to be allowed to be set from Godot UI
 * and by using a macro we dont have to write the same code in each class that uses the CRos1Publisher
 *
 */

#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <functional>
#include "ros1_pub_if.hpp"
#include <shared_mutex>
#include <mutex>

#include "grpc_interface/gen_protoc/ros1.pb.h"

namespace gros1 = ::godot_grpc::ros1;

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the .cpp file of the class that will use the CRos1Publisher class
 * For a example of usage see the CSensorBasicCamera class
 *
 * @param CLASS - Name of the GDClass
 * @param MEMBER_NAME - Name of the CRos1Publisher member within the CLASS
 * @param ROS1_MSG_TYPE - ROS1 message type
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_DEF_METHODS(CLASS, MEMBER_NAME, ROS1_MSG_TYPE)                                                                    \
    void CLASS::set_ros1_node_name(String f_node_name)                                                                              \
    {                                                                                                                               \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.node_name = f_node_name.utf8().get_data();                      \
    }                                                                                                                               \
                                                                                                                                    \
    String CLASS::get_ros1_node_name()                                                                                              \
    {                                                                                                                               \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.node_name.c_str();                                       \
    }                                                                                                                               \
                                                                                                                                    \
    void CLASS::set_ros1_topic_name(String f_topic_name)                                                                            \
    {                                                                                                                               \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.topic_name = f_topic_name.utf8().get_data();                    \
    }                                                                                                                               \
    String CLASS::get_ros1_topic_name()                                                                                             \
    {                                                                                                                               \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.topic_name.c_str();                                      \
    }                                                                                                                               \
    void CLASS::set_ros1_queue_size(int f_queue_size)                                                                               \
    {                                                                                                                               \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.queue_size = f_queue_size;                                      \
    }                                                                                                                               \
    int CLASS::get_ros1_queue_size()                                                                                                \
    {                                                                                                                               \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.queue_size;                                              \
    }                                                                                                                               \
    void CLASS::set_ros1_pub_rate(float f_pub_rate_hz)                                                                              \
    {                                                                                                                               \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_info.pub_rate_hz_f32 = f_pub_rate_hz;                       \
    }                                                                                                                               \
    float CLASS::get_ros1_pub_rate()                                                                                                \
    {                                                                                                                               \
        return MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_info.pub_rate_hz_f32;                                \
    }                                                                                                                               \
    void CLASS::set_ros1_pub_type_rate(int f_type)                                                                                  \
    {                                                                                                                               \
        MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_info.pub_type = static_cast<::godot::EPublishType>(f_type); \
    }                                                                                                                               \
    int CLASS::get_ros1_pub_type_rate()                                                                                             \
    {                                                                                                                               \
        return static_cast<int>(MEMBER_NAME.get_config().base_sensor_config.ros1_pub_config.pub_info.pub_type);                     \
    }

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the .hpp file of the class that will use the CRos1Publisher class
 * For a example of usage see the header file of CSensorBasicCamera class
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_DECLARE_METHODS()                \
    void set_ros1_node_name(String f_node_name);   \
    String get_ros1_node_name();                   \
    void set_ros1_topic_name(String f_topic_name); \
    String get_ros1_topic_name();                  \
    void set_ros1_queue_size(int f_queue_size);    \
    int get_ros1_queue_size();                     \
    void set_ros1_pub_rate(float f_pub_rate_hz);   \
    float get_ros1_pub_rate();                     \
    void set_ros1_pub_type_rate(int f_type);       \
    int get_ros1_pub_type_rate();

/**
 * Macro to allow faster godot bind method creation
 *
 * This macro is intended to be used in the implementation of the _bind_methods method of the class node
 * that use the CRos1Publisher class. This will bind to the godot system the methods defined in the macro
 * #GDROS1PUB_DECLARE_METHODS
 * For a example of usage see the header file of CSensorBasicCamera class
 *
 * @see CSensorBasicCamera
 */
#define GDROS1PUB_BIND_METHODS(CLASS)                                                                               \
    ClassDB::bind_method(D_METHOD("set_ros1_node_name", "ros1_node_name"), &CLASS::set_ros1_node_name);             \
    ClassDB::bind_method(D_METHOD("get_ros1_node_name"), &CLASS::get_ros1_node_name);                               \
    ClassDB::bind_method(D_METHOD("set_ros1_topic_name", "ros1_topic_name"), &CLASS::set_ros1_topic_name);          \
    ClassDB::bind_method(D_METHOD("get_ros1_topic_name"), &CLASS::get_ros1_topic_name);                             \
    ClassDB::bind_method(D_METHOD("set_ros1_queue_size", "ros1_queue_size"), &CLASS::set_ros1_queue_size);          \
    ClassDB::bind_method(D_METHOD("get_ros1_queue_size"), &CLASS::get_ros1_queue_size);                             \
    ClassDB::bind_method(D_METHOD("set_ros1_pub_rate", "ros1_pub_rate"), &CLASS::set_ros1_pub_rate);                \
    ClassDB::bind_method(D_METHOD("get_ros1_pub_rate"), &CLASS::get_ros1_pub_rate);                                 \
    ClassDB::bind_method(D_METHOD("set_ros1_pub_type_rate", "ros1_pub_type_rate"), &CLASS::set_ros1_pub_type_rate); \
    ClassDB::bind_method(D_METHOD("get_ros1_pub_type_rate"), &CLASS::get_ros1_pub_type_rate);                       \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_node_name"), "set_ros1_node_name", "get_ros1_node_name");      \
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "ros1_topic_name"), "set_ros1_topic_name", "get_ros1_topic_name");   \
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ros1_queue_size"), "set_ros1_queue_size", "get_ros1_queue_size");      \
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ros1_pub_rate"), "set_ros1_pub_rate", "get_ros1_pub_rate");          \
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ros1_pub_type_rate"), "set_ros1_pub_type_rate", "get_ros1_pub_type_rate");

namespace godot
{

    /**
     * Configuration struct for the CRos1Publisher class
     *
     * @tparam t_message_type - ROS1 message type
     */
    template <typename t_message_type>
    struct CRos1PublisherConfig
    {   
        /**Protobuf generated class for config */
        gros1::ROS1PublisherConfig proto_config;

        /** User callback function to generate new message*/
        std::function<t_message_type()> f_get_message;
    };

    /**
     * Generic ROS1 publisher class
     *
     * How to use the class:
     *  - Create a CRos1Publisher object and set the configuration parameters
     *  - Call the init() method
     *  - Call the publish_once() method to publish a message once if the publisher is configured in event mode
     *  - Call the publish_loop() method to publish a message in loop if the publisher is configured in rate mode. The call to this method is blocking and
     *   a callback function has to be set in the configuration in order to obtain the message to publish.
     *
     * @tparam t_message_type - ROS1 message type
     */
    template <typename t_message_type>
    class CRos1Publisher : public CRos1PublisherInterface
    {

    private:
        // CAuxInitRos m_aux_ros_init;
        ros::NodeHandle m_node_handle;
        ros::Publisher m_publisher;
        ros::Rate m_rate{10};

        // Flag indicating if the ROS node is down
        bool m_ros_error = false;

        std::thread m_loop_pub_thread;

        /**
         * Flag to indicate publish loop to exit
         */
        bool m_exit_pub_loop = false;

        /**
         * Mutex to safe thread access
         */
        std::shared_mutex m_loop_mtx;

        // Published message sequence id
        uint32_t m_seq_id_u32 = 0;

        CRos1PublisherConfig<t_message_type> m_config;

        /**
         * Internal method to publish a message
         *
         * Usees the ros publisher to publish a message and then spin once to process callbacks and update ROS state
         */
        void pub_message(t_message_type &f_msg)
        {
            // Publish the message
            m_publisher.publish(f_msg);

            // Spin once to process callbacks and update ROS state
            ros::spinOnce();

            // Sleep to maintain the publishing rate
            m_rate.sleep();

            // Increase sequence id
            m_seq_id_u32++;
        }

    public:
        CRos1Publisher(){};

        ~CRos1Publisher()
        {

            if (m_loop_pub_thread.joinable())
            {
                // Mark publish loop to exit in case
                set_exit_publish_loop(true);

                // Wait for thread to finish
                m_loop_pub_thread.join();
            }
        };

        /**
         * @copydoc CRos1PublisherInterface::ros_error
         *
         */
        virtual bool ros_error() override
        {
            return m_ros_error;
        }

        /**
         * @copydoc CRos1PublisherInterface::stop_publishing
         *
         */
        virtual void stop_publishing() override
        {
            set_exit_publish_loop(true);
        }

        /**
         * Get the current sequence id
         * The sequence id is increased each time a message is published starts at 0
         */
        uint32_t get_seq_id()
        {
            return m_seq_id_u32;
        }

        void set_config(CRos1PublisherConfig<t_message_type> const &f_config)
        {
            m_config = f_config;
        };

        CRos1PublisherConfig<t_message_type> &get_config()
        {
            return m_config;
        };

        void init() override
        {

            if (m_config.proto_config.enabled() == true)
            {

                // Initialize the ROS node handle
                m_node_handle = ros::NodeHandle();

                // Create a publisher object
                m_publisher = m_node_handle.advertise<t_message_type>(m_config.proto_config.topic_name(), 10);

                if (m_config.proto_config.pub_type() == gros1::PUBLISH_TYPE_RATE)
                {

                    // Set the publishing rate
                    m_rate = ros::Rate(m_config.proto_config.pub_rate_hz_f32());

                    // Crete publishing thread
                    m_loop_pub_thread = std::thread(&CRos1Publisher<t_message_type>::publish_loop, this);
                }
            }
            else
            {
                throw std::runtime_error("CRosTopicPub::init() : Publisher not configured");
            }
        };

        bool get_exit_publish_loop()
        {
            std::shared_lock<std::shared_mutex> l_lock(m_loop_mtx);
            return m_exit_pub_loop;
        };

        void set_exit_publish_loop(bool f_value)
        {
            std::unique_lock<std::shared_mutex> l_lock(m_loop_mtx);
            m_exit_pub_loop = f_value;
        };

        /**
         * Method to publish a message in loop
         *
         * Use this method if the publisher is configured in rate mode. The message to publish is obtained
         * from calling user's callback function to generate the message.
         */
        void publish_loop() override
        {

            if (m_config.proto_config.pub_type() == gros1::PUBLISH_TYPE_RATE)
            {

                while (!get_exit_publish_loop() && !m_ros_error)
                {

                    if (ros::ok())
                    {
                        auto l_message = m_config.proto_config.f_get_message();

                        pub_message(l_message);
                    }
                    else
                    {
                        // Some error occured node is down
                        m_ros_error = true;
                    }
                }
            }

            std::cout << "ROS node exiting publish loop" << std::endl;
        };

        /**
         * Publish a message once
         *
         * Use this method if the publisher is configured in event mode. The mesage to publish has to
         * be passed as parameter
         *
         * @param f_message - Message to publish
         */
        void publish_once(t_message_type &f_message)
        {

            if (m_config.proto_config.pub_type() == gros1::PUBLISH_TYPE_EVENT)
            {
                pub_message(f_message);
            }
            else
            {
                throw std::runtime_error("CRosTopicPub::publish() : Publisher not configured for publishing once");
            }
        };
    };
} // namespace godot
