#pragma once

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
                        auto l_message = m_config.f_get_message();

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
