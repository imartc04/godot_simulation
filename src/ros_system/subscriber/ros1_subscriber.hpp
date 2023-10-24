

#pragma once

#include <string>
#include <functional>
#include <ros/ros.h>
#include <thread>
#include <cstdint>

#include "subs_config.hpp"



template <typename t_ros1_msg>
class CROS1Subscriber 
{

private:
    /******************* VARIABLES *******************/
    /*
     *Thread to spin the ros1 node
     */
    std::thread m_ros1_spin_thread;
    
    // config
    CROSSubscribConfig<t_ros1_msg> m_config;

    // ROS1 node handle
    ros::NodeHandle m_node_handle;

    /******************* METHODS *******************/

public:

    typedef CROSSubscribConfig<t_ros1_msg> t_config;
    typedef t_ros1_msg t_msg; 

    CROS1Subscriber(/* args */){

    };

    ~CROS1Subscriber(){

        // Join thread
        if (m_ros1_spin_thread.joinable())
        {
            ros::shutdown();

            m_ros1_spin_thread.join();
        }

    };


    void set_config(CROSSubscribConfig<t_ros1_msg> & f_config)
    {
        m_config = f_config;
    }


    /**
     * Thread method
     */
    void ros1_spin_thread_method()
    {
        ros::spin();
    }

    void stop()
    {
        // Stop ros1 node
        ros::shutdown();
    }

    void init() 
    {

        // Check if ros is initialized
        if (!ros::isInitialized())
        {
            int l_argc = 0;
            char **l_argv = NULL;
            // Init ros1 node
            ros::init(l_argc, l_argv, m_config.node_name);
        }

        // Create node handle
        m_node_handle = ros::NodeHandle();

        // Create subscriber
        m_subscriber = m_node_handle.subscribe(m_config.topic_name, m_config.queue_size, m_config.callback);

        // Spin thread
        if (m_config.spin_thread)
        {
            m_ros1_spin_thread = std::thread(&CROS1Subscriber::ros1_spin_thread_method, this);
        }
    }

    /**
     * Method to spin once the ros1 node when not using the spin thread mode
    */
    void spin_once()
    {
        ros::spinOnce();
    }
};
