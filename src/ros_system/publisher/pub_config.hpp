#pragma once

#include <string>
#include "pub_sim_config.hpp"
#include <functional>


/**
 * Parameters to configure ROS publisher node
*/
template<typename t_ros_msg>
struct CROSPubConfig
{

    CPubSimConfig ros_pub_config;
    
    std::function<void (t_ros_msg& )> ros_new_data_callback;

};
