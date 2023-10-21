#pragma once

#include <string>

/**
 * Parameters to configure ROS publisher node
*/
struct CROS2PubConfig
{
    bool enabled = false;
    std::string topic_name;
    float pub_rate;


};
