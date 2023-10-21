#pragma once

#include <string>


/**
 * Parameters to configure ROS publisher node
*/
struct CRosSimPubConfig
{
    bool enabled = false;
    std::string topic_name;
    float pub_rate;

};
