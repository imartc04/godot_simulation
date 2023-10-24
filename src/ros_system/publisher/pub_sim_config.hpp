#pragma once

#include <string>
#include "common/topic_types.hpp"

/**
 * Parameters to configure ROS publisher node
*/
struct CRosSimPubConfig
{
    bool enabled = false;
    std::string topic_name;
    float pub_rate;

    uint32_t queue_size = 10u;

    // EGODOT_ROS_TYPES topic_type;

};
