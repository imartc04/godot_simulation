#pragma once

#include <string>
#include <cstdint>
#include <functional>

/**
 * Parameters to configure a ROS subscriber node
*/
struct CRosSimSubsConfig
{
    
    bool enabled = false;

    std::string topic_name;

    uint32_t queue_size = 10u;

};
