#pragma once

#include <string>
#include <cstdint>
#include<functional>

/**
 * Parameters to configure a ROS subscriber node
*/
template<typename t_data>
struct CROS2SubscribConfig
{
    
    bool enabled = false;

    std::string topic_name;

    uint32_t queue_size = 10u;
    
    /**
     * Callback function used by the subscriber when new data is received
    */
    std::function< void  (t_data const & )> new_data_func;

};
