#pragma once

#include <string>
#include <cstdint>
#include <functional>
#include "subs_sim_config.hpp"

/**
 * Parameters to configure a ROS subscriber node
 */
template <typename t_subs_data>
struct CROSSubscribConfig
{

    bool enabled = false;

    std::string topic_name;

    uint32_t queue_size = 10u;

    /**
     * Callback function used by the subscriber when new data is received
     */
    std::function<void(t_subs_data &)> new_data_func;
};
