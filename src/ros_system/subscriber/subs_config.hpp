#pragma once

#include <string>
#include <cstdint>
#include <functional>
#include "subs_sim_config.hpp"

/**
 * Parameters to configure a ROS subscriber node
*/
template<typename t_subs_data>
struct CROSSubscribConfig
{

    CRosSubsSimConfig gen_config;

    /**
     * Callback function used by the subscriber when new data is received
    */
    std::function< void  (t_subs_data const & )> new_data_func;

};
