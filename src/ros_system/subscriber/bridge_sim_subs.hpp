#pragma once
#include <functional>

template <typename t_subs, typename t_sim_data>
struct CBridgeSimSubsConfig
{

    //User's new received data callback
    std::function<void (t_sim_data& )> new_data_callback;

    typename t_subs::t_subs_config subscriber_config;

    //Funtion to parse from subscriber type to t_sim_data
    std::function<void (typename t_subs::t_subs_msg& , t_sim_data &)> parsing_func;


};

/**
 * Class created to separate subscriber implementation code from
 * simulation user one. This ways conflicts between ROS and ROS2
 * can be avoided.
 */
template <typename t_subs, typename t_sim_data>
class CBridgeSimSubs
{

public:
    typename t_subs::t_subs_msg t_subs_msg;
    typename t_subs::t_subs_config t_subs_config;

    CBridgeSimSubs(/* args */){};
    ~CBridgeSimSubs(){};

private:
    // Subscriber object
    t_subs m_subscriber;

    // Sim data object used to retrieve data
    t_sim_data m_sim_data;

    //Config
    CBridgeSimSubsConfig<t_subs, t_sim_data> m_config;

    void subs_callback(t_subs_msg &f_msg){
        
        // Parse t_subs_msg to t_sim_data
        m_config.parsing_func(f_msg, m_sim_data);

        // Call user callback with t_sim_data
        m_config.new_data_callback(m_sim_data);

    };
    
};
