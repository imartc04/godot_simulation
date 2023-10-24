#pragma once
#include <functional>

template <typename t_subs, typename t_sim_data>
struct CBridgeSimSubsConfig
{
    /**
     * General parameters used to configure subscriber, the user callback goes isolated
     * as the new_data_callback field of this structure. This is because the callback used for
     * the subscribe object will be defined in the correspondent bridge object and it will make use
     * of the user callback.
     */
    struct
    {
        bool enabled = false;

        std::string topic_name;

        uint32_t queue_size = 10u;
    } sub_gen_params;

    // Funtion to parse from subscriber type to t_sim_data
    std::function<void(typename t_subs::t_subs_msg &, t_sim_data &)> parsing_func;

    // User's new received data callback
    std::function<void(t_sim_data &)> new_data_callback;
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
    typename t_subs::t_msg t_subs_msg;
    typename t_subs::t_config t_subs_config;

    CBridgeSimSubs(/* args */){};
    ~CBridgeSimSubs(){};

    void set_config(CBridgeSimSubsConfig<t_subs, t_sim_data> &f_config)
    {
        m_config = f_config;

        typename t_subs::t_config l_config = {f_config.sub_gen_params.enabled, f_config.sub_gen_params.topic_name, f_config.sub_gen_params.queue_size, std::bind(&CBridgeSimSubs<t_subs, t_sim_data>::subs_callback, this, std::placeholders::_1)};
        m_subscriber.set_config(l_config);

    };

    void init()
    {
        m_subscriber.init();
    };

private:
    // Subscriber object
    t_subs m_subscriber;

    // Sim data object used to retrieve data
    t_sim_data m_sim_data;

    // Config
    CBridgeSimSubsConfig<t_subs, t_sim_data> m_config;

    void subs_callback(t_subs_msg &f_msg)
    {

        // Parse t_subs_msg to t_sim_data
        m_config.parsing_func(f_msg, m_sim_data);

        // Call user callback with t_sim_data
        m_config.new_data_callback(m_sim_data);
    };
    
};
