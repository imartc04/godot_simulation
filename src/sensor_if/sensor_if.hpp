#pragma once

#include "ros_system/publisher/pub_sim_config.hpp"
#include "common/bridge_sim_pub.hpp"
#include <functional>

template <typename t_data>
struct CSensorIfConfig
{

    CRosSimPubConfig ros_pub_config;

    std::function< std::shared_ptr<CPubBufferIf<t_data>> (CRosSimPubConfig const & ) > ros_pub_gen_func;  
    // CROS2PubConfig ros2_pub_config;
};

template <typename t_data>
class CSensorIf
{

public:
    CSensorIf(/* args */){

    };
    ~CSensorIf(){

    };

    void update_data(t_data const &f_data)
    {
        // Update buffer data
        m_pub_if->writeData(f_data);
    };

    void set_config(CSensorIfConfig<t_data> &f_config)
    {
        m_config = f_config;
    };

    void init()
    {

        if(m_config.ros_pub_config.enabled)
        {
            //Create ROS publisher through ROS shared library API
            m_pub_if = m_config.ros_pub_gen_func(m_config.ros_pub_config);   
        }

    };

private:
    /************************************** VARIABLES  **************************************/

    CSensorIfConfig<t_data> m_config;

    /**
     * Interface object with publisher. We write new data here and then it is published through 
     * the selected method
     */
    std::shared_ptr<CPubBufferIf<t_data>> m_pub_if;

    /************************************** METHODS  **************************************/
    
};
