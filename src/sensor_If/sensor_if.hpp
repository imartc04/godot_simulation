#pragma once

#include "ros_system/publisher/pub_config.hpp"
#include "ros2_system/publisher/ros2_pub_config.hpp"
#include "interfaces/pub_buffer_if.hpp"

template <typename t_data>
struct CSensorIfConfig
{

    CROSPubConfig ros_pub_config;
    CROS2PubConfig ros2_pub_config;
    CPubBufferIf<t_data> pub_buffer;
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

        m_config.pub_buffer->mtx.lock();
        *(m_config.pub_buffer->data) = f_data;

        m_config.pub_buffer->mtx.unlock();
    }

private:
    /************************************** VARIABLES  **************************************/

    CSensorIfConfig<t_data> m_config;

    /************************************** METHODS  **************************************/
};
