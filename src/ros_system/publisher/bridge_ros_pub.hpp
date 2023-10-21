#pragma once

#include <array>
#include <memory>
#include "interfaces/pub_buffer_if.hpp"
#include <functional>

template <typename t_sim_data, typename t_pub_type>
struct CBridgeRosPubConfig
{
    /**
     * Publisher configuration object
     */
    typename t_pub_type::t_config pub_config;

    /**
     * Function that parses sim related data to its equivalent used by publisher object
     */
    std::function < void(t_sim_data &, typename t_pub_type::t_pub_msg &) parse_sim_to_pub_data;
};

template <typename t_data, typename t_pub_type>
class CBridgeRosPub
{

public:
    typename t_pub_type::t_pub_msg t_pub_msg;


    CBridgeRosPub()
    {

        m_read = &m_data[0];

        m_pubIf = std::make_shared<CPubBufferIf<t_data>>();

        m_pubIf->m_mutex = &m_mutex;
        m_pubIf->m_readP = &m_read;

        if (m_read = &m_data[0])
        {
            m_pubIf->m_writeP = &m_data[1];
        }
        else
        {
            m_pubIf->m_writeP = &m_data[0];
        }
    };

    ~CBridgeRosPub(){

    };

    std::shared_ptr<CPubBufferIf<t_data>> getPubif()
    {
        return m_pubIf;
    };

private:
    // Publisher object
    t_pub_type m_pub;

    std::array<t_data, 2> m_data;

    /**
     * Interface for class users to write new data for the publisher
     *
     */
    std::shared_ptr<CPubBufferIf<t_data>> m_pubIf;

    /**
     * Pointer used when processing new data for the ROS publisher
     */
    t_data * m_read;

    // Mutex used to sync read and write buffers
    std::mutex m_mutex;

    // Config
    CBridgeRosPubConfig<t_data, t_pub_type> m_config;

    /********************************** METHODS **********************************/

    void set_config(CBridgeRosPubConfig &f_config)
    {
        m_config = f_config;
        m_pub.set_config(m_config.pub_config);
    }

    void init()
    {

        //Create pub if
        m_pubIf = std::make_shared<CPubBufferIf<t_data>>();

        m_pub.init();
    }

    void new_data_pub_callback(t_pub_msg &f_msg)
    {
        m_mutex.lock();
        // Parse last generated data into pub data
        m_config.parse_sim_to_pub_data(*m_read, f_msg);

        m_mutex.unlock();
    }
};
