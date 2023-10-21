#pragma once

#include <shared_mutex>
#include <memory>
#include <mutex>

/**
 * Structure provided by publishers to users in order to update data to publish
 * Only 1 structure is associated to 1 publisher
 */
template <typename t_data>
class CPubBufferIf
{

public:
    template <typename t_data, typename t_ros_msg>
    friend class CBridgeRosPub;

    void writeData(t_data const &f_data)
    {

        *m_writeP = f_data;

        // Swap read and write buffers
        m_mutex->lock();

        t_data *aux = *m_readP;

        *m_readP = m_writeP;

        m_writeP = aux;

        m_mutex->unlock();
    };

private:
    // Pointer to the publisher
    std::mutex *m_mutex;

    // Pointer to pointer used to publish data by the publisher
    t_data **m_readP;

    t_data *m_writeP;
};
