#pragma once

#include "interfaces/joint_data.hpp"
#include "ros_system/publisher/pub_config.hpp"
#include "ros_system/subscriber/subs_config.hpp"
#include "ros2_system/publisher/ros2_pub_config.hpp"
#include "ros2_system/subscriber/ros2_subs_config.hpp"
#include "interfaces/pub_buffer_if.hpp"
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <memory>

struct CJointControllerIfConfig
{

    /**
     * Callback for new received data from other system
     */
    // std::function<void(CJointData &)> new_rec_data_func;

    CROS2PubConfig ros2_pub_config;
    CROSPubConfig ros_pub_config;

    CROSSubscribConfig ros_subs_config;
    CROS2SubscribConfig ros2_subs_config;
};

class CJointControllerIf
{

public:
    CJointControllerIf(/* args */);
    ~CJointControllerIf();

    void set_config(CJointControllerIfConfig &f_config);

    void write_new_joint_reads(CJointData &f_new_reads);

    CJointData get_last_joint_commands();

private:
    CJointControllerIfConfig m_config;

    // Last joint reads data
    struct
    {
        CJointData data;
        std::mutex mtx;
    } m_joint_reads;

    //Pointer to current publisher buffer data
    std::shared_ptr<CPubBufferIf<CJointData>> m_pub_buffer;
};
