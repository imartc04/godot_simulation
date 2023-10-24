#pragma once

#include "interfaces/joint_data.hpp"
#include "ros_system/publisher/pub_sim_config.hpp"
#include "ros_system/subscriber/subs_config.hpp"
#include "common/pub_buffer_if.hpp"
#include "common/joint_data.hpp"
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <memory>

struct CJointControllerIfConfig
{

    // CROS2PubConfig ros2_pub_config;
    CRosSimPubConfig ros_pub_config;

    struct
    {
        bool enabled = false;

        std::string topic_name;

        uint32_t queue_size = 10u;

    } ros_subs_config;

    // CROS2SubscribConfig ros2_subs_config;
};

class CJointControllerIf
{

public:
    CJointControllerIf(/* args */);
    ~CJointControllerIf();

    void set_config(CJointControllerIfConfig &f_config);

    void init();

    void write_new_joint_reads(CJointData &f_new_reads);

    CJointData get_last_joint_commands();

private:
    CJointControllerIfConfig m_config;

    /**
     * Last joint commands, protected with mutex due
     * multiple access from different subscribers can happen
     */
    struct
    {
        CJointData data;
        std::mutex mtx;
    } m_joint_commands;

    // Pointers to possible publisher buffer data
    std::shared_ptr<CPubBufferIf<CJointData>> m_ros_pub_buffer;

    std::shared_ptr<CPubBufferIf<CJointData>> m_ros2_pub_buffer;

    std::shared_ptr<CPubBufferIf<CJointData>> m_dds_pub_buffer;


    /********************************************** METHODS**********************************************/

    void rec_new_commands_callback(CJointData& f_commands);

};
