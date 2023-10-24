#include "joint_controller_if.hpp"
#include "ros_system/godot/godot_ros_lib/ros_lib.hpp"

using namespace std;

CJointControllerIf::CJointControllerIf(/* args */)
{
}

CJointControllerIf::~CJointControllerIf()
{
}

void CJointControllerIf::set_config(CJointControllerIfConfig &f_config)
{
    m_config = f_config;
}

void CJointControllerIf::init()
{

    if (m_config.ros_pub_config.enabled)
    {
        // Create ROS publisher for CJointData
        m_ros_pub_buffer = createJointPub(m_config.ros_pub_config);
    }

    if (m_config.ros_subs_config.enabled)
    {

        CROSSubscribConfig<CJointData> l_config = {
            m_config.ros_subs_config.enabled,
            m_config.ros_subs_config.topic_name,
            m_config.ros_subs_config.queue_size,
            std::function(&CJointControllerIf::rec_new_commands_callback, std::placeholders::_1)
        };

        createJointMsgSubs(l_config);
    }
}

void CJointControllerIf::write_new_joint_reads(CJointData &f_new_reads)
{

    if(m_ros_pub_buffer)
    {
        m_ros_pub_buffer->writeData(f_new_reads);
    }

    if(m_ros2_pub_buffer)
    {
        m_ros2_pub_buffer->writeData(f_new_reads);
    }

    if(m_dds_pub_buffer)
    {
        m_dds_pub_buffer->writeData(f_new_reads);
    }
    
}

void CJointControllerIf::rec_new_commands_callback(CJointData &f_commands)
{
    m_joint_commands.mtx.lock();

    m_joint_commands.data = f_commands;
    m_joint_commands.mtx.unlock();
}

CJointData CJointControllerIf::get_last_joint_commands()
{
    std::lock_guard<std::mutex> l_lock(m_joint_commands.mtx);
    return m_joint_commands.data;
}