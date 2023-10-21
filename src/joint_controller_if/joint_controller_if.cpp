#include "joint_controller_if.hpp"

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

void CJointControllerIf::write_new_joint_reads(CJointData &f_new_reads)
{

    m_joint_reads.mtx.lock();

    m_joint_reads.data = f_new_reads;

    m_joint_reads.mtx.unlock();
}

CJointData CJointControllerIf::get_last_joint_commands()
{
    std::unique_lock<std::shared_mutex> l_lock(m_pub_buffer->mtx);
    return *m_pub_buffer->data;

}