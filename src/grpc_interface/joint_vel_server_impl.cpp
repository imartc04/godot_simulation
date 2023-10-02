#include "joint_vel_server_impl.hpp"

//using namespace ::godot_grpc::joint_vel;

JointVelServerImpl::JointVelServerImpl()
{
}

JointVelServerImpl::~JointVelServerImpl()
{
}

void JointVelServerImpl::set_new_joint_reads(std::vector<double> const &f_values)
{
    std::unique_lock<std::shared_mutex> l_lock(m_mtx);
    // std::lock_guard<std::mutex> l_lock(m_mtx);
    m_joint_vels_read = f_values;
}

std::vector<double> JointVelServerImpl::get_joint_commands()
{

    std::shared_lock<std::shared_mutex> l_lock(m_mtx);
    // std::unique_lock<std::mutex> l_lock(m_mtx);
    // std::lock_guard<std::mutex> l_lock(m_mtx);

    return m_joint_vels_cmd;
}

::grpc::Status JointVelServerImpl::setJointCommand(::grpc::ServerContext *context, const ::godot_grpc::joint_vel::jointVelocitiesMsg *request, ::godot_grpc::emptyMsg *response)
{

    auto l_req_data = request->vels();
    auto l_num_data = l_req_data.size();

    m_joint_vels_cmd.resize(l_num_data);

    for (int64_t i_idx = 0; i_idx < l_num_data; i_idx++)
    {
        m_joint_vels_cmd[i_idx] = l_req_data[i_idx];
    }

    return ::grpc::Status::OK;
}

::grpc::Status JointVelServerImpl::readJointData(::grpc::ServerContext *context, const ::godot_grpc::emptyMsg *request, ::godot_grpc::joint_vel::jointVelocitiesMsg *response)
{

    //*response = m_joint_vels_read;

    auto l_num_data = m_joint_vels_read.size();

    response->mutable_vels()->Resize(l_num_data, 0u);

    for (int64_t i_idx = 0; i_idx < l_num_data; i_idx++)
    {
        response->set_vels(i_idx, m_joint_vels_read[i_idx]);
    }

    return ::grpc::Status::OK;
}
