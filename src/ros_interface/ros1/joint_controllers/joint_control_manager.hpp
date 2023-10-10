#pragma once

#include "ros_interface/ros1/joint_controllers/ros1_control_if.hpp"
#include "grpc_interface/grpc_subprocess_manager.hpp"

#include "grpc_interface/joint_control_server_impl.hpp"

#include <chrono>

#include <mutex>

namespace godot
{

    /**
     * Config struct for  \ref CRos1VelocityHWinterface
     *
     */
    struct CJointControlManagerConfig
    {
        /** Name of the controlled variable*/
        std::vector<std::string> control_var_names;

        /**
         * Period in ms to update commands and control reads
         */
        float update_period_ms;

        ::godot_grpc::CGRPCSubprocessConfig grpc_config;
        
    };

    /**
     * Class that control
     *
     * The class inherits from \ref Ros1ControlIf to set up the commmon interface used by
     * \ref CGeneric6DOFController and \ref CHingeJoint3DController
     *
     *
     */
    class CJointControlManager : public ControlIf
    {

    public:
        CJointControlManager(){};

        ~CJointControlManager(){};

        void init();

    
        void set_new_joint_reads(t_joint_values const & f_vaues);

        t_joint_values get_new_commands();


        void set_config(CJointControlManagerConfig const &config);

        // Get config
        CJointControlManagerConfig &get_config();

    private:
        ::godot_grpc::CGRPCSubprocess m_process;

        //Joint control server 
        std::unique_ptr<ControlJointServerImpl> m_joint_control_server;

        // Configuration
        CJointControlManagerConfig m_config;

        // // Control multi-threading access in velocity var
        // std::mutex m_mutex;
    };

} // namespace godot