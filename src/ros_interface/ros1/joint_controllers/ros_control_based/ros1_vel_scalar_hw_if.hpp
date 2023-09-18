#pragma once

#include <godot_cpp/classes/hinge_joint3d.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "ros1_control_if.hpp"

#include <mutex>

namespace godot
{

    /**
     * Config struct for  \ref CRos1VelocityHWinterface
     *
     */
    struct CRos1VelocityHWinterfaceConfig
    {
        /** Name of the controlled variable*/
        std::string control_var_name;

        /** Index referencing the variable used to read/write data in the order define in \ref Ros1ControlIf::set_new_joint_reads */
        uint16_t control_var_index;
    };

    /**
     * Class that interacts with ROS1_control package to manage 1 dof of a joint
     *
     * The class inherits from \ref Ros1ControlIf to set up the commmon interface used by
     * \ref CGeneric6DOFController and \ref CHingeJoint3DController
     *
     * The class \ref hardware_interface::RobotHW is used to interact with ROS1_control package. See [ros_control](http://wiki.ros.org/ros_control)
     * docs for more info
     */
    class CRos1VelocityHWinterface : public Ros1ControlIf, public hardware_interface::RobotHW
    {

    public:
        CRos1VelocityHWinterface(){};

        ~CRos1VelocityHWinterface(){};

        void init();

        /**
         * Set new joint commands
         *
         * From the joint control inteface passed vector only necessary to modify the
         * target velocity indicated by \ref CRos1VelocityHWinterfaceConfig::control_var_index
         *
         */
        virtual void set_new_joint_reads(std::vector<double> const &) override;

        /**
         * Get joint commands
         *
         * From the joint control inteface passed vector only necessary to read the
         * target velocity indicated by \ref CRos1VelocityHWinterfaceConfig::control_var_index
         */
        virtual std::vector<double> get_joint_commands() const override;

        void set_config(CRos1VelocityHWinterfaceConfig const &config);

        // Get config
        CRos1VelocityHWinterfaceConfig &get_config();

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface m_vel_joint_if;

        /** Store for the new velocity command value*/
        double cmd;

        /** Aux variable needed to initialize interaction with ros_control*/
        double pos;

        /** Variable holding the current joint velocity value*/
        double vel;

        /** Aux variable needed to initialize interaction with ros_control*/
        double eff;

        // Configuration
        CRos1VelocityHWinterfaceConfig m_config;

        // // Control multi-threading access in velocity var
        // std::mutex m_mutex;
    };

} // namespace godot